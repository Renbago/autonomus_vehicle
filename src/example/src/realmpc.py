#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import rospy
import cv2
import numpy as np
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
from geometry_msgs.msg import PoseStamped
import serial
import time
import math
from math import atan2
from std_msgs.msg import String
import json
from nav_msgs.msg import Odometry
import xml.etree.ElementTree as ET
import pandas as pd
from utils.msg import localisation, IMU

from casadi import sin, cos, pi
import casadi as ca
import pandas as pd

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from example.msg import MekatronomYolo
#from ultralytics import YOLO
from obstacle_detector.msg import Obstacles
import sys

class mpc():
    # ===================================== INIT==========================================
    def __init__(self,camera_topic_name,pose_topic_name):
        """
        Creates a bridge for converting the image from Gazebo image intro OpenCv image
        """
        
        #Parameters
        self.debug = rospy.get_param('~debug', True)
        self.hue_min = rospy.get_param('~hue_min', 0)
        self.hue_max = rospy.get_param('~hue_max', 60)
        self.sat_min = rospy.get_param('~sat_min', 0)
        self.sat_max = rospy.get_param('~sat_max', 255)
        self.val_min = rospy.get_param('~val_min', 70)
        self.val_max = rospy.get_param('~val_max', 255)
        # self.graphml_file_path = rospy.get_param('~graphml_file_path', '/home/mekatronom/boschmekatronom_ws/BoschMekatronom/src/example/src/Competition_track_graph.graphml')
        self.PathSituations = False
        self.mpc_started = False
        self.localisation_CB = False
        self.IMU_cb = False
    
        #For checking behaviour tree
        self.last_update_time = rospy.Time.now()
        self.last_update_time_obstacles_checking = rospy.Time.now()
        self.last_path_index  = 0
        self.last_target_update_time = None
        self.current_target = None

        self.yolo_data = MekatronomYolo()

        self.state = "keep lane"
        # self.intercept_targets = ['416','440','470', '486', '58', '6', '42', '20', '26', '58','451','452','454','453','455','456']
        self.intercept_targets = []
        self.distance_flag = False
        self.yolo_intercept_flag = False
        self.park_scenerio = False

        self.prev_time = rospy.Time.now().to_sec()      
        self.bridge = CvBridge()

        #sükrü pathfinding icin
        self.source_node="800"
        self.target_node="459"
        #newPathFinding
        self.graphml_file_path = rospy.get_param('~graphml_file_path', '/home/mekatronom/boschmekatronom_ws/BoschMekatronom/src/example/src/fixed.graphml')#sollamalı 
        self.file_path_original ='/home/mekatronom/boschmekatronom_ws/BoschMekatronom/src/example/src/gercek.graphml'
        self.obs_dontuse=["489","127","96"]#kullanma listesi
        self.yolvar=False
        self.parking_nodes_id = ["900","901","902","903"]
        self.parking_spot_is_full = []
        self.parkings_are_available =[]
        self.expath = []

        self.flagsolla = 0
        self.obstaclesayac = 0
        self.center_x = []
        self.center_y = []
        self.obstacles_array = []
        self.past_obs_dontuse = []
        self.pathGoalsYawDegreecalled = False
        self.process_and_publish_data("800","459")#burda çağırılmış ilk defa her çıkmaz sokakta yine çağırıcam 

        ##############################3sükrü    
        #ForPathFinding
        self.file_path = rospy.get_param('~file_path', self.graphml_file_path)
        self.data_pub = rospy.Publisher('~graph_data', String, queue_size=10)
        
        # self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        
        #subscribers
        # self.rgb=rospy.Subscriber(camera_topic_name,Image,self.image_callback)
        self.pose=rospy.Subscriber(pose_topic_name,PoseStamped,self.pose_callback)
        self.yolo_intercept_sub = rospy.Subscriber('/automobile/yolo', MekatronomYolo, self.yolo_intercept_callback)
        self.obstacleDetector_sub = rospy.Subscriber('/obstacles', Obstacles, self.obstacleDetector_callback)

        #timerCB
        self.behaviourTimer = rospy.Timer(rospy.Duration(0.5), self.behaviourTimerCallback)
        self.mpcTimer = rospy.Timer(rospy.Duration(0.05), self.mpcTimerCallback)

        print("mpc node")
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f"Device: {self.device}")


    def DM2Arr(self,dm):
        return np.array(dm.full())
    
    def shift_timestep(self):
        
        # rospy.loginfo('shift_time_step')

        self.state_init = ca.DM([self.position_x, self.position_y,  self.yaw_rad])  
        #print("self.u[1, :].T",self.u[1, :].T)
        f_value = self.f(self.state_init,self.u[0, :].T)

        # print("f_Value",f_value)
        self.next_state = ca.DM.full(self.state_init + (self.step_horizon * f_value))
        # print("self.initial_state",self.state_init)
        #print("self.next_state",self.next_state)
        # print("self.staate_init",self.state_init)
        # print("self.state_target",self.state_target)
        self.t0 = self.t0 + self.step_horizon
        self.u0 = ca.vertcat(self.u[1:, :], self.u[-1, :])
        
        #print("self.u0",self.u0)

    def mpc_start_settings(self):
        # print("mpcsettings")

        if self.park_scenerio:
            rospy.loginfo("SETTINGS TO PARK PARAMS FOR MPC")
            Q_x = rospy.get_param('~Q_x', 1)
            Q_y = rospy.get_param('~Q_y', 1)
            Q_theta = rospy.get_param('~Q_theta', 0.1)
            R1 = rospy.get_param('~R1', 0.02)
            R2 = rospy.get_param('~R2', 0.08)
            step_horizon = rospy.get_param('~step_horizon', 0.1)
            N = rospy.get_param('~N', 16)
            rob_diam = rospy.get_param('~rob_diam', 0.354)
            wheel_radius = rospy.get_param('~wheel_radius', 1)
            L_value = rospy.get_param('~Lx', 0.3)
            Ly = rospy.get_param('~Ly', 0.04)
            v_max = rospy.get_param('~v_max', 0.16)
            v_min = rospy.get_param('~v_min', -0.25)
            omega_max = rospy.get_param('~omega_max', pi/7.5) #degree
            omega_min = rospy.get_param('~omega_min', -pi/7.5) #degree
            self.iteration = 0
            self.steeringRad = 0.0     

        else:
            rospy.loginfo("INITIAL KEEP LANE PARAMS FOR MPC")

            Q_x = rospy.get_param('~Q_x', 1)
            Q_y = rospy.get_param('~Q_y', 1)
            Q_theta = rospy.get_param('~Q_theta', 1)
            R1 = rospy.get_param('~R1', 1.0)
            R2 = rospy.get_param('~R2', 1.8)
            step_horizon = rospy.get_param('~step_horizon', 0.2)
            N = rospy.get_param('~N', 12)
            rob_diam = rospy.get_param('~rob_diam', 0.354)
            wheel_radius = rospy.get_param('~wheel_radius', 1)
            L_value = rospy.get_param('~Lx', 0.4)
            Ly = rospy.get_param('~Ly', 0.04)
            v_max = rospy.get_param('~v_max', 0.22)
            v_min = rospy.get_param('~v_min', -0.1)
            omega_max = rospy.get_param('~omega_max', pi/7.5) #degree
            omega_min = rospy.get_param('~omega_min', -pi/7.5) #degree
            self.iteration = 0
            self.steeringRad = 0.0
        
        rotationMatrix = np.array([[cos(self.yaw_rad),-sin(self.yaw_rad)],
                [sin(self.yaw_rad),cos(self.yaw_rad)]])
        matrix = np.array([1,4]) 
        self.Q_x,self.Q_y = np.dot(rotationMatrix,matrix)

        print("Q_x",Q_x,"Q_y", Q_y )
        
        ######eklenen yeni kod:

        nodes_x = np.array([node[1] for node in self.pathGoalsYawDegree])
        nodes_y = np.array([node[2] for node in self.pathGoalsYawDegree])
        
        distances = np.sqrt((nodes_x - self.position_x)**2 + (nodes_y - self.position_y)**2)
        min_distance, index = min((val, idx) for (idx, val) in enumerate(distances))
        closest_node_id = self.pathGoalsYawDegree[index][0]
        self.last_path_index = index        

        # current_id = closest_node_id
        if self.park_scenerio:
            current_id = self.current_id
            print("parking scenerio")

        else:
            current_id = "800"    # yeni eklediğim current_id 

        matching_pairs = [pair for pair in self.SourceTargetNodes if pair[0] == current_id]
        next_id = matching_pairs[0][1] 
        matching_entry = next((entry for entry in self.pathGoalsYawDegree if entry[0] == next_id), None)
        print("matching_entry",matching_entry)
        target_x, target_y = matching_entry[1], matching_entry[2]
        target_y = target_y
        #print("target_x",target_x)
        #print("target_y",target_y)
        dx = target_x - self.position_x
        dy = target_y - self.position_y
        yaw = atan2(dy, dx)
        ######

        state_init = ca.DM([self.position_x, self.position_y, self.yaw_rad])        # initial state
        state_target = ca.DM([target_x, target_y, yaw])

        #bu eklediğim state_target kısmıyla buradan state_target girmene gerek kalmaması lazım. sadece zed'e konum girsen olur.


        #state_target = ca.DM([6.07, 13.72-13.04, 0.0])  # target state
        #state_target = ca.DM([9.1, 13.72-13.07, 0.0])   #parking scenerio
        #state_target = ca.DM([0.5, 0.0, 0.0])
        
        # state symbolic variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(
            x,
            y,
            theta
        )
        n_states = states.numel()

        #new define 
        v = ca.SX.sym('v')
        omega = ca.SX.sym('omega')
        controls = ca.vertcat(
            v,
            omega
        )
        n_controls = controls.numel()
        # print("n_controls",n_controls)

        # matrix containing all states over all time steps +1 (each column is a state vector)
        X = ca.SX.sym('X', n_states, (N + 1))

        # matrix containing all control actions over all time steps (each column is an action vector)
        U = ca.SX.sym('U', n_controls, N)
        # print(U)
        # coloumn vector for storing initial state and target state
        P = ca.SX.sym('P', n_states + n_states)

        # state weights matrix (Q_X, Q_Y, Q_THETA)
        Q = ca.diagcat(Q_x, Q_y, Q_theta)


        # controls weights matrix
        R = ca.diagcat(R1, R2)

        # discretization model (e.g. x2 = f(x1, v, t) = x1 + v * dt)
        rot_3d_z = ca.vertcat(
            ca.horzcat(cos(theta), -sin(theta), 0),
            ca.horzcat(sin(theta),  cos(theta), 0),
            ca.horzcat(         0,           0, 1)
        )
        # Mecanum wheel transfer function which can be found here: 
        # https://www.researchgate.net/publication/334319114_Model_Predictive_Control_for_a_Mecanum-wheeled_robot_in_Dynamical_Environments
        # J = (wheel_radius/4) * ca.DM([
        #     [         1,         1,          1,         1],
        #     [        -1,         1,          1,        -1],
        #     [-1/(Lx+Ly), 1/(Lx+Ly), -1/(Lx+Ly), 1/(Lx+Ly)]
        # ])
        # RHS = states + J @ controls * step_horizon  # Euler discretization
        #RHS = rot_3d_z @ J @ controls

        L = ca.SX.sym('L')

        RHS = ca.vertcat(
            v * ca.cos(theta),
            v * ca.sin(theta),
            v / L_value * ca.tan(omega)
        )
        # maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T
        f = ca.Function('f', [states, controls], [RHS])


        cost_fn = 0  # cost function
        g = X[:, 0] - P[:n_states]  # constraints in the equation
        
        st = X[:,0]
        # print("st",st)
        # runge kutta
        #print(P[3:6])
        print("g",g)
        print("st",st)

        for k in range(N):
            st = X[:, k]
            con = U[:, k]
            print("con",con)
            print("st",st)
            #print(R)
            #
            cost_fn = cost_fn + ca.mtimes(ca.mtimes((st - P[3:6]).T, Q), (st - P[3:6])) + ca.mtimes(ca.mtimes(con.T, R), con)

            st_next = X[:, k+1]
            #print("st_next",st_next)
            k1 = f(st, con)
            k2 = f(st + (step_horizon/2)*k1, con)
            k3 = f(st + (step_horizon/2)*k2, con)
            k4 = f(st + step_horizon * k3, con)


            st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            print("st_next_RK4",st_next_RK4)

            g = ca.vertcat(g, st_next - st_next_RK4)

        obs_x = -3.0
        obs_y = -3.0
        obs_diam = 0.3
        print(g.shape)

        #Engelden Kaçma constraints eklenmesi

        # for k in range(N+1):  # In Python, range goes from 0 to N, so we use range(N+1) for 1 to N+1
        #     # Calculate the squared distance from the robot to the obstacle
        #     distance_squared = (X[0, k] - obs_x)**2 + (X[1, k] - obs_y)**2
        #     # The constraint ensures the robot stays outside the sum of the semi-diameters
        #     constraint = -ca.sqrt(distance_squared) + (rob_diam/2 + obs_diam/2)
        #     # Append the constraint to the list
        #     g = ca.vertcat(g,constraint)
            
        print(g.shape)    
          # Vertically concatenate all constraints into a CasADi vector
        #print("g",g)
        #print("st",st)
        #print("con",con)

        OPT_variables = ca.vertcat(   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
            ca.reshape(X,3*(N+1),1),
            ca.reshape(U,2*N, 1)
        )
        #print("opt_Variables",OPT_variables)

        nlp_prob = {
            'f': cost_fn,
            'x': OPT_variables,
            'g': g,
            'p': P
        }

        opts = {
            'ipopt': {
                'max_iter': 100,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }

        #print("nlp_prob",nlp_prob)

        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        print("solver",solver)

        lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
        ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
        lbg = ca.DM.zeros((n_states*(N+1), 1))
        ubg = ca.DM.zeros((n_states*(N+1), 1))

        # lbg[n_states*(N+1)+1: 3*(N+1) + (N+1)] = -ca.inf
        # ubg[n_states*(N+1)+1: 3*(N+1) + (N+1)] = 0

        lbx[0: n_states*(N+1): n_states] = -ca.inf     # X lower bound
        lbx[1: n_states*(N+1): n_states] = -ca.inf     # Y lower bound
        lbx[2: n_states*(N+1): n_states] = -ca.inf     # theta lower bound

        ubx[0: n_states*(N+1): n_states] = ca.inf      # X upper bound
        ubx[1: n_states*(N+1): n_states] = ca.inf      # Y upper bound
        ubx[2: n_states*(N+1): n_states] = ca.inf      # theta upper bound

        lbx[n_states*(N+1):n_states*(N+1)+2*N:2] = v_min                  # v lower bound for all V
        ubx[n_states*(N+1):n_states*(N+1)+2*N:2] = v_max                  # v upper bound for all V
        lbx[n_states*(N+1)+1:n_states*(N+1)+2*N:2] = omega_min                  # omega bound for all V
        ubx[n_states*(N+1)+1:n_states*(N+1)+2*N:2] = omega_max                  # omega bound for all V


        #print("lbx",lbx)
        #print("ubx",ubx)

        args = {
            'lbg': lbg,  # constraints lower bound
            'ubg': ubg,  # constraints upper bound
            'lbx': lbx,
            'ubx': ubx
        }
    
        #print('Size of lbg:', args['lbg'].shape)
        


        # print("args",args)
        self.t0 = 0

        # print("ilk_target",state_target)
        # print("ilk_konum",state_init)

        # xx = DM(state_init)
        t = ca.DM(self.t0)
        
        u0 = ca.DM.zeros(N, 2)  # initial control
        X0 = ca.repmat(state_init, 1, N+1).T         # initial state full
        #print("u0",u0)

        cat_states = self.DM2Arr(X0)
        #cat_controls = self.DM2Arr(u0[0, :])
        cat_controls = np.array([]).reshape(0, u0.shape[1])

        self.times = np.array([[0]])

        # print("ilk_u0",u0)
        # print("ilk_X0",X0)        
        # print("state_init",state_init)
        # print("cat_states",cat_states)
        # print("cat_controls",cat_controls)

        self.step_horizon = step_horizon
        self.cat_controls = cat_controls
        self.cat_states = cat_states
        self.t = t
        self.f = f
        self.solver = solver
        self.state_init =state_init
        self.state_target = state_target
        self.n_states = n_states
        self.N = N
        self.X0 =X0
        self.args = args
        self.u0 = u0
        self.n_controls = n_controls
        self.v_min = v_min
        self.v_max = v_max
        self.omega_min = omega_min
        self.omega_max = omega_max

    def mpcTimerCallback(self,event):
        try:

            if  self.localisation_CB == True and self.IMU_cb == True:

                if not self.mpc_started:
                    print("mpc started")
                    self.mpc_start_settings()
                    self.mpc_started = True
                    print("mpc settings finished")
                self.prev_time = rospy.Time.now().to_sec()
                
                # lbx = ca.DM.zeros((self.n_states*(self.N+1) + self.n_controls*self.N, 1))
                # ubx = ca.DM.zeros((self.n_states*(self.N+1) + self.n_controls*self.N, 1))
                # lbg = ca.DM.zeros((self.n_states*(self.N+1), 1))
                # ubg = ca.DM.zeros((self.n_states*(self.N+1), 1))

                # # lbg[self.n_states*(self.N+1)+1: 3*(self.N+1) + (self.N+1)] = -ca.inf
                # # ubg[self.n_states*(self.N+1)+1: 3*(self.N+1) + (self.N+1)] = 0

                # lbx[0: self.n_states*(self.N+1): self.n_states] = -ca.inf     # X lower bound
                # lbx[1: self.n_states*(self.N+1): self.n_states] = -ca.inf     # Y lower bound
                # lbx[2: self.n_states*(self.N+1): self.n_states] = -ca.inf     # theta lower bound

                # ubx[0: self.n_states*(self.N+1): self.n_states] = ca.inf      # X upper bound
                # ubx[1: self.n_states*(self.N+1): self.n_states] = ca.inf      # Y upper bound
                # ubx[2: self.n_states*(self.N+1): self.n_states] = ca.inf      # theta upper bound

                # lbx[self.n_states*(self.N+1):self.n_states*(self.N+1)+2*self.N:2] = self.v_min                  # v lower bound for all V
                # ubx[self.n_states*(self.N+1):self.n_states*(self.N+1)+2*self.N:2] = self.v_max                  # v upper bound for all V
                # lbx[self.n_states*(self.N+1)+1:self.n_states*(self.N+1)+2*self.N:2] = self.omega_min                  # omega bound for all V
                # ubx[self.n_states*(self.N+1)+1:self.n_states*(self.N+1)+2*self.N:2] = self.omega_max                  # omega bound for all V


                # #print("lbx",lbx)
                # #print("ubx",ubx)

                # args = {
                #     'lbg': lbg,  # constraints lower bound
                #     'ubg': ubg,  # constraints upper bound
                #     'lbx': lbx,
                #     'ubx': ubx
                # }
        
                
                # self.args = args

                self.args['p'] = ca.vertcat(
                    self.state_init,    # current state
                    self.state_target   # target state
                )

                self.args['x0'] = ca.vertcat(
                    ca.reshape(self.X0.T, self.n_states*(self.N+1), 1),
                    ca.reshape(self.u0.T, self.n_controls*self.N, 1)
                )

                # print("self.args['x0']",self.args['x0'])
                # print("self.args['p']",self.args['p'])
                # print("self.u0",self.u0.T)
                # print("self.X0",self.X0.T)

                sol = self.solver(
                    x0=self.args['x0'],
                    lbx=self.args['lbx'],
                    ubx=self.args['ubx'],
                    lbg=self.args['lbg'],
                    ubg=self.args['ubg'],
                    p=self.args['p']
                )

                self.u = ca.reshape((sol['x'][self.n_states * (self.N + 1):]).T, self.n_controls, self.N).T


                # Daha sonra yeniden şekillendirin

                # print("self.u",self.u)
                
                self.cat_states = np.dstack((
                    self.cat_states,
                    self.DM2Arr(self.X0)
                ))

                self.cat_controls = np.vstack((
                    self.cat_controls,
                    self.DM2Arr(self.u[0, :])
                ))

                
                self.shift_timestep()
                #print("output_shift_time_step")
                self.X0 = ca.reshape((sol['x'][: self.n_states * (self.N+1)]).T, self.n_states, self.N+1).T
                # print("self.X0 ",self.X0)

                self.X0 = ca.vertcat(
                    self.X0[1:, :],
                    ca.reshape(self.X0[-1, :], 1, -1)
                )

                # xx ...

                # print("u",self.u)
                # print("uygulanan kontrol degree",self.DM2Arr(self.u[0, 1]))
                # self.twist.angular.z = float(self.DM2Arr(self.u[0, 1]))
                # self.twist.linear.x = float(self.DM2Arr(self.u[0, 0]))

                if self.state == "keep lane":
                    self.steerAngle = math.degrees(float(self.DM2Arr(self.u[0, 1])))
                    self.steerLateral = float(self.DM2Arr(self.u[0, 0]))
                    self.carControlPublisher()  


                #print("self.u",self.u)
                # print("self.state_init",self.state_init)
                # print("self.state_target",self.state_target)
                # print("self.next_state",self.next_state)
                #print("self.pathGoalsYawDegree:",self.pathGoalsYawDegree)

                state_target_slice = self.state_target[:2]
                state_init_slice = self.state_init[:2]

                # Calculate the norm
                distance = ca.norm_2(ca.DM(state_init_slice) - ca.DM(state_target_slice))
                

                if distance < 0.25:
                    #print("test3")
                    print("insideofdistance")

                    nodes_x = np.array([node[1] for node in self.pathGoalsYawDegree])
                    nodes_y = np.array([node[2] for node in self.pathGoalsYawDegree])
                    #print("nodes_x",nodes_x )
                    #print("nodes_y",nodes_y)

                    distances = np.sqrt((nodes_x - self.position_x)**2 + (nodes_y - self.position_y)**2)
                    min_distance, index = min((val, idx) for (idx, val) in enumerate(distances))
                    closest_node_id = self.pathGoalsYawDegree[index][0]
                    print("closest_node_id",closest_node_id)

                    self.last_path_index = index        
                    current_id = closest_node_id
                    self.current_id = current_id

                    sign_looking_band = []

                    nodes_x_for_obstacles = np.array([node[1] for node in self.pathGoalsYawDegreeCopy])
                    nodes_y_for_obstacles = np.array([node[2] for node in self.pathGoalsYawDegreeCopy])
                    #print("nodes_x",nodes_x )
                    #print("nodes_y",nodes_y)

                    distances = np.sqrt((nodes_x_for_obstacles - self.position_x)**2 + (nodes_y_for_obstacles - self.position_y)**2)
                    min_distance, index = min((val, idx) for (idx, val) in enumerate(distances))
                    current_id_for_obstacles = self.pathGoalsYawDegreeCopy[index][0]

                    for i in range(1, 4):
                        matching_pairs_signs = [pair for pair in self.SourceTargetNodesCopy if pair[0] == current_id_for_obstacles]
                        matching_entry = next((entry for entry in self.pathGoalsYawDegreeCopy if entry[0] == current_id_for_obstacles), None)
                        # print("self.path_original",self.path_original)
                        # print("self.SourceTargetNodes",self.SourceTargetNodes)
                        print("testing")    
                        print("matching_entry",matching_entry)
                        print("matcihing_pairs_signs",matching_pairs_signs)
                        if matching_pairs_signs:
                            next_id_signs = matching_pairs_signs[0][1]
                            matching_entry_second = next((entry for entry in self.pathGoalsYawDegreeCopy if entry[0] == next_id_signs), None)

                            print("matching_entry_second",matching_entry_second)
                            print("next_id",next_id_signs)
                            
                            sign_looking_band.append(current_id_for_obstacles)  
                            current_id_for_obstacles = next_id_signs 
                            
                            TargetPosition = matching_entry[1:3]
                            # TargetPositionNext = matching_entry_second[1:3]

                            x_diff = abs(matching_entry[1] - matching_entry_second[1])
                            y_diff = abs(matching_entry[2] - matching_entry_second[2])

                            x_threshold = 0.13
                            y_threshold  = 0.13

                           
                            for center_x, center_y in zip(self.center_x, self.center_y):
                                ObstaclePosition = [center_x, center_y]

                                # if in self.parking_nodes_id:
                                #     self.parking_spot_is_full.append(matching_entry[0])

                                # Calculate the norm
                                obstaclediff = np.linalg.norm(np.array(ObstaclePosition) - np.array(TargetPosition))

                                is_within_x_threshold = min(matching_entry[1], matching_entry_second[1]) - x_threshold <= center_x <= max(matching_entry[1], matching_entry_second[1]) + x_threshold
                                is_within_y_threshold = min(matching_entry[2], matching_entry_second[2]) - y_threshold <= center_y <= max(matching_entry[2], matching_entry_second[2]) + y_threshold
                                print("obstaclediff",obstaclediff)
                                print("center_x",center_x)
                                print("center_y",center_y)
                                print("is_within_x_threshold",is_within_x_threshold)
                                print("is_within_y_threshold",is_within_y_threshold)

                                if obstaclediff < 0.15 or (is_within_x_threshold and is_within_y_threshold):

                                    print("obstaclediffINSIDE",obstaclediff)
                                    print("current_idObstacle",self.current_id)
                                    print("matching_entryObstacle",matching_entry[0])
                                    
                                    self.past_obs_dontuse = self.obs_dontuse.copy()

                                    # self.obstacles_array = []  ## bu 2 kod sornadan kaldırılacaktır.
                                    # self.obs_dontuse=["489","127","96"]#kullanma listesi

                                    if matching_entry[0] not in self.obstacles_array:
                                        self.obstacles_array.append(matching_entry[0])
                                    for obstacle_id in self.obstacles_array:
                                        print("self.obs_dontuseinside",self.obs_dontuse)
                                        print("self.obstacles_arrayinside",self.obstacles_array)
                                        obstacle_id_str = str(obstacle_id)
                                        if obstacle_id_str not in self.obs_dontuse:
                                            self.obs_dontuse.insert(0, obstacle_id_str) 
                                    if self.obs_dontuse != self.past_obs_dontuse:
                                        self.process_and_publish_data(self.source_node,self.target_node)#curent den global targete göndericem
                                        self.last_update_time_obstacles_checking = rospy.Time.now()

                                    else:
                                        print("obs_dontuse same")

                                    print("self.obs_dontuse",self.obs_dontuse)
                                    print("self.obstacles_array",self.obstacles_array)

                                if self.parking_spot_is_full:
                                    self.parkings_are_available = list(set(self.parking_nodes_id) - set(self.parking_spot_is_full))
                                    print("self.parkings_are_available",self.parkings_are_available)


                                if self.obstacles_array and self.flagsolla == 0:
                                    for obstacle_id in self.obstacles_array:
                                        # Code to execute for each obstacle
                                        # ...

                                        matching_entry_obstacle_first = next((entry for entry in self.pathGoalsYawDegreeCopy if entry[0] == obstacle_id), None)
                                        matching_entry_id_second = [pair for pair in self.SourceTargetNodesCopy if pair[0] == obstacle_id]
                                        matching_entry_obstacle_second = next((entry for entry in self.pathGoalsYawDegreeCopy if entry[0] == matching_entry_id_second[0][1]), None)
                                        # print("self.pathGoalsYawDegree",self.pathGoalsYawDegree)
                                        # print("matching_entry_obstacle",matching_entry_obstacle_first)
                                        print("obstacle_id",obstacle_id)
                                        targetposition_obstacle_first = matching_entry_obstacle_first[1:3]
                                        targetposition_obstacle_second = matching_entry_obstacle_second[1:3]
                                        print("firstcalculation",np.linalg.norm(np.array(targetposition_obstacle_first) - np.array(ObstaclePosition)))
                                        print("secondcalculation",np.linalg.norm(np.array(targetposition_obstacle_second) - np.array(ObstaclePosition)))
                                        if np.linalg.norm(np.array(targetposition_obstacle_first) - np.array(ObstaclePosition)) < 0.16 or np.linalg.norm(np.array(targetposition_obstacle_second) - np.array(ObstaclePosition)) < 0.16:
                                            self.state = "waiting the obstacle move on"
                                            self.last_update_time_obstacles_checking = rospy.Time.now()
                                            self.obstaclesayac = 0

                    print("self.statefirst",self.state)
                    print("self.flagsolla",self.flagsolla)
                    self.obstaclesayac = self.obstaclesayac + 1

                    if self.state == "keep lane" and self.obstacles_array and self.flagsolla == 0 and self.obstaclesayac == 10:  
                            
                        print("hi im here")
                        print("self.obstacles_array",self.obstacles_array)
                        for value in self.obstacles_array:
                            if value in self.obs_dontuse:
                                self.obs_dontuse.remove(value)
                        self.obstacles_array = []

                        self.process_and_publish_data(self.source_node,self.target_node)#curent den global targete göndericem

                    # if any obstacle in the matching_pairs_signs create new path so call djikstra

                    if any(sign_id in self.intercept_targets for sign_id in sign_looking_band) and "stop" in self.yolo_data.object:

                        if current_id in self.intercept_targets:
                            self.steerAngle = math.degrees(0.0) 
                            self.steerLateral = float(0.0)
                            self.state = "waiting on intercept"
                            # rospy.loginfo("self.state: {}".format(self.state))
                            self.carControlPublisher()     
                        
                        else:
                            self.last_update_time = rospy.Time.now()

                    elif any(sign_id in self.intercept_targets for sign_id in sign_looking_band) and "crosswalk" in self.yolo_data.object:
                        
                        self.steerAngle = math.degrees(float(self.DM2Arr(self.u[0, 1])))
                        self.steerLateral = float(0.09)
                        # Assuming self.yolo_data.distance is an array of floats (float32[])
                        # Check if the array is not empty and then process the distances
                        distance_meets_criteria = False
                        # if self.yolo_data.distance:
                        #     # If you want to check if any distance meets your criteria
                        #     # Example: Check if any distance is within the specific range
                        #     if any(0.3 < d < 0.7 for d in self.yolo_data.distance):
                        #         distance_meets_criteria = True
                        #         print("test1")
                        #     else:
                        #         distance_meets_criteria = False
                        #         print("test2")

                        # Use distance_meets_criteria for your logic
                        # if distance_meets_criteria and current_id in self.intercept_targets:
                        #     self.steerAngle = math.degrees(float(self.DM2Arr(self.u[0, 1])))
                        #     self.steerLateral = float(0.0)
                        #     self.state = "obstacles on crosswalk"
                        # else:   
                        #     self.state = "no obstacle on crosswalk"

                        rospy.loginfo("self.state: {}".format(self.state))

                        self.carControlPublisher() 
                        print("test3")
                        self.last_update_time = rospy.Time.now()
                        print("test4")

                    # elif (current_id == '1' or current_id == "143"):
                        
                    #     distance_meets_criteria = False
                    #     if self.yolo_data.distance:
                    #         # If you want to check if any distance meets your criteria
                    #         # Example: Check if any distance is within the specific range
                    #         if any(0.3 < d < 0.8 for d in self.yolo_data.distance):
                    #             distance_meets_criteria = True
                    #             print("test1")
                    #             print("inside of 1")
                    #         else:
                    #             distance_meets_criteria = False
                    #             print("test2")
                    #     # Use distance_meets_criteria for your logic
                    #     if distance_meets_criteria: 
                    #         self.steerAngle = math.degrees(float(0.0))
                    #         self.steerLateral = float(0.0)
                    #         self.state = "obstacles on ID: 42"
                    #         rospy.loginfo("self.state: {}".format(self.state))
                    #         print("test5")
                            
                        # self.carControlPublisher()
                    #     print("test3")
                    #     self.last_update_time = rospy.Time.now()    
                    #     print("test4")

                    else:
                        self.state = "keep lane"
                        rospy.loginfo("self.state: {}".format(self.state))

                    print("yolo_data.object",self.yolo_data.object)
                    if any(sign_id in self.intercept_targets for sign_id in sign_looking_band) and "parking" in self.yolo_data.object:
                        self.mpc_start_settings()
                        self.park_scenerio = True
                        rospy.loginfo("self.state: park section id:1")

                    matching_pairs = [pair for pair in self.SourceTargetNodes if pair[0] == current_id]
                    print("matching_pairs",matching_pairs)
                    if not matching_pairs:
                        self.distance_flag = False
                        state_target_slice = self.state_target[:3]

                        state_init_slice = self.state_init[:3]

                        # Calculate the norm
                        distance = ca.norm_2(ca.DM(state_init_slice) - ca.DM(state_target_slice))
                        if distance < 0.15:
                            self.state_target = ca.DM([self.position_x, self.position_y, self.yaw_rad])
                        self.last_update_time = rospy.Time.now()
                      
                    if matching_pairs:
                        next_id = matching_pairs[0][1] 
                        # print("next_id",next_id)
                        #print("self.pathGoalsYawDegree:",self.pathGoalsYawDegree)
                        #for i in range(1, 5):
                            
                        matching_entry = next((entry for entry in self.pathGoalsYawDegree if entry[0] == next_id), None)
                        if matching_entry:
                            target_x, target_y = matching_entry[1], matching_entry[2]
                            target_y = target_y
                            #print("target_x",target_x)
                            #print("target_y",target_y)
                            dx = target_x - self.position_x
                            dy = target_y - self.position_y
                            yaw = atan2(dy, dx)
                            # print("yaw",yaw)

                            if pi < yaw:
                                changing_Value = yaw - pi
                                yaw = -pi + changing_Value

                            if -pi > yaw:
                                changing_Value = yaw + pi
                                yaw = pi + changing_Value
                            
                            if pi < self.yaw_rad:
                                changing_Value = self.yaw_rad - pi
                                self.yaw_rad = -pi + changing_Value
                            
                            if -pi > self.yaw_rad:
                                changing_Value = self.yaw_rad + pi
                                self.yaw_rad = pi + changing_Value

                            bandwidth = 1.57
                            
                            if self.yaw_rad + bandwidth > pi and yaw < -1.57:
                                cars_value = pi - self.yaw_rad
                                goal_value = pi - abs(yaw)
                                yaw = cars_value + goal_value + self.yaw_rad
                            if self.yaw_rad - bandwidth < -pi and yaw > 1.57:
                                goal_value = pi - yaw
                                yaw = -pi - goal_value

                            print(f'Step {1} -> ID: {next_id}, X: {target_x:.2f}, Y: {target_y:.2f}, Yaw: {yaw:.2f} rad')
                            
                            if next_id == '55':
                                self.distance_flag = False
                            else:
                                self.distance_flag = True

                            if next_id == '459':
                                self.state_target = ca.DM([target_x, target_y, -1.57])
                                self.distance_flag = False
                            else:
                                self.state_target = ca.DM([target_x, target_y, yaw])


                            # Engelle olan mesafeyi kontrol et
                            # is_close_to_any_obstacle = any(
                            #     np.sqrt((x0[0] - ox)**2 + (x0[1] - oy)**2) < 0.6 for ox, oy in zip(obs_x1, obs_y1)
                            # )

                            # if i == 2 and is_close_to_any_obstacle:
                            #     ObstacleDetected = 1
                            # elif i == 1 and not ObstacleDetected:
                            #     xs = np.array([target_x, target_y, yaw])
                            # elif i == 4 and ObstacleDetected:
                            #     xs = np.array([target_x, target_y, yaw])

                            # if i in [3, 1, 2, 4] and not is_close_to_any_obstacle:
                            #     ObstacleDetected = 0
                            # print("current_id",current_id)
                            # print("next_id",next_id)

                            
  
                            self.goal_id = matching_entry  
                            if self.state == "keep lane":         
                                self.last_update_time = rospy.Time.now()     

                if self.distance_flag == True and self.goal_id != '459':
                    matching_pairs = [pair for pair in self.SourceTargetNodes if pair[0] == self.current_id]
                    next_id = matching_pairs[0][1] 
                    matching_entry = next((entry for entry in self.pathGoalsYawDegree if entry[0] == next_id), None)
                    if matching_pairs:
                        target_x, target_y = matching_entry[1], matching_entry[2]
                        target_y = target_y
                        dx = target_x - self.position_x
                        dy = target_y - self.position_y
                        yaw = atan2(dy, dx)

                        if pi < yaw:
                            changing_Value = yaw - pi
                            yaw = -pi + changing_Value

                        elif -pi > yaw:
                            changing_Value = yaw + pi
                            yaw = pi + changing_Value
                        
                        if pi < self.yaw_rad:
                            changing_Value = self.yaw_rad - pi
                            self.yaw_rad = -pi + changing_Value
                        
                        elif -pi > self.yaw_rad:
                            changing_Value = self.yaw_rad + pi
                            self.yaw_rad = pi + changing_Value

                        bandwidth = 1.57
                        
                        if self.yaw_rad + bandwidth > pi and yaw < -1.57:
                            cars_value = pi - self.yaw_rad
                            goal_value = pi - abs(yaw)
                            yaw = cars_value + goal_value + self.yaw_rad
                        elif self.yaw_rad - bandwidth < -pi and yaw > 1.57:
                            goal_value = pi - yaw
                            yaw = -pi - goal_value

                        self.state_target = ca.DM([target_x, target_y, yaw])

                if self.debug:
                    self.debug_callback()

                #  State Vector [x y yaw v s]'


        except AttributeError as e:
            print(e)
            rospy.logerr(e)

    def yolo_intercept_callback(self,data):

        self.yolo_data.object = data.object
        # print("self.yolo_data.object",self.yolo_data.object)
        self.yolo_data.distance = data.distance
        # print("self.yolo_data.distance",self.yolo_data.distance)

    def behaviourTimerCallback(self,event):
        # rospy.loginfo("behaviourTimerCallback")

        if rospy.Time.now() - self.last_update_time_obstacles_checking >= rospy.Duration(10.0):
            self.last_update_time_obstacles_checking = rospy.Time.now()

            if self.obstacles_array:  
                print("hi im here behavior")
                print("self.obstacles_array",self.obstacles_array)
                for value in self.obstacles_array:
                    if value in self.obs_dontuse:
                        self.obs_dontuse.remove(value)
                self.obstacles_array = []

                self.process_and_publish_data(self.source_node,self.target_node)#curent den global targete göndericem
                self.last_update_time_obstacles_checking = rospy.Time.now()

        if rospy.Time.now() - self.last_update_time >= rospy.Duration(5.0):
            # rospy.loginfo(f"ID 8 saniye boyunca değişmedi: {self.goal_id}")
            # rospy.loginfo("behaviourTimerCallback inside")

            if self.state == "waiting on intercept":

                self.state = "keep lane"
                matching_pairs = [pair for pair in self.SourceTargetNodes if pair[0] == self.goal_id[0]]

                if matching_pairs:
                    next_id = matching_pairs[0][1] 
                    
                    #print("self.pathGoalsYawDegree:",self.pathGoalsYawDegree)
                    #for i in range(1, 5):
                        
                    matching_entry = next((entry for entry in self.pathGoalsYawDegree if entry[0] == next_id), None)
                    self.goal_id = matching_entry
                    if matching_entry:
                        target_x, target_y = matching_entry[1], matching_entry[2]
                        target_y = target_y
                        #print("target_x",target_x)
                        #print("target_y",target_y)
                        dx = target_x - self.position_x
                        dy = target_y - self.position_y
                        yaw = atan2(dy, dx)

                        if pi < yaw:
                            changing_Value = yaw - pi
                            yaw = -pi + changing_Value

                        elif -pi > yaw:
                            changing_Value = yaw + pi
                            yaw = pi + changing_Value
                        
                        if pi < self.yaw_rad:
                            changing_Value = self.yaw_rad - pi
                            self.yaw_rad = -pi + changing_Value
                        
                        elif -pi > self.yaw_rad:
                            changing_Value = self.yaw_rad + pi
                            self.yaw_rad = pi + changing_Value


                        bandwidth = 1.57
                        
                        if self.yaw_rad + bandwidth > pi and yaw < -1.57:
                            cars_value = pi - self.yaw_rad
                            goal_value = pi - abs(yaw)
                            yaw = cars_value + goal_value + self.yaw_rad
                        elif self.yaw_rad - bandwidth < -pi and yaw > 1.57:
                            goal_value = pi - yaw
                            yaw = -pi - goal_value

                        print(f'Step {1} -> ID: {next_id}, X: {target_x:.2f}, Y: {target_y:.2f}, Yaw: {yaw:.2f} rad')
                        
                        #self.state_target = ca.DM([target_x, target_y, 0.0])

                        self.state_target = ca.DM([target_x, target_y, yaw])
                self.last_update_time = rospy.Time.now()

            if self.state == "no obstacle on crosswalk":
                self.state = "no obstacle on crosswalk"
                self.last_update_time = rospy.Time.now()

            if self.state == "obstacles on ID: 42":
                self.steerAngle = math.degrees(float(self.DM2Arr(self.u[0, 1])))
                self.steerLateral = float(0.00)
                distance_meets_criteria = False
                if self.yolo_data.distance:
                    # If you want to check if any distance meets your criteria
                    # Example: Check if any distance is within the specific range
                    if any(0.3 < d < 0.7 for d in self.yolo_data.distance):
                        distance_meets_criteria = True
                    else:
                        distance_meets_criteria = False
                # Use distance_meets_criteria for your logic
                if distance_meets_criteria:
                    self.steerAngle = math.degrees(float(0.0))
                    self.steerLateral = float(0.0)
                    self.state = "obstacles on ID: 42"
                    rospy.loginfo("self.state: {}".format(self.state))
                else:
                    self.state = "keep lane"

                    
                self.carControlPublisher() 
                self.last_update_time = rospy.Time.now()


            if self.state == "obstacles on crosswalk":
                self.steerAngle = math.degrees(float(self.DM2Arr(self.u[0, 1])))
                self.steerLateral = float(0.09)
                # Assuming self.yolo_data.distance is an array of floats (float32[])
                # Check if the array is not empty and then process the distances
                distance_meets_criteria = False
                if self.yolo_data.distance:
                    # If you want to check if any distance meets your criteria
                    # Example: Check if any distance is within the specific range
                    if any(0.3 < d < 0.7 for d in self.yolo_data.distance):
                        distance_meets_criteria = True
                    else:
                        distance_meets_criteria = False

                # Use distance_meets_criteria for your logic
                if distance_meets_criteria and self.current_id in self.intercept_targets:
                    self.steerAngle = math.degrees(float(self.DM2Arr(self.u[0, 1])))
                    self.steerLateral = float(0.0)
                    self.state = "obstacles on crosswalk"
                else:   
                    self.state = "no obstacle on crosswalk"

                rospy.loginfo("self.state: {}".format(self.state))

                self.carControlPublisher() 
                self.last_update_time = rospy.Time.now()
            # else:
            #     if rospy.Time.now() - self.last_update_time >= rospy.Duration(8.0) and self.goal_id:
            #         matching_pairs = [pair for pair in self.SourceTargetNodes if pair[0] == self.goal_id[0]]
            #         print("matching_pairs",matching_pairs)

            #         if matching_pairs:
            #             next_id = matching_pairs[0][1] 
                        
            #             #print("self.pathGoalsYawDegree:",self.pathGoalsYawDegree)
            #             #for i in range(1, 5):
                            
            #             matching_entry = next((entry for entry in self.pathGoalsYawDegree if entry[0] == next_id), None)
            #             self.goal_id = matching_entry
            #             if matching_entry:
            #                 target_x, target_y = matching_entry[1], matching_entry[2]
            #                 target_y = target_y
            #                 #print("target_x",target_x)
            #                 #print("target_y",target_y)
            #                 dx = target_x - self.position_x
            #                 dy = target_y - self.position_y
            #                 yaw = atan2(dy, dx)

            #                 if pi < yaw:
            #                     changing_Value = yaw - pi
            #                     yaw = -pi + changing_Value

            #                 elif -pi > yaw:
            #                     changing_Value = yaw + pi
            #                     yaw = pi + changing_Value
                            
            #                 if pi < self.yaw_rad:
            #                     changing_Value = self.yaw_rad - pi
            #                     self.yaw_rad = -pi + changing_Value
                            
            #                 elif -pi > self.yaw_rad:
            #                     changing_Value = self.yaw_rad + pi
            #                     self.yaw_rad = pi + changing_Value


            #                 bandwidth = 1.57
                            
            #                 if self.yaw_rad + bandwidth > pi and yaw < -1.57:
            #                     cars_value = pi - self.yaw_rad
            #                     goal_value = pi - abs(yaw)
            #                     yaw = cars_value + goal_value + self.yaw_rad
            #                 elif self.yaw_rad - bandwidth < -pi and yaw > 1.57:
            #                     goal_value = pi - yaw
            #                     yaw = -pi - goal_value

            #                 print(f'Step {1} -> ID: {next_id}, X: {target_x:.2f}, Y: {target_y:.2f}, Yaw: {yaw:.2f} rad')
                            
            #                 #self.state_target = ca.DM([target_x, target_y, 0.0])

            #                 self.state_target = ca.DM([target_x, target_y, yaw])
                
        else:
            pass
    # def image_callback(self, data):
    #     try:

    #         if  self.localisation_CB == True and self.IMU_cb == True:
    #             # start_time = time.time()
    #             # img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #             # gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #             # img_width = 640
    #             # img_height = 480
    #             # #img = cv2.resize(img, (img_width, img_height))
    #             # cv2.imshow('color', img)
    #             # cv2.waitKey(3)
    #             # if self.debug:
    #             #     self.debug_callback()
    #             # pass
    #             pass
    #     except AttributeError and CvBridgeError as e:
    #         print(e)
    #         rospy.logerr(e)

        
    #OLD# new path finding





    #####################   NEW PATH FINDING     #######################
    def process_and_publish_data(self,temp_source,temp_target):
        # GraphML dosyasını okuma
       

        #self.obs_dontuse=["360","314","321","344","315","270","367","64","169"]
        
        stx = time.time()# dosyadan endge okuma işelmleri zaman ölçümü için 
        #0.003 saniye sürüyo her bi okuma her çağırmada 2 defa çağırmak yerine 
        #normal olanı başta bir defa çağırıp  her engel tespiti yapıp
        # self.obs_dontuseya eleman eklediğimdede öbürünü çağırsam toplam süre 0.008 den 0.005 lere gerileyebilir
        #edge ve node datalarını fonksiyonun dışından almam gerekir



        tree = ET.parse(self.file_path_original)
        root = tree.getroot()
        #print("root : ",root)
        # Düğüm verilerini çıkarma
        self.nodes_data = self.extract_nodes_data(root)
        self.edges_data, self.edges_data_true_ilkverisyon = self.extract_edges_data(root)
        
        #print("nodes data ::::",self.edges_data)
        flagsolla=0
        self.flagsolla = 0
        for ciz in self.edges_data:#self.obs_dontuse listesindekilerden herhangi biri kesikli çizgi üstüne geliyo mu kontrolü 
            for kx in self.obs_dontuse:
                if ciz[1]==kx and ciz[2]==True:
                    flagsolla=1
                    self.flagsolla = 1
                    break#fazladan dolaşmasın bi kere bulması yeterli 
                #print("################ : ",ciz[2])

        etx = time.time()
        elapsed_timex = etx - stx
        print('Execution time:', elapsed_timex, 'seconds##################-----------------####################')
        



        if flagsolla==1:
            #sollama kısmı için özel graph dosyasını açıcam  ikinci bi edge çıkarma işlemi umarım fazla yavaşlatmaz 
            #şu anda 0.008 saniye
            tree2 = ET.parse(self.file_path)
            root2 = tree2.getroot()
            #print("root : ",root2)
            self.nodes_data = self.extract_nodes_data(root2)
            self.edges_data, self.edges_data_true_ilkverisyon = self.extract_edges_data(root2)

    

        #self.obs_dict = {node[0]: (node[1], node[2]) for node in self.nodes_data}#node dataya yeni düğümleri oluşturup burda self.nodes_data nın yerine koyucam
        #obs_dict bütün nodeların sıra ile  numarasını ve içidede x ve y var
        #print("//////////////////////////////////")
        #print("node datas::::::",self.obs_dict)#burda noktlalarındictonaryde id:  (x,y) formatında


        noded,edged=self.extract_graph()# (nodedict,edgedict) döndürür
        #print("edgedictionary ",edged)
        #self.obs_dontuse=["360","314","321","344","315","270","367","64","169","326"]
        path_short=self.dijkstra(temp_source,temp_target,noded,edged,self.obs_dontuse)#nodedictionary =noded
        self.path_original = path_short
        newnodedictionary,stlist=self.beizer(path_short,noded)#beizere targete giden path i veriyorum 
        self.obs_dict = newnodedictionary#yeni node sozlüğü
        self.edges_data_true =stlist#source target list 
        #print("-----------------------------------------")
        print(stlist)


        #print("-----------------------------------------")
        #nodedata ,sourcetarget
        #beizer çıktısı 1 : source targetnodes tipinde path  2:yeni eklenen noktalarıda içeren node_data o datayıda self.obs_dicte vericem 
        #409 da döngüye kapılıyo çıkamıyo 

        self.SourceTargetNodes = [(edge[0], edge[1]) for edge in self.edges_data_true if edge[2]]
        #self.SourceTargetNodes edge formatında benim pathi edge formatına çevirmemlazım 
        #burda sourceın içine hedef yolu vermek için edges data ture formatında en kısa yolu ustte vermemlazım
        #print("sorutceTrgetnodes:",self.edges_data_true)
        #print("**************************************************")
        #print("**************************************************")

        self.path = []
        for source_id, target_id in self.SourceTargetNodes:
            if target_id in self.obs_dict:
                coords = self.obs_dict[target_id]
                self.path.append((target_id, coords[0], coords[1]))
        #print("self.path::::",self.path)
        #print("**************************************************")
        #print("**************************************************")



        # Her bir nokta için bir sonraki nokta ile arasındaki açıyı hesaplama
        angles = []
        for i in range(len(self.path) - 1):
            dx = self.path[i + 1][1] - self.path[i][1]
            dy = self.path[i + 1][2] - self.path[i][2]
            angle = math.atan2(dy, dx)
            angles.append(angle)

        if angles:  # Check if angles list is not empty
            angles.append(angles[-1])

        self.pathGoalsYawDegree = [(*p, angle) for p, angle in zip(self.path, angles)]
        
        if not self.pathGoalsYawDegreecalled:
            self.pathGoalsYawDegreeCopy = self.pathGoalsYawDegree
            self.SourceTargetNodesCopy = self.SourceTargetNodes
            self.pathGoalsYawDegreecalled = True
        
        #print("raw edges data : ",self.edges_data)
        #print("self.SourceTargetNodes", self.SourceTargetNodes)
        #print("self.obs_dict", self.obs_dict)
        #print("**************************************************")
        #print("**************************************************")
        #print("**************************************************")
        #print("YawDegree", self.pathGoalsYawDegree)
        #print("self.path", self.path)
        #print(self.nodes_data
     
        

        # Data publishing simulation
        data_message = str(self.edges_data_true)
        # print("****************************************")
        # print("****************************************")
        # print("****************************************")
        # print("****************************************")

        # print("Graph data published:", data_message)

    def extract_nodes_data(self, root):
        nodes_data = []
        for node in root.findall(".//{http://graphml.graphdrawing.org/xmlns}node"):
            node_id = node.get('id')
            d0 = None
            d1 = None
            for data in node:
                if data.get('key') == 'd0':
                    d0 = float(data.text)  # 'x' koordinatı
                elif data.get('key') == 'd1':
                    d1 = 13.72-float(data.text)  # 'y' koordinatı
            if d0 is not None and d1 is not None:
                nodes_data.append((node_id, d0, d1))
        return nodes_data

    def extract_graph(self):
            #print(self.edges_data)
            # ('source','target',mesafe)
            inf = sys.maxsize
            graph_temp=self.edges_data

            nodedict=dict([])
            edgedict=dict([])#keyler source  valuelerde (target,mesafe)
            for node in self.nodes_data:
                nodedict[node[0]]={'mesafe':inf,'atalist':[],'solla':False,'x':node[1],'y':node[2]}#0  1 di   1  2 olarak düzelttim 
                #print(" nokta  :::::::::::::::::::::::::::::::::::node:",node,"x:",node[1]," y:",node[2])
                # key i id olan sırası ile mesafe ,parent(en kısay yoldaki köken düğüm), gezeldimi(0,1)  tutan dictionary
            for edge in graph_temp:
               # print(edge[0],edge[1])
                # sozlüğe yerleştirirken 1 key için var mi diye kontrol edicem varsa targeti valueya ekliycem yoksa oluşturup targeti valeuya ekliycem
                #edge[0] source  : edge
                temp=edge[0]
                edgemesafe =1#edge 0 ve edge 1 i verip ikisi arası mesafeyi vericem
                if temp in edgedict.keys():
                    #targeti valeuya append et mesafeleri de burda value içinde tutmamlazım 
                    #edgedict[edge[0]]=(edgedict[edge[0]],(edge[1]))
                    edgedict[edge[0]].append([edge[1],1])
                    
                else:
                    edgedict[edge[0]]=[[edge[1],1]]
                    #tuple dan list tipine cevirdim
                    
                    #sozlukte yoksa da ekliycek
                #print("edgedict :", edge[0],"  :",edgedict[edge[0]])
                edgetemp =edgedict[edge[0]]
                #for i in edgetemp:
                 #  print("test",i," leng :")
                #print(edgedict[edge[0]])
                # bağzı düğümler dosyadada tekrara düşmüş ikitane 88 var
            return (nodedict,edgedict)
    def minyol(self,edge_curent_source):
       # print(edge_curent_source)
        #print(edge_curent_source[0])
        #şimdilik her edge ağırlığı 1 diye önce 1. yi seçicek ama sonradan  mesafe hesapolı edgelerde bu kısmı lazım
        print(edge_curent_source[0][1])
        min=edge_curent_source[0][1]
        min_id=edge_curent_source[0][0]
        for edge in edge_curent_source:
            if edge[1]<min:
                min=edge[1]
                min_id=edge
        return min_id
    def hedef_mesafe(self,X1,Y1,X2,Y2):
        #kollara bakarken heapa kolları hedefe olan mesafelerine göre vermeyi deniyicem 
        
        #nodedicitonary den temp sourcela terget x y leini alıup mesafe çıkatıcam 
        x1=float(X1)
        x2=float(X2)
        y1=float(Y1)
        y2=float(Y2)
        #print("X1:",X1,"X2:",X2,"Y1:",Y1,"Y2:",Y2)
        #print("x1:",x1,"x2:",x2,"y1:",y1,"y2:",y2)
    
        mesafe_hedef=math.sqrt(((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2)))
        #print("mesafe fark var  ----------------------------------------:",mesafe_hedef)
        return mesafe_hedef
           
    def dijkstra(self,source ,target,nodedictt,edgedictt,yasaklistesi):
        #print("edgedict:",edgedictt)
        #print("____________________")
        nowaypoints=[]#çıkamzı döndürmek için tutuğum dizi 
        nowaypoints2=[]#yanı indekste tutmak için (targeti tutmak için)
        for ed in  list(edgedictt.items()):
            #edgedict tuple
            # engel olan noktaya giden düğümleri edgedictt ten silememişim 120 ye giderken 120leri siliyorum ama 120 ye giden 84 e giden leri silmiyorum 
            #bütün ölü sona giden yerleri bütün halde silmem mi gerekicek offfffffffffffff
            #print("ed:",ed[0]," ",ed[1])
            #print("edtype:",type(ed))
            for jkl in edgedictt[ed[0]]:#jkl liste objesi 
                #print("jnk ::. ",jkl)
                #print(type(jkl))

                for dont in yasaklistesi:
                    edg=str(ed[0])
                    if edg in edgedictt:
                        if  jkl[0]==dont:
                            #print(edg,"type:",type(edgedictt))
                            #print("karşılaştırma : edg:",edg ,"jkl[0]:",jkl[0],"self.obs_dontuse: ",self.obs_dontuse)
                            nowaypoints.append(edg)
                            nowaypoints2.append(jkl[0])
                            #silindi=edgedictt.pop(edg)
                            #jkl pop deniyicem
                            #print("------test-----")
                            
                            #print("start:",edgedictt[ed[0]])
                            temppp=edgedictt[ed[0]]
                            for test in temppp:
                                #print("t::",test)
                                if test==jkl:
                                    temppp.remove(test)
                                    #print("buldum")
                            #print("--------------------")
                            #print("temppp:",temppp)
                            #temp=temppp.pop(str(jkl))
                   
                            edgedictt[ed[0]]=temppp


                            #print("ed[1]:::",ed[1])
                            #print("silindi ",silindi)



        
        #print("edgedictt:",edgedictt)
        #print("-------------------------------")
        #print("-------------------------------")
        #print("-------------------------------")
        #print("nodedickt:",nodedictt)
        #print("-------------------------------")

        unvisited={n:float('inf') for n in edgedictt.keys()}
        #print("unvisited:",unvisited)
        unvisited[source]=0
        #revPath={}
        visited ={}
        a=0#teminal sonsuzda kalmasın diye
        while unvisited  :
            
            minNode=min(unvisited, key=unvisited.get)
            #print("minode:",minNode)
            visited[minNode]=unvisited[minNode]
            for j in edgedictt[minNode]:
                #j=  ['komsu ıd',mesafe]
                #print("komsu ",j,"type:",type(j))
                if j[0] in visited:
                    continue               
                cost=nodedictt[minNode]['mesafe']+1#her edge 1 olarak düşündüm
                #print("j[0]:",j[0],"minode:",minNode)
                #print("84 neredeeeeeeeeeeeeeeeeeeeeeeeee:",unvisited)
                if j[0] in unvisited:
                    if cost < unvisited[j[0]]:
                        # if e girmiyo
                        #print("daha kisa yol var "," bu ",j[0]," güncel mesafe :",cost)
                        nodedictt[j[0]]['mesafe']=cost#bu ilk yöntem ile yol çıkardığım 
                        nodedictt[j[0]]['atalist']=nodedictt[minNode]['atalist']+[minNode]#bu ilk yöntem ile yol çıkardığım 
                        unvisited[j[0]]=cost#2. yol 
                        #revPath[j[0]]=minNode#dijkstra da yolu çıkarmanın farklı bi yolu
                   
            unvisited.pop(minNode)
                #print("komşu:",neighbor)
        node=target
        #revPathString=node
        #print("pathxxxx:",nodedictt["371"]['atalist'])
        #print("---------------------------------------------------------------------------------------------")
        #print("---------------------------------------------------------------------------------------------")
        #print("path:",nodedictt[target]['atalist'])#ana çıktı yolu yazdırıyo yoruma aldım 
        #print("---------------------------------------------------------------------------------------------")
        #print("---------------------------------------------------------------------------------------------")[ERROR] [1712402619.277089]: 'mpc' object has no attribute 'expath'

        nodedictt[target]['atalist']=nodedictt[target]['atalist']+[target]
        yolll=nodedictt[target]['atalist']
        if source in yolll and target in yolll:
            print("yol var@@@@@@@@@@@@@@@@@@@")

            self.yolvar=True
            self.expath=nodedictt[target]['atalist']
            print("*************************")
            print("*************************")
            print(self.expath)
            print("*************************")
            print("*************************")
            return nodedictt[target]['atalist']
        else:
            print("yol yok###################")
            #burda bekleme calback çalışıcak  
            #yada bu kısmı dışarıda halledicem 
            #print(nodedictt)
            newtargetn=""
                                                            # shortstop=99999999999
                                                            # self.yolvar = False
                                                            # for tempstopx in yasaklistesi:
                                                            #     wayx=self.hedef_mesafe(nodedictt[tempstopx]['x'],nodedictt[tempstopx]['y'],nodedictt[target]['x'],nodedictt[target]['y'])

                                                            #     if wayx <shortstop:
                                                            #         shortstop=wayx
                                                            #         newtargetn=tempstopx

                                                            # #print("noway::",nowaypoints)
                                                            # #print("noway2:",nowaypoints2)
                                                            # indexno=nowaypoints2.index(newtargetn)
                                                            # #print("edges ::",edgedictt) 
                                                            # bagend=nowaypoints[indexno]
                                                            # nodedictt[bagend]['atalist']=nodedictt[bagend]['atalist']+[bagend]
                                                            # return nodedictt[bagend]['atalist']
            for tempstopx in yasaklistesi:
                if tempstopx in self.expath:
                    bagend=tempstopx
                    break
                    
            indexnoEX=self.expath.index(bagend)
            print("ex path:",self.expath)
            if len(self.expath)>2 :
                befbagend=self.expath[indexnoEX-2]#sourn burda olabillir tek nokta kalan path de indexi bir geriye alıp patlıyor olabilir
            else :
                befbagend=self.expath[indexnoEX]#bunu test et 
            print("before bagend",befbagend)

            #bagend in bi oncesinin ata listesi ni versem yeticek 
            #print("test ::",edgedictt)
            
            print("bagend :::",bagend)
            #print("bag path :",nodedictt[bagend]['atalist'])#burda bagend deki nokta ya eski source dan ulaşamadığımız için atalistesi boş ama  bi on cesini vermem lazim benim 
            nodedictt[befbagend]['atalist']=nodedictt[befbagend]['atalist']+[befbagend]
            return nodedictt[befbagend]['atalist']


        return nodedictt[target]['atalist']

    def plotbfmc(self,node_dat,path):
        #dosya adı için 
        obj = time.localtime(1627987508.6496193)
        gren=[0,255,0]
        orginx=0
        orginy=3863
        time_str = time.asctime(obj)
        name='/home/mekatronom/out/'+time_str
        #print("data:",node_dat)
        #print("path:",path)
        img=cv2.imread('/home/mekatronom/Downloads/Competition_track_graph.png')
        #cv2.circle(img, (0,3863), 20, gren, -1)
        for idxy in path:
            x=int(orginx+node_dat[idxy][0]*282)
            y=int(orginy-node_dat[idxy][1]*282)
            #print("x:",x,type(x))
            #print("y:",y,type(y))
            cv2.circle(img, (x,y), 15, gren, -1)
        #cv2.circle(img, (400,800), 15, gren, -1)

        #cv2.imshow('img', img)
        cv2.imwrite(name+".png", img) 
        cv2.waitKey(1)
    
    def beizer(self,path,node_d):
        #node_d nin y si hatalı geliyo ama araba düzgün sürüyo ?????????? 105 de y değiştirilmiş d1 = 13.72-float(data.text)  # 'y' koordinatı
        #hem beizeri hemde source targetteki gibi edge formatına getiricem 
        #print("node data _d:",node_d)#nose_dict node_d çok kalabalık daha saddeleştirip dictionary   id: (x,y),(x,y) formatına getiricem 
        #print("----------------------")
        #print("--------------------------------------------------------------------------------")
        #print("node data 470 :",node_d['470']['x']," ",node_d['470']['y'])
        #print("-----********************************--------------------------------------------------")
        new_node_data={}
        for idkey in node_d.keys():
            #print(idkey)
            new_node_data.update({idkey: (node_d[idkey]["x"],node_d[idkey]["y"])})
            #print("x:",x)
        #print("new_node_data:",new_node_data)
        new_point_counter=600#yeni düğmleri çakışma olmasın diye 600 den başlattım 
        new_path=path.copy()#path in kopyasını aldım değişiklikleri ona uyguluycam itarete ederken  list den eleman silip ekliyince hata veriyo

        

        #beizer1:verilen path de beizer uygulanması gereken yerleri bul açı kontrolü  
        print("beizer path :",path)
        

        #print("----------------------")
        path_length=len(path)
        for f in range(path_length-2):
            
            #açı hesaplıycam
            if node_d[path[f]]["x"]==node_d[path[f+1]]["x"]:
                #sıfıra bölme durumu
                angel_rad1=1.57#radyan cinsinden 90derece
            else:
                angel_rad1=math.atan((node_d[path[f]]["y"]-node_d[path[f+1]]["y"])/(node_d[path[f]]["x"]-node_d[path[f+1]]["x"]))#arctan 
                #math.atan radayan cinsinden  a numeric value between -PI/2 and PI/2 radians.
                #print("y deki fark :",abs(node_d[path[f]]["y"]-node_d[path[f+1]]["y"]),"x deki fark:",abs(node_d[path[f]]["x"]-node_d[path[f+1]]["x"]))
                #print("x1:",node_d[path[f]]["x"],"x2:",node_d[path[f+1]]["x"],"y1:",node_d[path[f]]["y"],"y2:",node_d[path[f+1]]["y"])
            angel_deg1=angel_rad1*57.3
            #print("açı bir :",angel_deg1)

            if node_d[path[f+1]]["x"]==node_d[path[f+2]]["x"]:
                #sıfıra bölme durumu
                angel_rad2=1.57#radyan cinsinden 90derece
            else:
                angel_rad2=math.atan((node_d[path[f+1]]["y"]-node_d[path[f+2]]["y"])/(node_d[path[f+1]]["x"]-node_d[path[f+2]]["x"]))#arctan 
                #math.atan radayan cinsinden  a numeric value between -PI/2 and PI/2 radians.
                #print("y deki fark :",abs(node_d[path[f]]["y"]-node_d[path[f+1]]["y"]),"x deki fark:",abs(node_d[path[f]]["x"]-node_d[path[f+1]]["x"]))
                #print("x1:",node_d[path[f]]["x"],"x2:",node_d[path[f+1]]["x"],"y1:",node_d[path[f]]["y"],"y2:",node_d[path[f+1]]["y"])
            angel_deg2=angel_rad2*57.3
            #print("açı iki :",angel_deg2)

            b_andgel=abs(angel_deg1-angel_deg2)
            #print("birinci açı :",angel_deg1," ikinci açı :",angel_deg2)
            
            if b_andgel>55 and b_andgel<110 :
                #print("--------------------------------------------------------------------------------------------------------------")
                #print("keskin  dönüş ::   başlangiç",path[f],"  köşe noktası",path[f+1],"  bitiş",path[f+2]," f+2:",f+2,"aradaki açı:",b_andgel)
                
                #lineer(lstart,lstop,t)#lineer fonksiyon da başlangıç ve bitiş arasında her çağırldığında t kadar ilerleyip ilerleyen nokta kordinatları döndüren fonk
                #print("koşe x:",node_d[path[f+1]]["x"],"y:",node_d[path[f+1]]["y"])
                numPts=2
                #iflerin orda keskin köşelere hardkodladım  düzeltme gerekebilir o 3 nokta için   sadece bu 3 nokta da hardkodlama var        # number of points in Bezier Curve
                if path[f]=='30'and path[f+1]=='35' and path[f+2]=='27':    
                    controlPts=[[node_d[path[f]]["x"],node_d[path[f]]["y"]],[4.60,4.30],[node_d[path[f+2]]["x"],node_d[path[f+2]]["y"]]]# control points

                elif path[f]=='58'and path[f+1]=='62' and path[f+2]=='55':    
                    controlPts=[[node_d[path[f]]["x"],node_d[path[f]]["y"]],[4.60,0.70],[node_d[path[f+2]]["x"],node_d[path[f+2]]["y"]]]# control points

                elif path[f]=='6'and path[f+1]=='11' and path[f+2]=='3':    
                    controlPts=[[node_d[path[f]]["x"],node_d[path[f]]["y"]],[4.60,7.10],[node_d[path[f+2]]["x"],node_d[path[f+2]]["y"]]]# control points
                else:
                    controlPts=[[node_d[path[f]]["x"],node_d[path[f]]["y"]],[node_d[path[f+1]]["x"],node_d[path[f+1]]["y"]],[node_d[path[f+2]]["x"],node_d[path[f+2]]["y"]]]# control points
                t=np.array([i*1/numPts for i in range(0,numPts+1)])

                B_x=(1-t)*((1-t)*controlPts[0][0]+t*controlPts[1][0])+t*((1-t)*controlPts[1][0]+t*controlPts[2][0])#yeni noktaların x i
                B_y=(1-t)*((1-t)*controlPts[0][1]+t*controlPts[1][1])+t*((1-t)*controlPts[1][1]+t*controlPts[2][1])#yeni y si
                temp_new_nodelist=[]
                for new_p in range(1,numPts):#+1 yapıp bütün noktaları yazdırdım  yeni düğüm eklerken 1 den numPts ye kadar gezip yeni düğüm ekliycem
                    #once yeni noktaları new_node_data ya ekliycem
                    new_point_counter=new_point_counter+1
                    new_point_str=str(new_point_counter)
                    #print("yeni nokta ",new_p,"x:",B_x[new_p],"y:",B_y[new_p])
                    new_node_data.update({new_point_str: (B_x[new_p],B_y[new_p])})
                    temp_new_nodelist.append(new_point_str)#nodu temp liste ekliyorum 
                # print("bx:",B_x.tolist())
                # print("by:",B_y.tolist())
                # print("stop x:",node_d[path[f+2]]["x"],"y:",node_d[path[f+2]]["y"])
                #print("node data:",node_d)
                #f+1 deki noktayı path den çıkartıp araya yeni noktaları yerleştirmemlazım 
                #burda pathde  keskin donuşte f+1 de olan kose noktasını path den çıkarıp yeni noktaları edge olarak araya ekliycem
                #print("path::::",path)
                #print("koşe noktası ::::",path[f+1],"index:",path.index(path[f+1]))
                new_path.remove(path[f+1])#path  de koşe noktasını new path densilicem
                
                #şimdi yeni noktaları path e sırasıyla insert edicem 
                temp_index=path.index(path[f+1])
                #print("bunun peşine ekle :",path[f+1])
                for insrt in temp_new_nodelist:
                    new_path.insert(temp_index,insrt)
                    temp_index=temp_index+1
                    #indexde hata var sabah bak  93 deden sonrası path de çakışma olmuş 
                #print("new:path :",new_path)
                #edge
            ####3 self.plotbfmc(new_node_data,new_path)
        #print("new path :",new_path)#en son path
        source_target=[]
        pathlen=len(new_path)
        #print("len :",pathlen)
        for n_edge in range(pathlen-1):
            source_target.append((new_path[n_edge],new_path[n_edge+1],True))
        #print("formattli:",source_target)
        #print("--------------------------------------------")
        #print("node data:",new_node_data)
        print("leaving here1")

        self.plotbfmc(new_node_data,new_path)


        print("leaving here2")
        return new_node_data,source_target
            
        #edge haline getirip return edicem 

        #print("new_node_data new new :",new_node_data)

        #beizer2:3 noktayı bulunca beizer ile ara noktalar oluşturup path de araya inssert edicem 

        #formatlama:düğüm düğüm olan path i edge edge hale gertip return edicem
 
    def extract_edges_data(self, root):
        edges_data = []
        edges_data_true = []
        for edge in root.findall(".//{http://graphml.graphdrawing.org/xmlns}edge"):
            source_id = edge.get('source')
            target_id = edge.get('target')
            data_d2 = edge.find(".//{http://graphml.graphdrawing.org/xmlns}data[@key='d2']")
            d2_value = data_d2 is not None and data_d2.text == 'True'
            # None null safety gibi bişey mi sor
            source_id_int = int(source_id)
            target_id_int = int(target_id)
            ranges = [(469, 479), (409, 425), (386, 398), (357, 365), (259, 281)]
            #ranges = [(469, 479)]
            if any(start <= source_id_int <= end for start, end in ranges) and any(start <= target_id_int <= end for start, end in ranges):
                d2_value = True
                #d2 dokumantasyonda solanabilir yerleri belirtmek için kullanılmış ama burda devam belirteci olarak kullanılıyo bunu sor 
                edges_data_true.append((source_id, target_id, d2_value))
            edges_data.append((source_id, target_id, d2_value))
            #edeges_data normal dutun edgelerin verisi  edges_data_true  ranges dahilindeki true olan edgeler
        return edges_data, edges_data_true        

############################################newpathfinding

    def obstacleDetector_callback(self, data):
        # print("obstacleDetector_callback")
        self.center_x = []
        self.center_y = []
        if not data.circles:  # Eğer circles dizisi boşsa
            # Direkt olarak 0.0, 0.0 ekleyin
            # self.center_x.append(0.0)
            # self.center_y.append(0.0)
            pass
        else:
            # Eğer circles boş değilse, her bir dairenin merkez koordinatlarını al ve listeye ekle
            for circle in data.circles:
                self.center_x.append(circle.center.x)
                self.center_y.append(circle.center.y)

    #Debug Start
    def debug_callback(self):
        current_time = rospy.Time.now().to_sec()
        PassedTime = current_time - self.prev_time
        # rospy.loginfo("Elapsed time: %.4f seconds", PassedTime)
        self.prev_time = current_time
        # print("self.nodes_data",self.nodes_data)
        # print("self.edges_data",self.edges_data)
        # print("self.edges_data_true",self.edges_data_true)
        # print("self.SourceTargetNodes",self.SourceTargetNodes)
        # print("self.obs_dict",self.obs_dict)
        # print("self.pathGoalsYawDegree",self.pathGoalsYawDegree)
        # print("self.path",self.path )

    def pose_callback(self, data):
        
        start_time = time.time()
        # print("Position: ", data.pose.position)
        # print("Orientation: ", data.pose.orientation)
        self.position_x = data.pose.position.x
        self.position_y = data.pose.position.y
        # print("self.position_x",self.position_x)
        # print("self.position_y",self.position_y)
        
        quaternion = (data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z, data.pose.orientation.w)
        (self.roll, self.pitch, yaw_rad) = euler_from_quaternion(quaternion)
        self.yaw_rad = yaw_rad
        end_time = time.time()
        duration = end_time - start_time  # İşlemin sürdüğü süreyi hesapla
        # print(f"CallbackPosition süresi: {duration} saniye.")
        self.localisation_CB = True
        self.IMU_cb = True
        #self.mpcTimerCallback(event=None)
        
    def carControlPublisher(self):  

        pass        
        # # print("carcontroltest")
        # ser = serial.Serial(port='/dev/ttyACM0',baudrate=19200)
        # #switchOn=input('Hiz Degerini Giriniz'
        # # print("carcontroltest2")
        # switchOn = "#1:{};;\r\n".format(self.steerLateral*100)

        # ser.write(bytes(switchOn,'utf-8'))
        # # value=ser.readline()
        # # valueInString=str(value,'UTF-8')
        # # print(valueInString)

        # if self.steerAngle > 25:
        #     self.steerAngle = 25
        # if self.steerAngle < -25:
        #     self.steerAngle = -25
        
        # #switchOn=input('Hiz Degerini Giriniz')
        # switchOn = "#2:{};;\r\n".format(-1*(self.steerAngle))
        # ser.write(bytes(switchOn,'utf-8'))
    
        # rospy.loginfo("used steeringAngle %s", self.steerAngle)
        # rospy.loginfo("used steeringLateral %s", self.steerLateral*100)
        
        # value=ser.readline()
        # valueInString=str(value,'UTF-8')
        #print(valueInString)

        # print("self.steerAngle",self.steerAngle)
        # print("self.steerLateral",self.steerLateral*100)

        # carData = {}
        # carData = {
        #     'action': '2',
        #     'steerAngle': self.steering_angle 
        #     # 'steerAngle': 30.0
        # }
        # # print("data",carData)
        # # print("math.degrees(float(self.DM2Arr(self.u[0, 1])))  ",math.degrees(float(self.DM2Arr(self.u[0, 1])))  )
        # self.carData = json.dumps(carData)
        # self.carControl.publish(self.carData)
        # #rospy.sleep(0.01)
        # car2Data = {}
        # # İkinci mesaj için veriler
        # car2Data = {
        #     'action': '1',
        #     'speed': self.steerLateral  
        #     #'speed': 0.0
        # }   
        # #print("data",car2Data)
        # self.car2Data = json.dumps(car2Data)
        # self.carControl.publish(self.car2Data)



def main():
    rospy.init_node('MPCtracking', anonymous=False)
    cameratracker = mpc(camera_topic_name = "/zed2i/zed_node/rgb/image_rect_color",
                                 pose_topic_name = "/zed2i/zed_node/pose")

    rate = rospy.Rate(200) # 10Hz
    try:
        while not rospy.is_shutdown():
            # Sürekli çalışacak kodlar burada
            rate.sleep()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        

if __name__ == '__main__':
    main()
