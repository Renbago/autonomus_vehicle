#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import math
from math import atan2
import json
import xml.etree.ElementTree as ET
from casadi import sin, cos, pi
import casadi as ca
import sys
import time
import random
from std_msgs.msg import String, Bool, Header, Byte
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from utils.msg import localisation, IMU
from nav_msgs.msg import Odometry
import numpy as np

from example.msg import MekatronomYolo
import os
import tf
import tf

from obstacle_detector.msg import Obstacles
class MPC():

    def __init__(self):


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
        self.crosswalk_update_time = rospy.Time.now()
        self.last_time_went_node = rospy.Time.now()
        self.last_path_index  = 0
        self.last_target_update_time = None 
        self.current_target = None

        self.yolo_data = MekatronomYolo()

        self.state = "keep lane"

        
        self.intercept_targets = ["243","261","252","54","51","56","317","367","397","404","34","84","163","80","93","176",
                                        "91","14","18","2","6","143","102","32","16","222","219","30","38","100","26","42",
                                        "109","113","121","75","185","71","40","205","226","283","198"]

        self.dont_check_obstacles_here = ["228","229","230","231","232","233","234","235","236","237","238","239","240"]
        # self.parking_secture_nodes_id = ["450","451","452","453","454","455","456","457","458","459","460","461","462","463","464","465","466","467"]

        # self.intercept_targets = []
        self.distance_flag = False
        self.yolo_intercept_flag = False
        self.park_scenerio = False
        self.parking_flag = False
        self.crosswalk_scenerio = False
        self.crosswalk_flag = False
        self.motorway_flag = False
        self.motorway_scenerio = False

        self.prev_time = rospy.Time.now().to_sec()      
        self.bridge = CvBridge()

        #sükrü pathfinding icin
        self.source_node="263" #472   
        self.target_node="225" #197  #901
        self.current_id=self.source_node
        self.current_id_original=self.source_node
        #newPathFinding
        self.graphml_file_path = os.path.expanduser(
            rospy.get_param(
                '~graphml_file_path', 
                '~/autonomus_ws/src/autonomus_vehicle/src/example/graphml/fixed2.graphml'
            )
        )

        self.file_path_original = os.path.expanduser(
            '~/autonomus_ws/src/autonomus_vehicle/src/example/graphml/gercek2.graphml'
        )
        self.callnumber=0
        self.new_point_ctr=600
        self.obs_dontuse = ["273"]
        self.yolvar=False
        self.parking_nodes_id = ["900","901","902","903","904","910","911","912","913","914"]
        self.traffic_light_nodes_id = ["32","121","135"]
        self.parking_spot_is_full = []
        self.parkings_are_available =[]
        self.expath = []

        self.flagsolla = 0
        self.obstaclesayac = 0
        self.center_x = []
        self.center_y = []
        self.obstacles_array = []
        self.past_obs_dontuse = []
        self.traffic_light_master = []
        self.traffic_light_slave = []
        self.traffic_light_start = []
        self.pathGoalsYawDegreecalled = False


        self.process_and_publish_data(self.source_node,self.target_node)#burda çağırılmış ilk defa her çıkmaz sokakta yine çağırıcam 

        ##############################3sükrü    
        #ForPathFinding
        self.file_path = rospy.get_param('~file_path', self.graphml_file_path)
        self.data_pub = rospy.Publisher('~graph_data', String, queue_size=10)
        
        # self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))

        #Subscribers
        self.image_sub = rospy.Subscriber('/automobile/image_raw', Image, self.image_callback)
        self.localisation_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.localisation_callback)
        self.imu_sub = rospy.Subscriber('/automobile/IMU', Imu, self.imu_callback)
        self.obstacleDetector_sub = rospy.Subscriber('/obstacles', Obstacles, self.obstacleDetector_callback)
        self.yolo_intercept_sub = rospy.Subscriber('/automobile/yolo', MekatronomYolo, self.yolo_intercept_callback)
        self.traffic_light_master_sub = rospy.Subscriber('/automobile/trafficlight/master', Byte, self.traffic_light_master_callback)
        self.traffic_light_slave_sub = rospy.Subscriber('/automobile/trafficlight/slave', Byte, self.traffic_light_slave_callback)
        self.traffic_light_start_sub = rospy.Subscriber('/automobile/trafficlight/start', Byte, self.traffic_light_start_callback)
        #Publishers
        
        self.carControl = rospy.Publisher('/automobile/command', String, queue_size=2, latch=True)

        # self.imu_pub = rospy.Publisher('/imu0', Imu, queue_size=10) #the sensor_msgs imu
        # self.localisation_pub = rospy.Publisher('/localisation0', PoseWithCovarianceStamped, queue_size=10) #the sensor_msgs localisation


        #timerCB
        self.behaviourTimer = rospy.Timer(rospy.Duration(0.5), self.behaviourTimerCallback)
        self.mpcTimer = rospy.Timer(rospy.Duration(0.05), self.mpcTimerCallback)

        

    def DM2Arr(self,dm):
        return np.array(dm.full())

    def shift_timestep(self):
        
        self.state_init = ca.DM([self.position_x, self.position_y,  self.yaw_rad])
        f_value = self.f(self.state_init,self.u[0, :].T)
        self.next_state = ca.DM.full(self.state_init + (self.step_horizon * f_value))
        self.t0 = self.t0 + self.step_horizon
        self.u0 = ca.vertcat(self.u[1:, :], self.u[-1, :])

    def mpc_start_settings(self):
        # print("mpcsettings")

        if self.park_scenerio:
            rospy.loginfo("SETTINGS TO PARK PARAMS FOR MPC")

            Q_x = rospy.get_param('~Q_x', 1)
            Q_y = rospy.get_param('~Q_y', 4)
            Q_theta = rospy.get_param('~Q_theta', 0.05)
            R1 = rospy.get_param('~R1', 0.01)
            R2 = rospy.get_param('~R2', 0.02)
            step_horizon = rospy.get_param('~step_horizon', 0.1)
            N = rospy.get_param('~N', 6)
            rob_diam = rospy.get_param('~rob_diam', 0.354)
            wheel_radius = rospy.get_param('~wheel_radius', 1)
            L_value = rospy.get_param('~Lx', 0.3)
            Ly = rospy.get_param('~Ly', 0.04)
            v_max = rospy.get_param('~v_max', 0.2)
            v_min = rospy.get_param('~v_min', -0.25) 
            omega_max = rospy.get_param('~omega_max', pi/7.5) #degree
            omega_min = rospy.get_param('~omega_min', -pi/7.5) #degree
            self.iteration = 0
            self.steeringRad = 0.0     

        elif self.crosswalk_scenerio:
            rospy.loginfo("SETTINGS TO CROSSWALK PARAMS FOR MPC")

            Q_x = rospy.get_param('~Q_x', 1)
            Q_y = rospy.get_param('~Q_y', 1)
            Q_theta = rospy.get_param('~Q_theta', 1.0)
            R1 = rospy.get_param('~R1', 1.0)
            R2 = rospy.get_param('~R2', 1.8)
            step_horizon = rospy.get_param('~step_horizon', 0.1)
            N = rospy.get_param('~N', 12)
            rob_diam = rospy.get_param('~rob_diam', 0.354)
            wheel_radius = rospy.get_param('~wheel_radius', 1)
            L_value = rospy.get_param('~Lx', 0.3)
            Ly = rospy.get_param('~Ly', 0.04)
            v_max = rospy.get_param('~v_max', 0.13)
            v_min = rospy.get_param('~v_min', -0.2) 
            omega_max = rospy.get_param('~omega_max', pi/7.5) #degree
            omega_min = rospy.get_param('~omega_min', -pi/7.5) #degree
            self.iteration = 0
            self.steeringRad = 0.0  

        elif self.motorway_scenerio:
            rospy.loginfo("INITIAL KEEP LANE PARAMS FOR MPC")

            Q_x = rospy.get_param('~Q_x', 1)
            Q_y = rospy.get_param('~Q_y', 1)
            Q_theta = rospy.get_param('~Q_theta', 2)
            R1 = rospy.get_param('~R1', 2.0)
            R2 = rospy.get_param('~R2', 2.8)
            step_horizon = rospy.get_param('~step_horizon', 0.2)
            N = rospy.get_param('~N', 4)
            rob_diam = rospy.get_param('~rob_diam', 0.354)
            wheel_radius = rospy.get_param('~wheel_radius', 1)
            L_value = rospy.get_param('~Lx', 0.5)
            Ly = rospy.get_param('~Ly', 0.04)
            v_max = rospy.get_param('~v_max', 0.5)
            v_min = rospy.get_param('~v_min', -0.1)
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
            R2 = rospy.get_param('~R2', 1.4)
            step_horizon = rospy.get_param('~step_horizon', 0.2)
            N = rospy.get_param('~N', 12)
            rob_diam = rospy.get_param('~rob_diam', 0.354)
            wheel_radius = rospy.get_param('~wheel_radius', 1)
            L_value = rospy.get_param('~Lx', 0.4)
            Ly = rospy.get_param('~Ly', 0.04)
            v_max = rospy.get_param('~v_max', 0.33)
            v_min = rospy.get_param('~v_min', -0.1)
            omega_max = rospy.get_param('~omega_max', pi/7.5) #degree
            omega_min = rospy.get_param('~omega_min', -pi/7.5) #degree
            self.iteration = 0
            self.steeringRad = 0.0
        
        rotationMatrix = np.array([[cos(self.yaw_rad),-sin(self.yaw_rad)],
                [sin(self.yaw_rad),cos(self.yaw_rad)]])
        matrix = np.array([1,4])
        self.Q_x,self.Q_y = np.dot(rotationMatrix,matrix)

        nodes_x = np.array([node[1] for node in self.pathGoalsYawDegreeCopy])
        nodes_y = np.array([node[2] for node in self.pathGoalsYawDegreeCopy])
        
        distances = np.sqrt((nodes_x - self.position_x)**2 + (nodes_y - self.position_y)**2)
        min_distance, index = min((val, idx) for (idx, val) in enumerate(distances))
        closest_node_id = self.pathGoalsYawDegreeCopy[index][0]
        self.last_path_index = index        

        if self.park_scenerio:
            current_id = self.current_id
            print("parking scenerio")

        else:
            current_id = closest_node_id

        matching_pairs = [pair for pair in self.SourceTargetNodesCopy if pair[0] == current_id]
        next_id = matching_pairs[0][1]
        matching_entry = next((entry for entry in self.pathGoalsYawDegreeCopy if entry[0] == next_id), None)

        target_x, target_y = matching_entry[1], matching_entry[2]
        target_y = target_y

        dx = target_x - self.position_x
        dy = target_y - self.position_y
        yaw = atan2(dy, dx)

        self.goal_id = matching_entry
        state_init = ca.DM([self.position_x, self.position_y, self.yaw_rad])        # initial state
        state_target = ca.DM([target_x, target_y, yaw])

        print("state_init",state_init)
        print("state_target",state_target)

        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(
            x,
            y,
            theta
        )
        n_states = states.numel()

        v = ca.SX.sym('v')
        omega = ca.SX.sym('omega')
        controls = ca.vertcat(
            v,
            omega
        )
        n_controls = controls.numel()

        # matrix containing all states over all time steps +1 (each column is a state vector)
        X = ca.SX.sym('X', n_states, (N + 1))

        # matrix containing all control actions over all time steps (each column is an action vector)
        U = ca.SX.sym('U', n_controls, N)

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


        for k in range(N):
            st = X[:, k]
            con = U[:, k]

            cost_fn = cost_fn + ca.mtimes(ca.mtimes((st - P[3:6]).T, Q), (st - P[3:6])) + ca.mtimes(ca.mtimes(con.T, R), con)

            st_next = X[:, k+1]
            #print("st_next",st_next)
            k1 = f(st, con)
            k2 = f(st + (step_horizon/2)*k1, con)
            k3 = f(st + (step_horizon/2)*k2, con)
            k4 = f(st + step_horizon * k3, con)


            st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            #print("st_next_RK4",st_next_RK4)

            g = ca.vertcat(g, st_next - st_next_RK4)

        obs_x = -3.0
        obs_y = -3.0
        obs_diam = 0.3
        #print(g.shape)

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



        args = {
            'lbg': lbg,  # constraints lower bound
            'ubg': ubg,  # constraints upper bound
            'lbx': lbx,
            'ubx': ubx
        }

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
                


                self.args['p'] = ca.vertcat(
                    self.state_init,    # current state
                    self.state_target   # target state
                )

                self.args['x0'] = ca.vertcat(
                    ca.reshape(self.X0.T, self.n_states*(self.N+1), 1),
                    ca.reshape(self.u0.T, self.n_controls*self.N, 1)
                )

                sol = self.solver(
                    x0=self.args['x0'],
                    lbx=self.args['lbx'],
                    ubx=self.args['ubx'],
                    lbg=self.args['lbg'],
                    ubg=self.args['ubg'],
                    p=self.args['p']
                )



                self.u = ca.reshape((sol['x'][self.n_states * (self.N + 1):]).T, self.n_controls, self.N).T

                print("\n\n\nstate_target",self.state_target)


                self.cat_states = np.dstack((
                    self.cat_states,
                    self.DM2Arr(self.X0)
                ))

                self.cat_controls = np.vstack((
                    self.cat_controls,
                    self.DM2Arr(self.u[0, :])
                ))

                self.shift_timestep()

                self.X0 = ca.reshape((sol['x'][: self.n_states * (self.N+1)]).T, self.n_states, self.N+1).T
                # print("self.X0 ",self.X0)

                self.X0 = ca.vertcat(
                    self.X0[1:, :],
                    ca.reshape(self.X0[-1, :], 1, -1)
                )


                if self.state == "keep lane":
                    self.steerAngle = math.degrees(float(self.DM2Arr(self.u[0, 1])))
                    self.steerLateral = float(self.DM2Arr(self.u[0, 0]))
                    self.carControlPublisher()


                state_target_slice = self.state_target[:2]
                state_init_slice = self.state_init[:2]

                # Calculate the norm
                distance = ca.norm_2(ca.DM(state_init_slice) - ca.DM(state_target_slice))
                

                if distance < 0.2:
                    #print(self.nodedatabest)
                    print("self.current_id",self.current_id)
                    #print(self.path_original)

                    now=self.current_id_original
                    if self.nodedatabest[now]["solla"]:
                        self.flagsolla=1
                    else:
                        self.flagsolla=0
                    print("sollaaaaaa",self.flagsolla)

                    #print("test3")
                    print("insideofdistance")

                    if self.pathGoalsYawDegree: 
                        nodes_x = np.array([node[1] for node in self.pathGoalsYawDegree])
                        nodes_y = np.array([node[2] for node in self.pathGoalsYawDegree])
                        distances = np.sqrt((nodes_x - self.position_x)**2 + (nodes_y - self.position_y)**2)
                        min_distance, index = min((val, idx) for (idx, val) in enumerate(distances))
                        closest_node_id = self.pathGoalsYawDegree[index][0]

                        self.last_path_index = index        
                        current_id = closest_node_id
                        self.current_id = current_id

                        nodes_x_original = np.array([node[1] for node in self.pathGoalsYawDegreeOriginal])
                        nodes_y_original = np.array([node[2] for node in self.pathGoalsYawDegreeOriginal])
                        distances_original = np.sqrt((nodes_x_original - self.position_x)**2 + (nodes_y_original - self.position_y)**2)
                        min_distance_original, index_original = min((val, idx) for (idx, val) in enumerate(distances_original))
                        closest_node_id_original = self.pathGoalsYawDegreeOriginal[index_original][0]
                        
                        self.current_id_original = closest_node_id_original
                        print("\n\n\n\ncurrent_id_original",self.current_id_original)
                        print("\n\n\n\n\ncurrent_id",self.current_id)

                    else:
                        nodes_x = np.array([node[1] for node in self.pathGoalsYawDegreeCopy])
                        nodes_y = np.array([node[2] for node in self.pathGoalsYawDegreeCopy])
                        distances = np.sqrt((nodes_x - self.position_x)**2 + (nodes_y - self.position_y)**2)
                        min_distance, index = min((val, idx) for (idx, val) in enumerate(distances))
                        closest_node_id = self.pathGoalsYawDegreeCopy[index][0]

                        self.last_path_index = index        
                        current_id = closest_node_id
                        self.current_id = current_id


                    sign_looking_band = []

                    nodes_x_for_obstacles = np.array([node[1] for node in self.pathGoalsYawDegreeCopy])
                    nodes_y_for_obstacles = np.array([node[2] for node in self.pathGoalsYawDegreeCopy])

                    distances = np.sqrt((nodes_x_for_obstacles - self.position_x)**2 + (nodes_y_for_obstacles - self.position_y)**2)
                    min_distance, index = min((val, idx) for (idx, val) in enumerate(distances))
                    current_id_for_obstacles = self.pathGoalsYawDegreeCopy[index][0]
                    print(type(current_id_for_obstacles))
                    
                    for i in range(1, 5):
                        matching_pairs_signs = [pair for pair in self.SourceTargetNodesCopy if pair[0] == current_id_for_obstacles]
                        matching_entry = next((entry for entry in self.pathGoalsYawDegreeCopy if entry[0] == current_id_for_obstacles), None)

                        print("testing")    
                        print("matching_entry",matching_entry)
                        print("matcihing_pairs_signs",matching_pairs_signs)
                        if matching_pairs_signs:
                            next_id_signs = matching_pairs_signs[0][1]

                            matching_entry_second = next((entry for entry in self.pathGoalsYawDegreeCopy if entry[0] == next_id_signs), None)
                            third_pairs_signs = [pair for pair in self.SourceTargetNodesCopy if pair[0] == next_id_signs]
                            if third_pairs_signs:
                                print("third_pair_sings", third_pairs_signs)
                                matching_entry_third = next((entry for entry in self.pathGoalsYawDegreeCopy if entry[0] == third_pairs_signs[0][1]), None)
                            print("matching_entry_second",matching_entry_second)
                            print("next_id",next_id_signs)
                            
                            sign_looking_band.append(current_id_for_obstacles)  
                            current_id_for_obstacles = next_id_signs 
                            
                            TargetPosition = matching_entry[1:3]
                            # TargetPositionNext = matching_entry_second[1:3]

                            x_diff = abs(matching_entry[1] - matching_entry_second[1])
                            y_diff = abs(matching_entry[2] - matching_entry_second[2])

                            x_threshold = 0.10
                            y_threshold  = 0.10

                           
                            for center_x, center_y in zip(self.center_x, self.center_y):
                                ObstaclePosition = [center_x, center_y]

                                # if "parking" in self.yolo_data.object:
                                for node_id, (x, y) in self.obstacle_node_positions.items():
                                    distance = np.sqrt((center_x - x) ** 2 + (center_y - y) ** 2)
                                    if distance <= 0.4 and node_id not in self.parking_spot_is_full:
                                        self.parking_spot_is_full.append(node_id)
                                        self.parking_nodes_id.remove(node_id)  # Engelli node ID'yi park alanlarından çıkar

                                if self.target_node in self.parking_spot_is_full:
                                    self.target_node = self.parking_nodes_id[0]
                                    self.process_and_publish_data(self.current_id_original,self.target_node)


                                # TODO: Baha burada kaldın
                                rospy.loginfo("parking_spot_is_full: %s", self.parking_spot_is_full)
                                rospy.loginfo("parking_nodes_id: %s", self.parking_nodes_id)


                                # Calculate the norm
                                obstaclediff = np.linalg.norm(np.array(ObstaclePosition) - np.array(TargetPosition))

                                is_within_x_threshold = min(matching_entry[1], matching_entry_second[1]) - x_threshold <= center_x <= max(matching_entry[1], matching_entry_second[1]) + x_threshold
                                is_within_y_threshold = min(matching_entry[2], matching_entry_second[2]) - y_threshold <= center_y <= max(matching_entry[2], matching_entry_second[2]) + y_threshold
                                print("obstaclediff",obstaclediff)
                                print("center_x",center_x)
                                print("center_y",center_y)
                                print("is_within_x_threshold",is_within_x_threshold)
                                print("is_within_y_threshold",is_within_y_threshold)

                                if obstaclediff < 0.15 or (is_within_x_threshold and is_within_y_threshold) and matching_entry[0] not in self.dont_check_obstacles_here:  
                                # if obstaclediff < 0.15:

                                    print("obstaclediffINSIDE",obstaclediff)
                                    print("current_idObstacle",self.current_id)
                                    print("matching_entryObstacle",matching_entry[0])
                                    
                                    self.past_obs_dontuse = self.obs_dontuse.copy()

                                    # self.obstacles_array = []  ## bu 2 kod sornadan kaldırılacaktır.
                                    # self.obs_dontuse=["489","127","96"]#kullanma listesi

                                    if matching_entry[0] not in self.obstacles_array:
                                        self.obstacles_array.append(matching_entry[0])
                                        if third_pairs_signs:
                                            if matching_entry_third[0] not in self.obstacles_array:
                                                self.obstacles_array.append(matching_entry_third[0])
                                    if matching_entry_second[0] not in self.obstacles_array:
                                        self.obstacles_array.append(matching_entry_second[0])
                                    
                                    for obstacle_id in self.obstacles_array:
                                        print("self.obs_dontuseinside",self.obs_dontuse)
                                        print("self.obstacles_arrayinside",self.obstacles_array)
                                        obstacle_id_str = str(obstacle_id)
                                        if obstacle_id_str not in self.obs_dontuse:
                                            self.obs_dontuse.insert(0, obstacle_id_str) 
                                    if self.obs_dontuse != self.past_obs_dontuse:
                                        self.process_and_publish_data(self.current_id_original,self.target_node)#curent den global targete göndericem
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

                                        targetposition_obstacle_first = matching_entry_obstacle_first[1:3]
                                        targetposition_obstacle_second = matching_entry_obstacle_second[1:3]
                                        # Calculate the norm
                                        x_threshold = 0.15
                                        y_threshold = 0.15

                                        is_within_x_threshold = min(matching_entry_obstacle_first[1], matching_entry_obstacle_second[1]) - x_threshold <= center_x <= max(matching_entry_obstacle_first[1], matching_entry_obstacle_second[1]) + x_threshold
                                        is_within_y_threshold = min(matching_entry_obstacle_first[2], matching_entry_obstacle_second[2]) - y_threshold <= center_y <= max(matching_entry_obstacle_first[2], matching_entry_obstacle_second[2]) + y_threshold


                                        if np.linalg.norm(np.array(targetposition_obstacle_first) - np.array(ObstaclePosition)) < 0.16 or np.linalg.norm(np.array(targetposition_obstacle_second) - np.array(ObstaclePosition)) < 0.16 or (is_within_x_threshold and is_within_y_threshold):
                                            self.state = "waiting the obstacle move on"
                                            self.last_update_time_obstacles_checking = rospy.Time.now()
                                            self.obstaclesayac = 0

                    print("self.statefirst",self.state)
                    print("self.flagsolla",self.flagsolla)
                    self.obstaclesayac = self.obstaclesayac + 1

                    if self.state == "keep lane" and self.obstacles_array and self.flagsolla == 0 and self.obstaclesayac == 50:  
                            
                        print("hi im here")
                        print("self.obstacles_array",self.obstacles_array)
                        for value in self.obstacles_array:
                            if value in self.obs_dontuse:
                                self.obs_dontuse.remove(value)
                        self.obstacles_array = []

                        self.process_and_publish_data(self.current_id_original,self.target_node)#curent den global targete göndericem

                    # if any obstacle in the matching_pairs_signs create new path so call djikstra

                    if any(sign_id in self.traffic_light_nodes_id for sign_id in sign_looking_band): 
                        
                        rospy.loginfo("self.traffic_lights_nodes_idchecking")
                        
                        if self.traffic_light_master[0] == 0 and self.current_id =="489":
                            rospy.loginfo("self.traffic_light_master[0] == 0")
                            self.steerAngle = math.degrees(0.0) 
                            self.steerLateral = float(0.0)
                            self.state = "waiting on traffic light"
                            self.last_update_time = rospy.Time.now()   
                            # rospy.loginfo("self.state: {}".format(self.state))
                            self.carControlPublisher()

                        if self.traffic_light_slave[0] == 0  and self.current_id == "135":
                            rospy.loginfo("self.traffic_light_slave[0] == 0")
                            self.steerAngle = math.degrees(0.0) 
                            self.steerLateral = float(0.0)
                            self.state = "waiting on traffic light"
                            self.last_update_time = rospy.Time.now()   

                            # rospy.loginfo("self.state: {}".format(self.state))
                            self.carControlPublisher()

                        if self.traffic_light_start[0] == 0 and self.current_id =="121":
                            rospy.loginfo("self.traffic_light_start[0] == 0")  
                            self.steerAngle = math.degrees(0.0) 
                            self.steerLateral = float(0.0)
                            self.state = "waiting on traffic light"
                            self.last_update_time = rospy.Time.now()
                            # rospy.loginfo("self.state: {}".format(self.state))
                            self.carControlPublisher()

                    elif any(sign_id in self.intercept_targets for sign_id in sign_looking_band) and "stop" in self.yolo_data.object:

                        if current_id in self.intercept_targets:

                            self.steerAngle = math.degrees(0.0) 
                            self.steerLateral = float(0.0)
                            self.state = "waiting on intercept"
                            # rospy.loginfo("self.state: {}".format(self.state))
                            self.carControlPublisher()     
                        
                        else:
                            self.last_update_time = rospy.Time.now()

                    elif any(sign_id in self.intercept_targets for sign_id in sign_looking_band) and "crosswalk" in self.yolo_data.object:
                        
                        if self.crosswalk_flag == False:
                            self.crosswalk_scenerio = True
                            # self.state = "no obstacle on crosswalk"
                            self.mpc_start_settings()     
                            self.crosswalk_flag = True   
                            self.crosswalk_update_time = rospy.Time.now()      

                        distance_meets_criteria = False

                        rospy.loginfo("self.state: {}".format(self.state))

                        self.crosswalk_update_time = rospy.Time.now()
                        self.last_update_time = rospy.Time.now()

                    else:
                        self.state = "keep lane"
                        rospy.loginfo("self.state: {}".format(self.state))

                    print("yolo_data.object",self.yolo_data.object)
                    if any(sign_id in self.intercept_targets for sign_id in sign_looking_band) and "parking" in self.yolo_data.object and self.parking_flag == False:
                        
                        self.park_scenerio = True
                        self.mpc_start_settings()
                        self.parking_flag = True
                        rospy.loginfo("self.state: park section id:1")

                    if  "motorway" in self.yolo_data.object and self.motorway_flag == False:
                        
                        self.motorway_scenerio = True
                        self.mpc_start_settings()
                        self.motorway_flag = True

                    if "endofmotorway" in self.yolo_data.object and self.motorway_flag == False:
                        
                        self.motorway_scenerio = False
                        self.mpc_start_settings()
                        self.motorway_flag = False

                    matching_pairs = [pair for pair in self.SourceTargetNodes if pair[0] == current_id]
                    print("matching_pairs",matching_pairs)
                    # print("pathGoalsYawDegree",self.pathGoalsYawDegree)
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
                        print("matching_entry",matching_entry)
                        if matching_entry is not None:
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
                            self.last_time_went_node = rospy.Time.now()


                            if next_id in self.parking_nodes_id:
                                self.state_target = ca.DM([target_x, target_y, 0.0])
                                self.distance_flag = False
                            else:
                                self.state_target = ca.DM([target_x, target_y, yaw])
                                self.distance_flag = True

                        
                            self.goal_id = matching_entry  
                            if self.state == "keep lane":         
                                self.last_update_time = rospy.Time.now()     

                        elif matching_entry is None:
                            self.distance_flag = False
                            self.state_target = ca.DM([self.position_x, self.position_y, self.yaw_rad])
                            self.last_update_time = rospy.Time.now()

                if self.distance_flag == True:
                                           
                        # if self.traffic_light_master[0] == 2 and self.current_id =="489":
                        #     rospy.loginfo("self.traffic_light_master[0] == 0")
                        #     self.steerAngle = math.degrees(0.0) 
                        #     self.steerLateral = float(0.0)
                        #     self.state = "waiting on traffic light"
                        #     # rospy.loginfo("self.state: {}".format(self.state))
                        #     self.carControlPublisher()

                        # if self.traffic_light_slave[0] == 2  and self.current_id == "135":
                        #     rospy.loginfo("self.traffic_light_slave[0] == 0")
                        #     self.steerAngle = math.degrees(0.0) 
                        #     self.steerLateral = float(0.0)
                        #     self.state = "waiting on traffic light"
                        #     # rospy.loginfo("self.state: {}".format(self.state))
                        #     self.carControlPublisher()

                        # if self.traffic_light_start[0] == 2 and self.current_id =="121":
                        #     rospy.loginfo("self.traffic_light_start[0] == 0")  
                        #     self.steerAngle = math.degrees(0.0) 
                        #     self.steerLateral = float(0.0)
                        #     self.state = "waiting on traffic light"
                        #     # rospy.loginfo("self.state: {}".format(self.state))
                        #     self.carControlPublisher()


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
        rospy.loginfo("yolo_intercept_callback")
        self.yolo_data = MekatronomYolo()
        self.yolo_data.object = data.object
        rospy.loginfo("self.yolo_data.object: {}".format(self.yolo_data.object))


    def behaviourTimerCallback(self,event):
        rospy.loginfo("behaviourTimerCallback")

        if (rospy.Time.now() - self.last_time_went_node  >= rospy.Duration(10.0)) and (rospy.Time.now() - self.last_update_time_obstacles_checking >= rospy.Duration(10.0)):
            rospy.loginfo("behaviourTimerCallback Last Time Went Node and Last Update Time Obstacles Checking")
            
            self.last_time_went_node = rospy.Time.now()
            self.process_and_publish_data(self.current_id_original,self.target_node)

            if self.mpc_started:
                matching_pairs = [pair for pair in self.SourceTargetNodesCopy if pair[0] == self.goal_id[0]]
                rospy.loginfo("behaviourtimercallback inside of it")
                next_id = matching_pairs[0][1] 
                
                matching_entry = next((entry for entry in self.pathGoalsYawDegreeCopy if entry[0] == next_id))
                self.goal_id = matching_entry
                if matching_entry:
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

                    print(f'Step {1} -> ID: {next_id}, X: {target_x:.2f}, Y: {target_y:.2f}, Yaw: {yaw:.2f} rad')
                    

                    self.state_target = ca.DM([target_x, target_y, yaw])
                    self.carControlPublisher()

        if rospy.Time.now() - self.last_update_time_obstacles_checking >= rospy.Duration(10.0):
            self.last_update_time_obstacles_checking = rospy.Time.now()
            self.intercept_targets = ["243","261","252","54","51","56","317","367","397","404","34","84","163","80","93","176",
                                            "91","14","18","2","6","143","102","32","16","222","219","30","38","100","26","42",
                                            "109","113","121","75","185","71","40","205","226","283","198"]
        
            self.crosswalk_scenerio = False
            self.crosswalk_flag = False


            if self.obstacles_array:  
                print("hi im here behavior")
                print("self.obstacles_array",self.obstacles_array)
                for value in self.obstacles_array:
                    if value in self.obs_dontuse:
                        self.obs_dontuse.remove(value)
                self.obstacles_array = []
                print("angel kalktı ")
                # self.process_and_publish_data(self.current_id_original,self.target_node)#curent den global targete göndericem
                self.last_update_time_obstacles_checking = rospy.Time.now()

        if rospy.Time.now() - self.crosswalk_update_time >= rospy.Duration(5.0) and self.crosswalk_flag == True:
            self.crosswalk_update_time = rospy.Time.now()
            self.crosswalk_flag = False
            self.crosswalk_scenerio = False
            self.mpc_start_settings()

        if rospy.Time.now() - self.last_update_time >= rospy.Duration(1.0):

            if self.state == "waiting on traffic light":

                if self.traffic_light_master[0] == 2 and self.current_id =="489":
                    self.state = "keep lane"
                if self.traffic_light_slave[0] == 2  and self.current_id == "135":
                    self.state = "keep lane"
                if self.traffic_light_start[0] == 2 and self.current_id =="121":
                    self.state = "keep lane"
                   
                matching_pairs = [pair for pair in self.SourceTargetNodes if pair[0] == self.goal_id[0]]

                if matching_pairs and self.state == "keep lane":
                    next_id = matching_pairs[0][1] 
                    
                    #print("self.pathGoalsYawDegree:",self.pathGoalsYawDegree)
                    #for i in range(1, 5):
                        
                    matching_entry = next((entry for entry in self.pathGoalsYawDegree if entry[0] == next_id))
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
                        

                        self.state_target = ca.DM([target_x, target_y, yaw])

                self.last_update_time = rospy.Time.now()

        if rospy.Time.now() - self.last_update_time >= rospy.Duration(5.0):


            if self.state == "waiting on intercept":

                self.state = "keep lane"
                self.intercept_targets.remove(self.current_id)  # current_id'yi listeden kaldır
                matching_pairs = [pair for pair in self.SourceTargetNodes if pair[0] == self.goal_id[0]]

                if matching_pairs:
                    next_id = matching_pairs[0][1]
                    
                    matching_entry = next((entry for entry in self.pathGoalsYawDegree if entry[0] == next_id))
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
                        
                        self.state_target = ca.DM([target_x, target_y, yaw])

                self.last_update_time = rospy.Time.now()

        else:
            pass

        
    #OLD# new path finding

    #####################   NEW PATH FINDING     #######################
    def process_and_publish_data(self,temp_source,temp_target):
        # GraphML dosyasını okuma
        print("burdayım :",self.current_id," orjinal noktam  :",self.current_id_original)

        self.callnumber=self.callnumber+1
        stx = time.time()# dosyadan endge okuma işelmleri zaman ölçümü için

        tree = ET.parse(self.file_path_original)
        root = tree.getroot()
        self.nodes_data = self.extract_nodes_data(root)


        self.obstacle_node_positions = {node_id: (x, y) for node_id, x, y in self.nodes_data if node_id in self.parking_nodes_id}


        self.edges_data, self.edges_data_true_ilkverisyon = self.extract_edges_data(root)


        flagsolla=self.flagsolla
        for ciz in self.edges_data:#self.obs_dontuse listesindekilerden herhangi biri kesikli çizgi üstüne geliyo mu kontrolü
            for kx in self.obs_dontuse:
                if  ciz[2]==True:#ciz[1]==kx and ciz[2]==True: yine benim aptallığım eğer flag sollaya daha once müdehale edilmemişse ve engel kalktığı senaryoda bu ife girmiycek kendi şeridine geri dönemiycek
                    flagsolla=1
                    self.flagsolla = 1
                    #print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@solla")
                    break#fazladan dolaşmasın bi kere bulması yeterli
                #print("################ : ",ciz[2])

        if flagsolla==0:
            self.flagsolla=0
        etx = time.time()
        elapsed_timex = etx - stx
        print('Execution time:', elapsed_timex, 'seconds##################-----------------####################')


        if flagsolla==1:
            #sollama kısmı için özel graph dosyasını açıcam  ikinci bi edge çıkarma işlemi umarım fazla yavaşlatmaz
            #şu anda 0.008 saniye
            tree2 = ET.parse(self.graphml_file_path)
            root2 = tree2.getroot()
            #print("root : ",root2)
            self.nodes_data = self.extract_nodes_data(root2)
            self.edges_data, self.edges_data_true_ilkverisyon = self.extract_edges_data(root2)



        noded,edged=self.extract_graph()# (nodedict,edgedict) döndürür

        print("noded_first:",noded)

        #self.obs_dontuse=["360","314","321","344","315","270","367","64","169","326"]

        path_short=self.dijkstra(temp_source,temp_target,noded,edged,self.obs_dontuse)#nodedictionary =noded
        print("pathshort:",path_short)
        self.path_original = self.stformat(path_short)
        print("noded_second:",noded)
        self.nodedatabest=noded
        print("pathoriginal:",self.path_original)
        #print(noded)


        newnodedictionary,stlist=self.beizer(path_short,noded)#beizere targete giden path i veriyorum
        self.obs_dict = newnodedictionary#yeni node sozlüğü
        self.edges_data_true =stlist#source target list
        # print("self.obs_dict",self.obs_dict)
        # print("self.edges_data_true",self.edges_data_true)
        #print("-----------------------------------------")


        #print("-----------------------------------------")
        #nodedata ,sourcetarget
        #beizer çıktısı 1 : source targetnodes tipinde path  2:yeni eklenen noktalarıda içeren node_data o datayıda self.obs_dicte vericem
        #409 da döngüye kapılıyo çıkamıyo
        #print("edges_data_true",self.edges_data_true)
        #print("path_original",self.path_original)

        self.SourceTargetNodes = [(edge[0], edge[1]) for edge in self.edges_data_true if edge[2]]
        self.SourceTargetNodesOriginal = [(edge[0], edge[1]) for edge in self.path_original if edge[2]]
        #self.SourceTargetNodes edge formatında benim pathi edge formatına çevirmemlazım
        #burda sourceın içine hedef yolu vermek için edges data ture formatında en kısa yolu ustte vermemlazım
        #print("sorutceTrgetnodes:",self.edges_data_true)
        #print("**************************************************")
        #print("**************************************************")

        self.path = []
        self.pathOriginal = []


        for source_id, target_id in self.SourceTargetNodesOriginal:
            if source_id in self.obs_dict:
                source_coords = self.obs_dict[source_id]
                self.pathOriginal.append((source_id, source_coords[0], source_coords[1]))
            if target_id in self.obs_dict:
                coords = self.obs_dict[target_id]
                self.pathOriginal.append((target_id, coords[0], coords[1]))
            # if source_id in self.obs_dict:
            #     coords = self.obs_dict[source_id]
            #     self.path.append((source_id, coords[0], coords[1]))

        # print("\nself.path_original::::\n",self.pathOriginal)

        for source_id, target_id in self.SourceTargetNodes:
            if target_id in self.obs_dict:
                coords = self.obs_dict[target_id]
                self.path.append((target_id, coords[0], coords[1]))


        # Her bir nokta için bir sonraki nokta ile arasındaki açıyı hesaplama
        angles = []
        anglesOriginal = [] 

        for i in range(len(self.pathOriginal) - 1):
            dx = self.pathOriginal[i + 1][1] - self.pathOriginal[i][1]
            # print("self.pathOriginal[i + 1][1]",self.pathOriginal[i + 1][1])
            # print("self.pathOriginal[i][1]",self.pathOriginal[i][1])
            dy = self.pathOriginal[i + 1][2] - self.pathOriginal[i][2]

            angle = math.atan2(dy, dx)
            anglesOriginal.append(angle)

        # print("anglesOriginal",anglesOriginal)

        for i in range(len(self.path) - 1):
            dx = self.path[i + 1][1] - self.path[i][1]
            dy = self.path[i + 1][2] - self.path[i][2]
            angle = math.atan2(dy, dx)
            angles.append(angle)

        # print("angles",angles)

        if angles:  # Check if angles list is not empty
            angles.append(angles[-1])

        if anglesOriginal:  
            anglesOriginal.append(anglesOriginal[-1])

        self.pathGoalsYawDegree = [(*p, angle) for p, angle in zip(self.path, angles)]
        self.pathGoalsYawDegreeOriginal = [(*p, angle) for p, angle in zip(self.pathOriginal, anglesOriginal)]

        # print("\nself.pathGoalsYawDegree\n",self.pathGoalsYawDegree)
        # print("\nself.pathGoalsYawDegreeOriginal\n",self.pathGoalsYawDegreeOriginal)

        if not self.pathGoalsYawDegreecalled:
            self.pathGoalsYawDegreeCopy = self.pathGoalsYawDegreeOriginal
            self.SourceTargetNodesCopy = self.SourceTargetNodesOriginal
            self.pathGoalsYawDegreecalled = True

        # Data publishing simulation
        data_message = str(self.edges_data_true)



    def stformat(self,new_path):
        source_target=[]
        pathlen=len(new_path)
        #print("len :",pathlen)
        for n_edge in range(pathlen-1):
            source_target.append((new_path[n_edge],new_path[n_edge+1],True))
        return source_target


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
                    d1 = float(data.text)  # 'y' koordinatı
                # past version is =                     d1 = 13.72-float(data.text)  # 'y' koordinatı

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
                sollacurrent=False
                for ciz in self.edges_data:#self.obs_dontuse listesindekilerden herhangi biri kesikli çizgi üstüne geliyo mu kontrolü
                    if ciz[0]==node[0]:
                        sollacurrent=ciz[2]
                        break#fazladan dolaşmasın bi kere bulması yeterli
        
                nodedict[node[0]]={'mesafe':inf,'atalist':[],'solla':sollacurrent,'x':node[1],'y':node[2]}#0  1 di   1  2 olarak düzelttim

            for edge in graph_temp:

                temp=edge[0]
                edgemesafe =1#edge 0 ve edge 1 i verip ikisi arası mesafeyi vericem
                if temp in edgedict.keys():

                    edgedict[edge[0]].append([edge[1],1])

                else:
                    edgedict[edge[0]]=[[edge[1],1]]
                    #tuple dan list tipine cevirdim


                edgetemp =edgedict[edge[0]]

            return (nodedict,edgedict)
    
    def minyol(self,edge_curent_source):
       # print(edge_curent_source)
        #print(edge_curent_source[0])
        #şimdilik her edge ağırlığı 1 diye önce 1. yi seçicek ama sonradan  mesafe hesapolı edgelerde bu kısmı lazım
        min=edge_curent_source[0][1]
        min_id=edge_curent_source[0][0]
        for edge in edge_curent_source:
            if edge[1]<min:
                min=edge[1]
                min_id=edge
        return min_id
    def hedef_mesafe(self,X1,Y1,X2,Y2):

        x1=float(X1)
        x2=float(X2)
        y1=float(Y1)
        y2=float(Y2)


        mesafe_hedef=math.sqrt(((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2)))
        #print("mesafe fark var  ----------------------------------------:",mesafe_hedef)
        return mesafe_hedef
           
    def dijkstra(self,source ,target,nodedictt,edgedictt,yasaklistesi):
        #print("edgedict:",edgedictt)
        print("____________________",source)
        print("________self____________",self.source_node)
        nowaypoints=[]#çıkamzı döndürmek için tutuğum dizi
        nowaypoints2=[]#yanı indekste tutmak için (targeti tutmak için)
        for ed in  list(edgedictt.items()):
            for jkl in edgedictt[ed[0]]:#jkl liste objesi
                for dont in yasaklistesi:
                    edg=str(ed[0])
                    if edg in edgedictt:
                        if  jkl[0]==dont:

                            nowaypoints.append(edg)
                            nowaypoints2.append(jkl[0])
                            temppp=edgedictt[ed[0]]
                            for test in temppp:
                                if test==jkl:
                                    temppp.remove(test)
                            edgedictt[ed[0]]=temppp


        unvisited={n:float('inf') for n in edgedictt.keys()}
        #print("unvisited:",unvisited)
        unvisited[source]=0
        # print("en içteki kontrol source ::::",source)
        #revPath={}
        visited ={}
        a=0#teminal sonsuzda kalmasın diye
        while unvisited  :

            minNode=min(unvisited, key=unvisited.get)
            # print("minode:",minNode)
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

        nodedictt[target]['atalist']=nodedictt[target]['atalist']+[target]
        yolll=nodedictt[target]['atalist']


        if source in yolll and target in yolll:

            trflstate=False#şimdilik false yapıyorum trafik ışığının kırmızı ise True olucak bu değişken 
            yayastate=False#yolda onümüze random noktada yayay çıkma senaryosunda bu true olucak
            expathstate=False#bir önceki yol kısa mı kullanılabilir mi çok kısamı gibi bi kontrol yapabilirim 
            if(trflstate or yayastate or expathstate ):


                for tempstopxx in yasaklistesi:
                    if tempstopxx in self.expath:
                        bagendxx=tempstopxx
                        break
                indexnoEXxx=self.expath.index(bagendxx)
                # print("ex path:",self.expath)
                if len(self.expath)>2 :
                    #TODO: burası eskiden -2 idi -1yapmayı deniyoruz -2 de çalışıyor ama 2 node öncesinje veriyor irdelenecek.
                    befbagendxx=self.expath[indexnoEXxx-1]#sourn burda olabillir tek nokta kalan path de indexi bir geriye alıp patlıyor olabilir
                else :
                    befbagendxx=self.expath[indexnoEXxx]#bunu test et
                # print("before bagend",befbagendxx)
                # print("bagend :::",bagendxx)
                nodedictt[befbagendxx]['atalist']=nodedictt[befbagendxx]['atalist']+[befbagendxx]
                return nodedictt[befbagendxx]['atalist']

            else:#trafik ışığı kırmız değilse  yaya yoksa ve bi onceki yol düzgün değilse  çıkartılan yolu versin yani algoritmanın orijinal hali ile tepki versin 
                #orijinal hali

                self.yolvar=True
                self.expath=nodedictt[target]['atalist']

                return nodedictt[target]['atalist']
        else:

            newtargetn=""
            
            for tempstopx in yasaklistesi:
                if tempstopx in self.expath:
                    bagend=tempstopx
                    break

            indexnoEX=self.expath.index(bagend)
            # print("ex path:",self.expath)
            if len(self.expath)>2 :
                #TODO: burası eskiden -2 idi -1yapmayı deniyoruz -2 de çalışıyor ama 2 node öncesinje veriyor irdelenecek.
                befbagend=self.expath[indexnoEX-2]#sourn burda olabillir tek nokta kalan path de indexi bir geriye alıp patlıyor olabilir
            else :
                befbagend=self.expath[indexnoEX]#bunu test et

            nodedictt[befbagend]['atalist']=nodedictt[befbagend]['atalist']+[befbagend]
            return nodedictt[befbagend]['atalist']


        return nodedictt[target]['atalist']

    def plotbfmc(self,node_dat,path):
        #dosya adı için
        obj = time.localtime(1627987508.6496193)
        gren=[0,255,0]
        red=[255,255,255]
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
            if (int(idxy)-600)>0:
                cv2.circle(img, (x,y), 15, red, -1)
            else:
                cv2.circle(img, (x,y), 15, gren, -1)



        #cv2.circle(img, (400,800), 15, gren, -1)

        #cv2.imshow('img', img)
        cv2.imwrite(name+".png", img)
        cv2.waitKey(1)

    def beizer(self,path,node_d):
        new_node_data={}
        for idkey in node_d.keys():
            #print(idkey)
            new_node_data.update({idkey: (node_d[idkey]["x"],node_d[idkey]["y"])})
            #print("x:",x)
        #print("new_node_data:",new_node_data)
        new_point_counter=600#yeni düğmleri çakışma olmasın diye 600 den başlattım
        new_path=path.copy()#path in kopyasını aldım değişiklikleri ona uyguluycam itarete ederken  list den eleman silip ekliyince hata veriyo

        #print("----------------------")
        path_length=len(path)
        ppbuk=0
        
        for f in range(path_length-2):#2 idi daha smooth dönüş için 3 deniyorum ben şükrü

            #açı hesaplıycam
            if node_d[path[f]]["x"]==node_d[path[f+1]]["x"]:
                #sıfıra bölme durumu
                angel_rad1=1.57#radyan cinsinden 90derece
            else:
                angel_rad1=math.atan((node_d[path[f]]["y"]-node_d[path[f+1]]["y"])/(node_d[path[f]]["x"]-node_d[path[f+1]]["x"]))#arctan

            angel_deg1=angel_rad1*57.3
            #print("açı bir :",angel_deg1)

            if node_d[path[f+1]]["x"]==node_d[path[f+2]]["x"]:
                #sıfıra bölme durumu
                angel_rad2=1.57#radyan cinsinden 90derece
            else:
                angel_rad2=math.atan((node_d[path[f+1]]["y"]-node_d[path[f+2]]["y"])/(node_d[path[f+1]]["x"]-node_d[path[f+2]]["x"]))#arctan

            angel_deg2=angel_rad2*57.3
            #print("açı iki :",angel_deg2)

            b_andgel=abs(angel_deg1-angel_deg2)
            #print("birinci açı :",angel_deg1," ikinci açı :",angel_deg2)
            numout=1

            if b_andgel>55 and b_andgel<110 :#b_andgel>45 and b_andgel<110 and (ppbuk+1)!=f ppbuk sorunlu düzgün çalışmıyo 
                ppbuk=f
                if  True or   not (node_d[path[f]]["solla"] or node_d[path[f+1]]["solla"] or node_d[path[f+2]]["solla"]) :#if not self.flagsolla  : bu eski hali yeni hali olarak solla ile kontrol etsem burayı çok iyi olur 
                    #bezier ilk versiyon orijinal 
       
                    numPts=2
                    numout=2
        
            
                    controlPts=[[node_d[path[f]]["x"],node_d[path[f]]["y"]],[node_d[path[f+1]]["x"],node_d[path[f+1]]["y"]],[node_d[path[f+2]]["x"],node_d[path[f+2]]["y"]]]# control points
                    t=np.array([i*1/numPts for i in range(0,numPts+1)])

                    B_x=(1-t)*((1-t)*controlPts[0][0]+t*controlPts[1][0])+t*((1-t)*controlPts[1][0]+t*controlPts[2][0])#yeni noktaların x i
                    B_y=(1-t)*((1-t)*controlPts[0][1]+t*controlPts[1][1])+t*((1-t)*controlPts[1][1]+t*controlPts[2][1])#yeni y si
                    temp_new_nodelist=[]
                    for new_p in range(1,numPts):#+1 yapıp bütün noktaları yazdırdım  yeni düğüm eklerken 1 den numPts ye kadar gezip yeni düğüm ekliycem
                     

                        self.new_point_ctr=self.new_point_ctr+1
                        new_point_str=str(self.new_point_ctr)
                        #print("yeni nokta ",new_p,"x:",B_x[new_p],"y:",B_y[new_p])
                        new_node_data.update({new_point_str: (B_x[new_p],B_y[new_p])})
                        if self.nodedatabest[path[f]]["solla"] :
                            self.nodedatabest[new_point_str] = {'mesafe': float('inf'), 'atalist': [], 'solla': True, 'x': B_x[new_p], 'y': B_y[new_p]}
                            #self.nodedatabest.update({new_point_str: (99999,[],True,B_x[new_p],B_y[new_p])})#nezihe sor 
                        temp_new_nodelist.append(new_point_str)#nodu temp liste ekliyorum
                    new_path.remove(path[f+1])#üst if 
      

                temp_index=path.index(path[f+1])
    
                for insrt in temp_new_nodelist:
                    new_path.insert(temp_index,insrt)
                    temp_index=temp_index+1
        new_new_path=new_path.copy()
        pathlen22= len(new_path)
        print("::::::::::::::::::::::",new_new_path)
        for djj in range(pathlen22-3):
            #print("şşş:",new_new_path[djj],"  ",int(new_new_path[djj])/100==6)
            if int(new_new_path[djj])> 600 and int(new_new_path[djj])< 800:
                # nodedict[node[0]]={'mesafe':inf,'atalist':[],'solla':sollacurrent,'x':node[1],'y':node[2]}
                if new_new_path[djj-1] in self.nodedatabest.keys() or new_new_path[djj+1] in self.nodedatabest.keys() :
                    
                    print(self.nodedatabest[new_new_path[djj-1]]["solla"] ,"   ",self.nodedatabest[new_new_path[djj+1]]["solla"])
                    if self.nodedatabest[new_new_path[djj-1]]["solla"] or self.nodedatabest[new_new_path[djj+1]]["solla"]:
                        new_new_path.remove(new_new_path[djj])#break ver belki çalışır


        new_path=new_new_path
        print("new path :",new_new_path)#en son path
        source_target=[]
        pathlen=len(new_new_path)
        #print("len :",pathlen)
        for n_edge in range(pathlen-1):
            source_target.append((new_new_path[n_edge],new_new_path[n_edge+1],True))

        return new_node_data,source_target



    def extract_edges_data(self, root):
        edges_data = []
        edges_data_true = []
        for edge in root.findall(".//{http://graphml.graphdrawing.org/xmlns}edge"):
            source_id = edge.get('source')
            target_id = edge.get('target')
            data_d2 = edge.find(".//{http://graphml.graphdrawing.org/xmlns}data[@key='d2']")
            d2_value = data_d2 is not None and data_d2.text == 'True'
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

    def carControlPublisher(self):
        carData = {}
        carData = {
            'action': '2',
            'steerAngle': self.steerAngle 
            # 'steerAngle': 0.0
        }
        # print("data",carData)
        self.carData = json.dumps(carData)
        self.carControl.publish(self.carData)
        #rospy.sleep(0.01)
        car2Data = {}
        # İkinci mesaj için veriler
        car2Data = {
            'action': '1',
            'speed': self.steerLateral  
            # 'speed': 0.0
        }   
        self.car2Data = json.dumps(car2Data)
        self.carControl.publish(self.car2Data)
        # print("self.steerAngle",self.steerAngle)
        # print("self.steerLateral",self.steerLateral)
        
    def localisation_callback(self, data):
        # rospy.loginfo("Received localisation data - Timestamp: %s, posA: %f, posB: %f, rotA: %f, rotB: %f",
        #                 data.timestamp, data.posA , 13.8-data.posB , data.rotA, data.rotB)

        self.position_x = data.pose.pose.position.x
        self.position_y = data.pose.pose.position.y
        
        localisation_yaw_rad = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]

        # rospy.loginfo("Received IMU data yawlocalisation: %f", localisation_yaw_rad)
        # #rospy.loginfo(f'Received localisation data - Timestamp: {data.timestamp}, posA: {data.posA}, posB: {data.posB}, rotA: {data.rotA}, rotB: {data.rotB}')
        self.localisation_CB = True

        
    def imu_callback(self, data):
        
        self.yaw_rad = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])[2]

        # rospy.loginfo("Received IMU data yaw: %f", self.yaw_rad)
        self.IMU_cb = True

    
    #Callbacks Finish    
        
    def image_callback(self, data):
        try:

            if  self.localisation_CB == True and self.IMU_cb == True:

                # if self.debug:
                #     self.debug_callback()
                pass

        except AttributeError as e:
            #print(e)
            rospy.logerr(e)

    def traffic_light_master_callback(self, data):
        # print("traffic_light_master_callback")
        position_x = 4.21
        position_y = 3.30
        self.traffic_light_master = [data.data, position_x, position_y]

    def traffic_light_slave_callback(self, data):
        # print("traffic_light_slave_callback")
        position_x = 2.23
        position_y = 4.54
        self.traffic_light_slave = [data.data, position_x, position_y]

    def traffic_light_start_callback(self, data):

        position_x = 5.63
        position_y = 4.30
        self.traffic_light_start = [data.data, position_x, position_y]
        

    #Debug Start
    def debug_callback(self):
        current_time = rospy.Time.now().to_sec()
        PassedTime = current_time - self.prev_time
        # rospy.loginfo("Elapsed time: %.4f seconds", PassedTime)
        self.prev_time = current_time



    #Debug Finish

    def shutdown_hook(self):
        # This function will be called during the shutdown process.
        print("Shutting down, sending stop commands...")
        stop_data = json.dumps({
            'action': '2',
            'steerAngle': 0.0,
        })
        self.carControl.publish(stop_data)
        
        stop_data2 = json.dumps({
            'action': '1',
            'speed': 0.0,
        })
        self.carControl.publish(stop_data2)
        rospy.sleep(0.5)  # Give some time for the message to be sent

def main():
    rospy.init_node('mpc', anonymous=False)
    mpc = MPC()
    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
    finally:
        rospy.signal_shutdown('Manual shutdown')  # Ensure rospy shutdown hooks are called
        

if __name__ == '__main__':
    main()