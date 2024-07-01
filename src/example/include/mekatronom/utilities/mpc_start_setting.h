/*
* MIT License
* 
* Copyright (c) 2024 Mehmet Baha Dursun
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

/*
 * Author: Mehmet Baha Dursun
 */

#pragma once

#include "ros/ros.h"
#include "casadi/casadi.hpp"
#include "mekatronom/MpcNode.hpp"

using namespace casadi;


class MpcStartSetting
{
public:
    MpcStartSetting(const std::string& scenerio_name_, ros::NodeHandle& nh_local_, ros::NodeHandle& nh_, MpcNode& node)
    {
        ROS_INFO("scenerio_name: %s", scenerio_name_.c_str());

        //TODO: Those settings tuned by hand for different scenerios. They should be tuned from here
        
        try {
            
            MpcNode::Settings initial_settings{
                1.0, 1.0, 1., 1.0, 1.8, 0.2, 12., 0.354, 1., 0.4, 0.04, 0.33,-0.1, M_PI / 7.5, -M_PI / 7.5
            };
            MpcNode::Settings park_scenerio_settings{
                1.0, 4.0, 0.05, 0.01, 0.02, 0.1, 6., 0.354, 1., 0.3, 0.04, 0.2,-0.25, M_PI / 7.5, -M_PI / 7.5
            };
            MpcNode::Settings crosswalk_scenerio_settings{
                1.0, 1.0, 1., 1., 1.8, 0.1, 12., 0.354, 1., 0.3, 0.04, 0.13,-0.2, M_PI / 7.5, -M_PI / 7.5
            };
            MpcNode::Settings motorway_scenerio_settings{
                1.0, 1.0, 2., 2., 2.8, 0.2, 4., 0.354, 1., 0.5, 0.04, 0.5,-0.1, M_PI / 7.5, -M_PI / 7.5
            };
            
            
            if (scenerio_name_ == "park") { 
                ROS_INFO("The scenario is park.");
                // node.initial_settings_ = park_scenerio_settings;
            } else if (scenerio_name_ == "crosswalk") {
                ROS_INFO("The scenario is crosswalk.");
                // node.initial_settings_ = crosswalk_scenerio_settings;
            } else if (scenerio_name_ == "motorway") {
                ROS_INFO("The scenario is motorway.");
                // node.initial_settings_ = motorway_scenerio_settings;
            } 
            else {
                // Handle the case where scenerio_name_ is not recognized.
                // You can choose to throw an exception, log an error, or provide a default value.
                // Here, we set it to initial_settings_ as a fallback.
                // node.initial_settings_ = initial_settings;
                ROS_INFO("The scenario is not recognized. The default settings are set.");
            }

            ROS_INFO("The settings are set for the scenario: %s", node.initial_settings_.Q_x);
            ROS_INFO("The settings are set for the scenario: %s", node.initial_settings_.Q_y);
            ROS_INFO("The settings are set for the scenario: %s", node.initial_settings_.Q_theta);
            ROS_INFO("The settings are set for the scenario: %s", node.initial_settings_.R1);
            ROS_INFO("The settings are set for the scenario: %s", node.initial_settings_.R2);
            ROS_INFO("The settings are set for the scenario: %s", node.initial_settings_.step_horizon);
            ROS_INFO("The settings are set for the scenario: %s", node.initial_settings_.rob_diam);
            ROS_INFO("The settings are set for the scenario: %s", node.initial_settings_.wheel_radius);
            ROS_INFO("The settings are set for the scenario: %s", node.initial_settings_.L_value);
            ROS_INFO("The settings are set for the scenario: %s", node.initial_settings_.Ly);
            ROS_INFO("The settings are set for the scenario: %s", node.initial_settings_.v_max);
            ROS_INFO("The settings are set for the scenario: %s", node.initial_settings_.v_min);
            ROS_INFO("The settings are set for the scenario: %s", node.initial_settings_.omega_max);
            ROS_INFO("The settings are set for the scenario: %s", node.initial_settings_.omega_min);

        } catch (const std::exception& e) {
            ROS_FATAL("Exception caught during setting the scenario: %s", e.what());
            return;
        }
        
        // // Dönüşüm matrisini oluştur
        // Eigen::Matrix2d rotationMatrix;
        
        // rotationMatrix << std::cos(localisation_data_.yaw ), -std::sin(localisation_data_.yaw ),
        //                 std::sin(localisation_data_.yaw ), std::cos(localisation_data_.yaw );

        // ROS_INFO("imu_data_.yaw: %f", localisation_data_.yaw);
        // // Dönüştürülecek matrisi tanımla
        // Eigen::Vector2d matrix(1, 4);

        // // Dönüşümü uygula
        // Eigen::Vector2d result = rotationMatrix * matrix;

        // Başlangıç ve hedef durumları tanımla
        DM state_init = DM::vertcat({node.localisation_data_.x, node.localisation_data_.y, node.localisation_data_.yaw});
        DM state_target = DM::vertcat({12.09, 13.72-11.88, 0.0});

        // Durum sembolik değişkenlerini tanımla
        SX x = SX::sym("x");
        SX y = SX::sym("y");
        SX theta = SX::sym("theta");
        SX states = vertcat(x, y, theta);
        int n_states = states.size1() * states.size2(); // numel yerine size1 ve size2 çarpımı kullanılır
        std::cout << "n_states: " << n_states << std::endl;

        // new define
        SX v = SX::sym("v");
        SX omega = SX::sym("omega");
        SX controls = vertcat(v, omega);
        int n_controls = controls.numel();

        // matrix containing all states over all time steps +1 (each column is a state vector)
        SX X = SX::sym("X", n_states, (node.initial_settings_.N + 1));

        // matrix containing all control actions over all time steps (each column is an action vector)
        SX U = SX::sym("U", n_controls, node.initial_settings_.N);

        // column vector for storing initial state and target state
        SX P = SX::sym("P", n_states + n_states);

        // state weights matrix (Q_X, Q_Y, Q_THETA)
        std::vector<SX> Q_elements;
        Q_elements.push_back(SX::diag(node.initial_settings_.Q_x)); // Assuming Q_x, Q_y, Q_theta are already defined SX expressions
        Q_elements.push_back(SX::diag(node.initial_settings_.Q_y));
        Q_elements.push_back(SX::diag(node.initial_settings_.Q_theta));
        SX Q = diagcat(Q_elements); // Use vector for diagcat

        // Correct usage of diagcat for R as well
        std::vector<SX> R_elements;
        R_elements.push_back(SX::diag(node.initial_settings_.R1)); // Assuming R1, R2 are already defined SX expressions
        R_elements.push_back(SX::diag(node.initial_settings_.R2));
        SX R = diagcat(R_elements); // Use vector for diagcat


        // // discretization model (e.g. x2 = f(x1, v, t) = x1 + v * dt)
        // SX rot_3d_z = vertcat(
        //     horzcat(cos(theta), -sin(theta), 0),
        //     horzcat(sin(theta), cos(theta), 0),
        //     horzcat(0, 0, 1)
        // );

        // // Mecanum wheel transfer function
        // DM J = (wheel_radius / 4) * DM({
        //     {1, 1, 1, 1},
        //     {-1, 1, 1, -1},
        //     {-1 / (Lx + Ly), 1 / (Lx + Ly), -1 / (Lx + Ly), 1 / (Lx + Ly)}
        // });

        SX RHS = vertcat(
            v * cos(theta),
            v * sin(theta),
            omega
        );

        // maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T
        Function f = Function("f", {states, controls}, {RHS});

        SX cost_fn = 0; // cost function
        SX g = X(Slice(), 0) - P(Slice(0, n_states)); // constraints in the equation

        SX st = X(Slice(), 0);
        std::cout << "st: " << st << std::endl;
        std::cout << "g: " << g << std::endl;

        for (int k = 0; k < node.initial_settings_.N; ++k) {
            SX st = X(Slice(), k);
            SX con = U(Slice(), k);

            // Maliyet fonksiyonunu hesapla
            cost_fn += mtimes(mtimes((st - P(Slice(3, 6), 0)).T(), Q), (st - P(Slice(3, 6), 0))) + 
                    mtimes(mtimes(con.T(), R), con);

            // Bir sonraki durumu tahmin et
            SX st_next = X(Slice(), k+1);

            auto k1 = f(std::vector<SX>{st, con});
            auto k2 = f(std::vector<SX>{st + (node.initial_settings_.step_horizon / 2) * k1.at(0), con});
            auto k3 = f(std::vector<SX>{st + (node.initial_settings_.step_horizon / 2) * k2.at(0), con});
            auto k4 = f(std::vector<SX>{st + node.initial_settings_.step_horizon * k3.at(0), con});

            SX st_next_RK4 = st + (node.initial_settings_.step_horizon / 6) * (k1.at(0) + 2*k2.at(0) + 2*k3.at(0) + k4.at(0));

            // Kısıtlama vektörünü güncelle
            g = vertcat(g, st_next - st_next_RK4);

        }
        std::cout << "cost_fn: " << cost_fn << std::endl;
        std::cout << "g: " << g << std::endl;

        SX OPT_variables = vertcat(reshape(X, n_states * (node.initial_settings_.N + 1), 1), reshape(U, n_controls * node.initial_settings_.N, 1));

        // nlp_prob sözlüğünü tanımlama
        SXDict nlp_prob = {
            {"f", cost_fn},
            {"x", OPT_variables},
            {"g", g},
            {"p", P}
        };
        // Solver ayarlarını tanımlama
        Dict opts = {
            {"ipopt.max_iter", 100},
            {"ipopt.print_level", 0},
            {"ipopt.acceptable_tol", 1e-8},
            {"ipopt.acceptable_obj_change_tol", 1e-6},
            {"print_time", 0}
        };


        Function solver = nlpsol("solver", "ipopt", nlp_prob, opts);
        std::cout << "Solver " << solver << std::endl;

        DM lbx = DM::zeros(n_states*(node.initial_settings_.N+1) + n_controls*node.initial_settings_.N, 1);
        DM ubx = DM::zeros(n_states*(node.initial_settings_.N+1) + n_controls*node.initial_settings_.N, 1);
        DM lbg = DM::zeros(n_states*(node.initial_settings_.N+1), 1);
        DM ubg = DM::zeros(n_states*(node.initial_settings_.N+1), 1);

        // Sınırları ayarlama
        for (int i = 0; i < n_states*(node.initial_settings_.N+1); i+=n_states) {
            lbx(i) = -DM::inf();
            lbx(i+1) = -DM::inf();
            lbx(i+2) = -DM::inf();

            ubx(i) = DM::inf();
            ubx(i+1) = DM::inf();
            ubx(i+2) = DM::inf();
        }

        for (int i = n_states*(node.initial_settings_.N+1); i < n_states*(node.initial_settings_.N+1)+2*node.initial_settings_.N; i+=2) {
            lbx(i) = node.initial_settings_.v_min;
            ubx(i) = node.initial_settings_.v_max;
            lbx(i+1) = node.initial_settings_.omega_min;
            ubx(i+1) = node.initial_settings_.omega_max;
        }

        // İlk durum ve kontrol değerleri

        DM t0 = 0;
        DM u0 = DM::zeros(node.initial_settings_.N, 2); // İlk kontrol vektörü
        DM X0 = DM::repmat(state_init, 1, node.initial_settings_.N+1).T(); // İlk durum vektörü

        // Args sözlüğü oluşturma
        std::map<std::string, DM> args;
        args["lbg"] = lbg;
        args["ubg"] = ubg;
        args["lbx"] = lbx;
        args["ubx"] = ubx;

        std::cout << "args: " << args << std::endl;


    }
};