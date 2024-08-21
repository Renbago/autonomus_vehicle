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

#include "mekatronom/MpcNode.hpp"


class MpcRunning
{

public:
    MpcRunning(MpcNode& node)
    {
        try {

            ROS_INFO_ONCE("MpcRunning constructor");

            node.mpc_setting_outputs_.args["p"] = vertcat(
                node.mpc_setting_outputs_.state_init,
                node.mpc_setting_outputs_.state_target
            );
            node.mpc_setting_outputs_.args["x0"] = vertcat(
                reshape(node.mpc_setting_outputs_.X0.T(), node.mpc_setting_outputs_.n_states * (node.initial_settings_.N + 1), 1),
                reshape(node.mpc_setting_outputs_.u0.T(), node.mpc_setting_outputs_.n_controls * node.initial_settings_.N, 1)
            );

            // std::cout << "Solver setup: " << node.mpc_setting_outputs_.solver << std::endl;
            // // Debug values
            // std::cout << "x0 values: " << node.mpc_setting_outputs_.args["x0"] << std::endl;
            // std::cout << "lbx values: " << node.mpc_setting_outputs_.args["lbx"] << std::endl;
            // std::cout << "ubx values: " << node.mpc_setting_outputs_.args["ubx"] << std::endl;
            // std::cout << "lbg values: " << node.mpc_setting_outputs_.args["lbg"] << std::endl;
            // std::cout << "ubg values: " << node.mpc_setting_outputs_.args["ubg"] << std::endl;
            // std::cout << "p values: " << node.mpc_setting_outputs_.args["p"] << std::endl;

            // Example initialization of CasADi DM matrices
            DM x0 = node.mpc_setting_outputs_.args["x0"];
            casadi::DM lbx = node.mpc_setting_outputs_.args["lbx"];
            casadi::DM ubx = node.mpc_setting_outputs_.args["ubx"];
            casadi::DM lbg = node.mpc_setting_outputs_.args["lbg"];
            casadi::DM ubg = node.mpc_setting_outputs_.args["ubg"];
            casadi::DM p = node.mpc_setting_outputs_.args["p"];


            // Check for any unexpected negative sizes
            if (x0.size1() < 0 || x0.size2() < 0 ||
                lbx.size1() < 0 || lbx.size2() < 0 ||
                ubx.size1() < 0 || ubx.size2() < 0 ||
                lbg.size1() < 0 || lbg.size2() < 0 ||
                ubg.size1() < 0 || ubg.size2() < 0 ||
                p.size1() < 0 || p.size2() < 0) {
                std::cerr << "Error: Negative dimensions detected!" << std::endl;
            }

            // Prepare the solver arguments
            DMDict args = {
                {"x0", node.mpc_setting_outputs_.args["x0"]},
                {"lbx", node.mpc_setting_outputs_.args["lbx"]},
                {"ubx", node.mpc_setting_outputs_.args["ubx"]},
                {"lbg", node.mpc_setting_outputs_.args["lbg"]},
                {"ubg", node.mpc_setting_outputs_.args["ubg"]},
                {"p", node.mpc_setting_outputs_.args["p"]}
            };

            // Execute the solver
            DMDict sol;
            try {
                sol = node.mpc_setting_outputs_.solver(args);
                std::cout << "Solver executed successfully." << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Solver error: " << e.what() << std::endl;
                return;
            }

            DM u_portion = sol["x"](Slice(node.mpc_setting_outputs_.n_states * (node.initial_settings_.N + 1), Slice().stop));
            node.mpc_setting_outputs_.u = reshape(u_portion.T(), node.mpc_setting_outputs_.n_controls, node.initial_settings_.N).T();

            std::cout << "node.mpc_setting_outputs_.state_target" << node.mpc_setting_outputs_.state_target << std::endl;

            shiftTimestep(node);

            node.mpc_setting_outputs_.u0 = vertcat(
                node.mpc_setting_outputs_.u(Slice(1, node.mpc_setting_outputs_.u.rows()), Slice()), 
                node.mpc_setting_outputs_.u(Slice(-1), Slice())
            );

            // std::cout <<"u0 output"<< node.mpc_setting_outputs_.u0<<std::endl;

            DM sliced_sol_x = sol["x"](Slice(0, node.mpc_setting_outputs_.n_states * (node.initial_settings_.N + 1))).T();
            DM reshaped_X0 = reshape(sliced_sol_x, node.mpc_setting_outputs_.n_states, node.initial_settings_.N + 1);
            node.mpc_setting_outputs_.X0 = reshaped_X0.T();

            // Extract all rows except the first one
            DM X0_shifted_up = node.mpc_setting_outputs_.X0(Slice(1, node.mpc_setting_outputs_.X0.rows()), Slice());

            // Extract the last row and reshape it to 1 row by N columns
            DM last_row_reshaped = reshape(node.mpc_setting_outputs_.X0(node.mpc_setting_outputs_.X0.rows()-1, Slice()), 1, node.mpc_setting_outputs_.X0.columns());

            // Vertically concatenate the shifted matrix with the reshaped last row
            node.mpc_setting_outputs_.X0 = vertcat(X0_shifted_up, last_row_reshaped);

            // std::cout << "node.mpc_setting_outputs_.X0"<< node.mpc_setting_outputs_.X0 << std::endl;

            carControlExecution(node);

            
        } catch (const std::exception& e) {
            std::cerr << "Solver error: " << e.what() << std::endl;
            return;
        }
    }

    void carControlExecution(MpcNode& node) {

        if (node.car_behaviour_state_ == "keep_lane")
        {
            // Access the value at (0, 1) in the matrix and convert it to a double
            double steerAngleValue = static_cast<double>(node.mpc_setting_outputs_.u(0, 1));
            node.mpc_setting_outputs_.steerAngle = (180.0 / M_PI) * steerAngleValue;
            std::cout << "steerAngleValue: " << steerAngleValue << std::endl;
            // Access the value at (0, 0) in the matrix and convert it to a double
            double steerLateralValue = static_cast<double>(node.mpc_setting_outputs_.u(0, 0));
            node.mpc_setting_outputs_.steerLateral = steerLateralValue;
            carControlPublisher(node);
        }

        // Assuming state_target and state_init are DM (CasADi Dense Matrix) objects
        DM state_target_slice = node.mpc_setting_outputs_.state_target(Slice(0, 2));
        DM state_init_slice = node.mpc_setting_outputs_.state_init(Slice(0, 2));

        double distance = static_cast<double>(norm_2(state_target_slice - state_init_slice)(0));

        if (distance < node.initial_settings_.goal_checker) {

            if(node.djikstra_outputs_.pathGoalsYawDegree.size() > 0) {
                
                size_t last_path_index;
                int closest_node_id = calculateClosestNodeId(
                                            node.djikstra_outputs_.pathGoalsYawDegree, 
                                            node.localisation_data_.x, node.localisation_data_.y, 
                                            last_path_index);
                
                int closest_node_id_original = calculateClosestNodeIdOriginal(node.djikstra_outputs_.pathGoalsYawDegreeOriginal, 
                                                                        node.localisation_data_.x, node.localisation_data_.y);

                std::cout << "\n\n\n\n\nclosest_node_id: " << closest_node_id << std::endl;
                std::cout << "closest_node_id_original: " << closest_node_id_original << "\n\n\n"<< std::endl;
            }   
        }

        std::cout<< "node.mpc_setting_outputs_.steerAngle" << node.mpc_setting_outputs_.steerAngle << std::endl;
        std::cout<< "node.mpc_setting_outputs_.steerLateral" << node.mpc_setting_outputs_.steerLateral << std::endl;

    }

    int calculateClosestNodeId(
        const std::vector<std::tuple<int, double, double, double>>& pathGoalsYawDegree,
        double position_x, double position_y,
        size_t& last_path_index)
    {
        if (pathGoalsYawDegree.empty()) {
            throw std::runtime_error("pathGoalsYawDegree is empty");
        }

        std::vector<double> nodes_x;
        std::vector<double> nodes_y;

        for (const auto& node : pathGoalsYawDegree) {
            nodes_x.push_back(std::get<1>(node));
            nodes_y.push_back(std::get<2>(node));
        }

        std::vector<double> distances;
        for (size_t i = 0; i < nodes_x.size(); ++i) {
            double distance = std::sqrt(std::pow(nodes_x[i] - position_x, 2) + std::pow(nodes_y[i] - position_y, 2));
            distances.push_back(distance);
        }

        auto min_it = std::min_element(distances.begin(), distances.end());
        last_path_index = std::distance(distances.begin(), min_it);
        return std::get<0>(pathGoalsYawDegree[last_path_index]);
    }
    
    int calculateClosestNodeIdOriginal(
        const std::vector<std::tuple<int, double, double, double>>& pathGoalsYawDegreeOriginal,
        double position_x, double position_y)
    {
        if (pathGoalsYawDegreeOriginal.empty()) {
            throw std::runtime_error("pathGoalsYawDegreeOriginal is empty");
        }

        std::vector<double> nodes_x_original;
        std::vector<double> nodes_y_original;

        for (const auto& node : pathGoalsYawDegreeOriginal) {
            nodes_x_original.push_back(std::get<1>(node));
            nodes_y_original.push_back(std::get<2>(node));
        }

        std::vector<double> distances_original;
        for (size_t i = 0; i < nodes_x_original.size(); ++i) {
            double distance_original = std::sqrt(std::pow(nodes_x_original[i] - position_x, 2) + std::pow(nodes_y_original[i] - position_y, 2));
            distances_original.push_back(distance_original);
        }

        auto min_it_original = std::min_element(distances_original.begin(), distances_original.end());
        size_t index_original = std::distance(distances_original.begin(), min_it_original);
        return std::get<0>(pathGoalsYawDegreeOriginal[index_original]);
    }



    void shiftTimestep(MpcNode& node) {
        // Initialize state_init with the current position and orientation
        node.mpc_setting_outputs_.state_init = DM(std::vector<double>{
            node.localisation_data_.x,
            node.localisation_data_.y,
            node.localisation_data_.yaw
        });

        // Prepare the inputs for the CasADi function
        std::vector<DM> f_inputs = {
            node.mpc_setting_outputs_.state_init, 
            node.mpc_setting_outputs_.u(Slice(0), Slice()).T()
        };

        // Call the function and compute next_state (assuming f returns a vector of DM)
        std::vector<DM> f_outputs = node.mpc_setting_outputs_.f(f_inputs);
        DM f_value = f_outputs[0];

        // Update the next state
        node.mpc_setting_outputs_.next_state = node.mpc_setting_outputs_.state_init + 
                                            (node.initial_settings_.step_horizon * f_value);
        
    }

    void carControlPublisher(MpcNode& node) {
        // Create JSON object for car_steer_angle_data
        json car_steer_angle_data = {
            {"action", "2"},
            {"steerAngle", node.mpc_setting_outputs_.steerAngle}
        };
        std::string car_steer_angle_data_to_JSON = car_steer_angle_data.dump();

        // Create a std_msgs::String message for car_steer_angle_data
        std_msgs::String car_steer_angle_msg;
        car_steer_angle_msg.data = car_steer_angle_data_to_JSON;

        // Publish carData
        node.carControl_pub_.publish(car_steer_angle_msg);

        // Create JSON object for car_steer_lateral_data
        json car_steer_lateral_data = {
            {"action", "1"},
            {"speed", node.mpc_setting_outputs_.steerLateral}
        };
        std::string car_steer_lateral_data_to_JSON = car_steer_lateral_data.dump();

        // Create a std_msgs::String message for car_steer_lateral_data
        std_msgs::String car_steer_lateral_msg;
        car_steer_lateral_msg.data = car_steer_lateral_data_to_JSON;

        // Publish car2Data
        node.carControl_pub_.publish(car_steer_lateral_msg);
    }

};