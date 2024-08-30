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
                std::string closest_node_id = calculateClosestNodeId( node.djikstra_outputs_.pathGoalsYawDegree, 
                                                            node.localisation_data_.x, node.localisation_data_.y);
            }
            else {
                std::string closest_node_id = calculateClosestNodeId(node.djikstra_outputs_.pathGoalsYawDegreeCopy, 
                                                                node.localisation_data_.x, node.localisation_data_.y);
            }

            std::string closest_node_id_original = calculateClosestNodeId(node.djikstra_outputs_.pathGoalsYawDegreeOriginal, 
                                                                    node.localisation_data_.x, node.localisation_data_.y);
            processObstacles(node,closest_node_id_original);

        }

        std::cout<< "node.mpc_setting_outputs_.steerAngle" << node.mpc_setting_outputs_.steerAngle << std::endl;
        std::cout<< "node.mpc_setting_outputs_.steerLateral" << node.mpc_setting_outputs_.steerLateral << std::endl;

    }

    void processObstacles(MpcNode& node, const std::string& closest_node_id_original) {
        std::vector<std::string> sign_looking_band;
        std::string current_id_for_obstacles = calculateClosestNodeId(node.djikstra_outputs_.pathGoalsYawDegreeCopy, 
                                                                    node.localisation_data_.x, node.localisation_data_.y);
        for (int i = 1; i <= 5; ++i) {
            std::vector<std::pair<std::string, std::string>> matching_pairs_signs = findMatchingPairs(node.djikstra_outputs_.SourceTargetNodesCopy, current_id_for_obstacles);
            std::tuple<int, double, double, double> matching_entry = findMatchingEntry(node.djikstra_outputs_.pathGoalsYawDegreeCopy, current_id_for_obstacles);

            if (!matching_pairs_signs.empty()) {
                std::string next_id_signs = matching_pairs_signs[0].second;

                std::tuple<int, double, double, double>  matching_entry_second = findMatchingEntry(node.djikstra_outputs_.pathGoalsYawDegreeCopy, next_id_signs);

                sign_looking_band.push_back(current_id_for_obstacles);
                current_id_for_obstacles = next_id_signs;

                std::array<double, 2> target_position = {std::get<1>(matching_entry), std::get<2>(matching_entry)};
                double x_diff = std::abs(std::get<1>(matching_entry) - std::get<1>(matching_entry_second));
                double y_diff = std::abs(std::get<2>(matching_entry) - std::get<2>(matching_entry_second));

                double x_thereshold = 0.1;
                double y_thereshold = 0.1;

                findingObstacleNodes(node, closest_node_id_original, matching_entry, matching_entry_second, target_position, x_thereshold, y_thereshold);
            }
        }
    }

    void findingObstacleNodes(MpcNode& node, const std::string& closest_node_id_original, 
                            const std::tuple<int, double, double, double>& matching_entry, 
                            const std::tuple<int, double, double, double>& matching_entry_second, 
                            const std::array<double, 2>& target_position, 
                            double x_thereshold, double y_thereshold) 
    {
        for (size_t i = 0; i < node.center_x_.size(); ++i) {
            Eigen::Vector2d obstacleVec(node.center_x_[i], node.center_y_[i]);
            Eigen::Vector2d targetVec(target_position[0], target_position[1]);

            for (const auto& [node_id, coordinates] : node.djikstra_outputs_.obstacle_node_positions) {
                double x = coordinates.first;
                double y = coordinates.second;
                double distance = std::sqrt(std::pow(node.center_x_[i] - x, 2) + std::pow(node.center_y_[i] - y, 2));

                if (distance <= 0.4 && std::find(node.initial_settings_.parking_spot_is_full.begin(), 
                                                node.initial_settings_.parking_spot_is_full.end(), node_id) == node.initial_settings_.parking_spot_is_full.end()) {

                    node.initial_settings_.parking_spot_is_full.push_back(node_id);
                    node.initial_settings_.parking_nodes_id.erase(std::remove(node.initial_settings_.parking_nodes_id.begin(), 
                                                                                node.initial_settings_.parking_nodes_id.end(), node_id), 
                                                                node.initial_settings_.parking_nodes_id.end());
                }
            }

            if (std::find(node.initial_settings_.parking_nodes_id.begin(), 
                        node.initial_settings_.parking_nodes_id.end(), 
                        node.initial_settings_.target_node) != node.initial_settings_.parking_nodes_id.end()) 
            {
                node.initial_settings_.target_node = node.initial_settings_.parking_nodes_id[0];
                Djikstra djikstra(node.graphml_file_path_, closest_node_id_original, node.initial_settings_.target_node, node);
            }

            // Calculate distance to the target position
            double distance = (obstacleVec - targetVec).norm();

            // Use std::get<> to access tuple elements
            double x1 = std::get<1>(matching_entry);
            double y1 = std::get<2>(matching_entry);
            double x2 = std::get<1>(matching_entry_second);
            double y2 = std::get<2>(matching_entry_second);

            // Check if within thresholds
            bool is_within_x_threshold = std::min(x1, x2) - x_thereshold <= node.center_x_[i] &&
                                        node.center_x_[i] <= std::max(x1, x2) + x_thereshold;
            bool is_within_y_threshold = std::min(y1, y2) - y_thereshold <= node.center_y_[i] &&
                                        node.center_y_[i] <= std::max(y1, y2) + y_thereshold;

            int entry_id = std::get<0>(matching_entry);
            std::string entry_id_str = std::to_string(entry_id);

            checkingObstacleProximity(node, distance, is_within_x_threshold, is_within_y_threshold, entry_id_str, matching_entry, matching_entry_second, closest_node_id_original);
            updateAvailableParkingSpots(node);
            IsObstacleStillThere(node, obstacleVec);

        }
    }
    void checkingObstacleProximity(MpcNode& node, double distance, bool is_within_x_threshold, bool is_within_y_threshold, const std::string& entry_id_str, 
                                    const std::tuple<int, double, double, double>& matching_entry, 
                                    const std::tuple<int, double, double, double>& matching_entry_second, 
                                    const std::string& closest_node_id_original) 
    {
        if ((distance < 0.15 || (is_within_x_threshold && is_within_y_threshold)) && 
            (std::find(node.initial_settings_.dont_check_obstacles_this_nodes.begin(), 
                        node.initial_settings_.dont_check_obstacles_this_nodes.end(), 
                        entry_id_str) == node.initial_settings_.dont_check_obstacles_this_nodes.end())) 
        {
            std::cout << "Obstacle is close or within thresholds" << std::endl;

            std::vector<std::string> past_excluded_nodes = node.initial_settings_.excluded_nodes;

            // Convert tuple entries to strings
            std::string matching_entry_second_id_str = std::to_string(std::get<0>(matching_entry_second));

            // Check and update the obstacles array
            if (std::find(node.initial_settings_.obstacles_array.begin(),
                        node.initial_settings_.obstacles_array.end(), entry_id_str) == node.initial_settings_.obstacles_array.end()) 
            {
                node.initial_settings_.obstacles_array.push_back(entry_id_str);
            }

            // Check if the second matching entry should be added to obstacles array
            if (std::find(node.initial_settings_.excluded_nodes.begin(),
                        node.initial_settings_.excluded_nodes.end(), matching_entry_second_id_str) == node.initial_settings_.excluded_nodes.end()) 
            {
                node.initial_settings_.obstacles_array.push_back(matching_entry_second_id_str);
            }

            // Add obstacles to the excluded nodes if not already present
            for (const auto& obstacle_id : node.initial_settings_.obstacles_array) {
                // No need to convert obstacle_id if it's already a std::string
                if (std::find(node.initial_settings_.excluded_nodes.begin(),
                            node.initial_settings_.excluded_nodes.end(), obstacle_id) == node.initial_settings_.excluded_nodes.end()) 
                {
                    node.initial_settings_.excluded_nodes.push_back(obstacle_id);
                }
            }

            // Check if excluded nodes have changed and update if necessary
            if (node.initial_settings_.excluded_nodes != past_excluded_nodes) {
                Djikstra djikstra(node.graphml_file_path_, closest_node_id_original, node.initial_settings_.target_node, node); 
            }
            else {
                std::cout << "Found obstacle is already in the excluded nodes list" << std::endl;
            }

        }
    }

    std::vector<std::pair<std::string, std::string>> findMatchingPairs(const std::vector<std::pair<std::string, std::string>>& sourceTargetNodes, const std::string& current_id) {
        std::vector<std::pair<std::string, std::string>> matching_pairs;
        for (const auto& pair : sourceTargetNodes) {
            if (pair.first == current_id) {
                matching_pairs.push_back(pair);
            }
        }
        return matching_pairs;
    }

    std::tuple<int, double, double, double> findMatchingEntry(const std::vector<std::tuple<int, double, double, double>>& pathGoalsYawDegree, const std::string& current_id) {
        for (const auto& entry : pathGoalsYawDegree) {
            if (std::to_string(std::get<0>(entry)) == current_id) {
                return entry;
            }
        }
        throw std::runtime_error("Matching entry not found");
    }

    std::string calculateClosestNodeId(
        const std::vector<std::tuple<int, double, double, double>>& pathGoalsYawDegree,
        double position_x, double position_y)
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
        size_t last_path_index = std::distance(distances.begin(), min_it);
        return std::to_string(std::get<0>(pathGoalsYawDegree[last_path_index]));
    }

    void updateAvailableParkingSpots(MpcNode& node) {
        std::set_difference(node.initial_settings_.parking_nodes_id.begin(), node.initial_settings_.parking_nodes_id.end(),
                            node.initial_settings_.parking_spot_is_full.begin(), node.initial_settings_.parking_spot_is_full.end(),
                            std::back_inserter(node.initial_settings_.parkings_are_available));
    }

    void IsObstacleStillThere(MpcNode& node, const Eigen::Vector2d& obstacle_position) {
        for (const auto& obstacle_id : node.initial_settings_.obstacles_array) {
            auto matching_entry_obstacle_first = std::find_if(node.djikstra_outputs_.pathGoalsYawDegreeCopy.begin(), 
                                                            node.djikstra_outputs_.pathGoalsYawDegreeCopy.end(), 
                                                            [&obstacle_id](const std::tuple<int, double, double, double>& entry) {
                                                                return std::to_string(std::get<0>(entry)) == obstacle_id;
                                                            });
            auto matching_entry_id_second =std::find_if(node.djikstra_outputs_.SourceTargetNodesCopy.begin(), 
                                                        node.djikstra_outputs_.SourceTargetNodesCopy.end(), 
                                                        [&obstacle_id](const std::pair<std::string, std::string>& pair) {
                                                            return pair.first == obstacle_id;
                                                        });

            auto matching_entry_obstacle_second = std::find_if(node.djikstra_outputs_.pathGoalsYawDegreeCopy.begin(), 
                                                            node.djikstra_outputs_.pathGoalsYawDegreeCopy.end(), 
                                                            [&matching_entry_id_second](const std::tuple<int, double, double, double>& entry) {
                                                                return std::to_string(std::get<0>(entry)) == matching_entry_id_second->second;
                                                            });

            double x_thereshold = 0.15;
            double y_thereshold = 0.15;

            Eigen::Vector2d target_position_obstacle_first(std::get<1>(*matching_entry_obstacle_first), std::get<2>(*matching_entry_obstacle_first));               
            Eigen::Vector2d target_position_obstacle_second(std::get<1>(*matching_entry_obstacle_second), std::get<2>(*matching_entry_obstacle_second));

            bool is_within_x_threshold = std::min(target_position_obstacle_first[0], target_position_obstacle_second[0]) - x_thereshold <= node.center_x_[0] &&
                                        node.center_x_[0] <= std::max(target_position_obstacle_first[0], target_position_obstacle_second[0]) + x_thereshold;
                                        
            bool is_within_y_threshold = std::min(target_position_obstacle_first[1], target_position_obstacle_second[1]) - y_thereshold <= node.center_y_[0] &&
                                        node.center_y_[0] <= std::max(target_position_obstacle_first[1], target_position_obstacle_second[1]) + y_thereshold;

            double norm_first = (target_position_obstacle_first - obstacle_position).norm();
            double norm_second = (target_position_obstacle_second - obstacle_position).norm();

            if (norm_first < 0.16 || norm_second < 0.16 || (is_within_x_threshold && is_within_y_threshold)) {
                node.car_behaviour_state_ = "waiting the obstacle move away";
                node.checking_counter_ = 0;
            }
        }
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