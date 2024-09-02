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

// Scenario names as constants
const std::string SCENARIO_PARK = "park";
const std::string SCENARIO_CROSSWALK = "crosswalk";
const std::string SCENARIO_MOTORWAY = "motorway";
const std::string SCENARIO_INITIAL = "initial_settings";

class MpcStartSetting
{
public:
    MpcStartSetting(const std::string& scenerio_name_, MpcNode& node)
    {
        ROS_INFO("scenerio_name: %s", scenerio_name_.c_str());
        setlocale(LC_NUMERIC, "C"); 

        /*
        * Try function is bit cheating. Those parameters tuned in real life scenario. 
        * So if anyone want to tune it he can change the values in the settings.
        * This values following the order:
        * Qx,Qy,Qtheta,R1,R2,step_horizon,N,rob_diam,wheel_radius,L_value,Ly,v_max,v_min,omega_max,omega_min
        */
        try {
            
            MpcNode::Settings initial_settings{
                1.0, 1.0, 1.0, 1.0, 1.8, 0.2, 12, 0.354, 1.0, 0.4, 0.04, 0.33,-0.1, M_PI / 7.5, -M_PI / 7.5
            };
            MpcNode::Settings park_scenerio_settings{
                1.0, 4.0, 0.05, 0.01, 0.02, 0.1, 6, 0.354, 1., 0.3, 0.04, 0.2,-0.25, M_PI / 7.5, -M_PI / 7.5
            };
            MpcNode::Settings crosswalk_scenerio_settings{
                1.0, 1.0, 1., 1., 1.8, 0.1, 12, 0.354, 1., 0.3, 0.04, 0.13,-0.2, M_PI / 7.5, -M_PI / 7.5
            };
            MpcNode::Settings motorway_scenerio_settings{
                1.0, 1.0, 2., 2., 2.8, 0.2, 4, 0.354, 1., 0.5, 0.04, 0.5,-0.1, M_PI / 7.5, -M_PI / 7.5
            };
            
            
            if (scenerio_name_ == SCENARIO_PARK) { 
                ROS_INFO("The scenario is park.");
                node.initial_settings_ = park_scenerio_settings;
            } else if (scenerio_name_ == SCENARIO_CROSSWALK) {
                ROS_INFO("The scenario is crosswalk.");
                node.initial_settings_ = crosswalk_scenerio_settings;
            } else if (scenerio_name_ == SCENARIO_MOTORWAY) {
                ROS_INFO("The scenario is motorway.");
                node.initial_settings_ = motorway_scenerio_settings;
            } else if (scenerio_name_ == SCENARIO_INITIAL) {
                ROS_INFO("The scenario is initial settings.");
                node.initial_settings_ = initial_settings;
            } else {
                ROS_INFO("The scenario is not recognized. The default settings are set.");
            
            }

        } catch (const std::exception& e) {
            ROS_FATAL("Exception caught during setting the scenario: %s", e.what());
            return;
        }


        node.mpc_setting_outputs_.state_init = DM::vertcat({node.localisation_data_.x, node.localisation_data_.y, node.localisation_data_.yaw});

        std::vector<double> nodes_x;
        std::vector<double> nodes_y;

        for (const auto& node : node.djikstra_outputs_.pathGoalsYawDegreeCopy){
            nodes_x.push_back(std::get<1>(node));
            nodes_y.push_back(std::get<2>(node));
        }

        std::vector<double> distances;
        for (size_t i =0;i<nodes_x.size();i++){
            distances.push_back(sqrt(pow(nodes_x[i]-node.localisation_data_.x,2)+pow(nodes_y[i]-node.localisation_data_.y,2)));
        }

        auto min_it = std::min_element(distances.begin(), distances.end());
        double min_distance = *min_it;
        int index = std::distance(distances.begin(), min_it);

        int closest_node_id = std::get<0>(node.djikstra_outputs_.pathGoalsYawDegreeCopy[index]);
        node.mpc_setting_outputs_.last_path_index = index;

        // Finding the next node
        std::pair<std::string, std::string> matching_pair;
        for (const auto& pair : node.djikstra_outputs_.SourceTargetNodesCopy) {
            if (pair.first == std::to_string(closest_node_id)) {
                matching_pair = pair;
                break;
            }
        }

        std::string next_node_id_str = matching_pair.second;
        int next_node_id = std::stoi(next_node_id_str);  // Convert string back to int

        std::tuple<int, double, double, double> matching_entry;
        bool matching_entry_found = false;
        for (const auto& entry : node.djikstra_outputs_.pathGoalsYawDegreeCopy) {
            if (std::get<0>(entry) == next_node_id) {
                matching_entry = entry;
                matching_entry_found = true;
                break;
            }
        }

        if (!matching_entry_found) {
            ROS_FATAL("Matching entry not found for next node id: %d", next_node_id);
            return;
        } else {
            double target_x = std::get<1>(matching_entry);
            double target_y = std::get<2>(matching_entry);

            double dx = target_x - node.localisation_data_.x;
            double dy = target_y - node.localisation_data_.y;
            double yaw = std::atan2(dy, dx);

            int goal_id = std::get<0>(matching_entry);

            node.mpc_setting_outputs_.state_init = DM({node.localisation_data_.x, node.localisation_data_.y, node.localisation_data_.yaw});
            node.mpc_setting_outputs_.state_target = DM({target_x, target_y, yaw});

            std::cout << "state_init: " << node.mpc_setting_outputs_.state_init << std::endl;
            std::cout << "state_target: " << node.mpc_setting_outputs_.state_target << std::endl;
        }


        /*
        * Initial settings for the MPC 
        */
        SX x = SX::sym("x");
        SX y = SX::sym("y");
        SX theta = SX::sym("theta");
        SX states = vertcat(x, y, theta);
        node.mpc_setting_outputs_.n_states = states.size1() * states.size2(); 

        SX v = SX::sym("v");
        SX omega = SX::sym("omega");
        SX controls = vertcat(v, omega);
        node.mpc_setting_outputs_.n_controls = controls.numel();

        SX X = SX::sym("X", node.mpc_setting_outputs_.n_states, (node.initial_settings_.N + 1));
        SX U = SX::sym("U", node.mpc_setting_outputs_.n_controls, node.initial_settings_.N);
        SX P = SX::sym("P", node.mpc_setting_outputs_.n_states + node.mpc_setting_outputs_.n_states);

        std::vector<SX> Q_elements;
        Q_elements.push_back(SX::diag(node.initial_settings_.Q_x)); 
        Q_elements.push_back(SX::diag(node.initial_settings_.Q_y));
        Q_elements.push_back(SX::diag(node.initial_settings_.Q_theta));
        SX Q = diagcat(Q_elements); 

        std::vector<SX> R_elements;
        R_elements.push_back(SX::diag(node.initial_settings_.R1));
        R_elements.push_back(SX::diag(node.initial_settings_.R2));
        SX R = diagcat(R_elements); 

       /*
       * Bicycle model
       */
        SX RHS = vertcat(
            v * cos(theta),
            v * sin(theta),
            v / node.initial_settings_.L_value * tan(omega)

        );

        node.mpc_setting_outputs_.f = Function("f", {states, controls}, {RHS});

        SX cost_fn = 0; 
        SX g = X(Slice(), 0) - P(Slice(0, node.mpc_setting_outputs_.n_states)); 

        SX st = X(Slice(), 0);

        /*
        * Runge-Kutta 4th order integration method
        */
        for (int k = 0; k < node.initial_settings_.N; ++k) {
            SX st = X(Slice(), k);
            SX con = U(Slice(), k);
            std::cout << "st: " << st << std::endl;
            std::cout << "con: " << con << std::endl;

            cost_fn += mtimes(mtimes((st - P(Slice(3, 6), 0)).T(), Q), (st - P(Slice(3, 6), 0))) + 
                    mtimes(mtimes(con.T(), R), con);

            SX st_next = X(Slice(), k+1);

            auto k1 = node.mpc_setting_outputs_.f(std::vector<SX>{st, con});
            auto k2 = node.mpc_setting_outputs_.f(std::vector<SX>{st + (node.initial_settings_.step_horizon / 2) * k1.at(0), con});
            auto k3 = node.mpc_setting_outputs_.f(std::vector<SX>{st + (node.initial_settings_.step_horizon / 2) * k2.at(0), con});
            auto k4 = node.mpc_setting_outputs_.f(std::vector<SX>{st + node.initial_settings_.step_horizon * k3.at(0), con});

            SX st_next_RK4 = st + (node.initial_settings_.step_horizon / 6) * (k1.at(0) + 2*k2.at(0) + 2*k3.at(0) + k4.at(0));
            // std::cout << "st_next: " << st_next << std::endl;
            // std::cout << "st_next_RK4: " << st_next_RK4 << std::endl;
            // std::cout << "test: " << std::endl;
            g = vertcat(g, st_next - st_next_RK4);
            // Verify that number of constraints matches expectations
            // std::cout << "Number of constraints in g: " << g.size1() << std::endl;
        }

        SX OPT_variables = vertcat(reshape(X, node.mpc_setting_outputs_.n_states * (node.initial_settings_.N + 1), 1), reshape(U, node.mpc_setting_outputs_.n_controls * node.initial_settings_.N, 1));
        // std::cout << "OPT_variables: " << OPT_variables << std::endl;
        // std::cout << "cost_fn: " << cost_fn << std::endl;
        // std::cout << "g: " << g << std::endl;
        // std::cout << "P: " << P << std::endl;
        SXDict nlp_prob = {
            {"f", cost_fn},
            {"x", OPT_variables},
            {"g", g},
            {"p", P}
        };
        
        // std::cout << "nlp_prob: " << nlp_prob << std::endl;
        Dict opts = {
            {"ipopt.max_iter", 100},
            {"ipopt.print_level", 0},
            {"ipopt.acceptable_tol", 1e-8},
            {"ipopt.acceptable_obj_change_tol", 1e-6},
            {"print_time", 0},
        };

        // std::cout << "opts: " << opts << std::endl;

        node.mpc_setting_outputs_.solver = nlpsol("solver", "ipopt", nlp_prob, opts);

        /*
        * This section is for the constraints of the states and controls
        * Currently supporting velocity and angular velocity constraints
        */
        DM lbx = DM::zeros(node.mpc_setting_outputs_.n_states*(node.initial_settings_.N+1) + node.mpc_setting_outputs_.n_controls*node.initial_settings_.N, 1);
        DM ubx = DM::zeros(node.mpc_setting_outputs_.n_states*(node.initial_settings_.N+1) + node.mpc_setting_outputs_.n_controls*node.initial_settings_.N, 1);
        DM lbg = DM::zeros(node.mpc_setting_outputs_.n_states*(node.initial_settings_.N+1), 1);
        DM ubg = DM::zeros(node.mpc_setting_outputs_.n_states*(node.initial_settings_.N+1), 1);

        // std::cout<< "node.mpc_setting_outputs_.solver: " << node.mpc_setting_outputs_.solver << std::endl;

        for (int i = 0; i < node.mpc_setting_outputs_.n_states * (node.initial_settings_.N + 1); i += node.mpc_setting_outputs_.n_states) {
            // Assigning lower bounds
            lbx(i) = -DM::inf();  // Ensure lbx is initialized correctly
            lbx(i + 1) = -DM::inf();
            lbx(i + 2) = -DM::inf();

            // Assigning upper bounds
            ubx(i) = DM::inf();  // Ensure ubx is initialized correctly
            ubx(i + 1) = DM::inf();
            ubx(i + 2) = DM::inf();
        }


        for (int i = node.mpc_setting_outputs_.n_states*(node.initial_settings_.N+1); i < node.mpc_setting_outputs_.n_states*(node.initial_settings_.N+1)+2*node.initial_settings_.N; i+=2) {
            lbx(i) = node.initial_settings_.v_min;
            ubx(i) = node.initial_settings_.v_max;
            lbx(i+1) = node.initial_settings_.omega_min;
            ubx(i+1) = node.initial_settings_.omega_max;

        }


        /*
        * u0 is [velocity, angular velocity]
        * x0 is [x, y, theta] 
        * Here is initial settings for the mpcRunning
        */
        node.mpc_setting_outputs_.u0 = DM::zeros(node.initial_settings_.N, 2); 
        node.mpc_setting_outputs_.X0 = DM::repmat(node.mpc_setting_outputs_.state_init 
                                                    , 1, node.initial_settings_.N+1).T(); 
        node.mpc_setting_outputs_.args["lbg"] = lbg;
        node.mpc_setting_outputs_.args["ubg"] = ubg;
        node.mpc_setting_outputs_.args["lbx"] = lbx;
        node.mpc_setting_outputs_.args["ubx"] = ubx;

        // std::cout << "dual solution (x) = " << res.at("lbg") << std::endl;
        node.mpc_setting_outputs_.cat_states = node.mpc_setting_outputs_.X0;
        node.mpc_setting_outputs_.cat_controls = DM::zeros(0,node.mpc_setting_outputs_.u0.columns());
        // Check number of constraints
        // std::cout << "Number of constraints in g: " << g.size1() << std::endl;
        // std::cout << "Expected number of constraints: " << lbg.size1() << std::endl;

    }
};