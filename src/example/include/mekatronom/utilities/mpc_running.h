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
        ROS_INFO_ONCE("MpcRunning constructor");

        node.mpc_setting_outputs_.args["p"] = vertcat(
            node.mpc_setting_outputs_.state_init,
            node.mpc_setting_outputs_.state_target
        );
        node.mpc_setting_outputs_.args["x0"] = vertcat(
            reshape(node.mpc_setting_outputs_.X0.T(), node.mpc_setting_outputs_.n_states * (node.initial_settings_.N + 1), 1),
            reshape(node.mpc_setting_outputs_.u0.T(), node.mpc_setting_outputs_.n_controls * node.initial_settings_.N, 1)
        );

        std::cout << "Solver setup: " << node.mpc_setting_outputs_.solver << std::endl;
        try {
            std::cout << "x0 size: " << node.mpc_setting_outputs_.args["x0"].size1() 
                    << " x " << node.mpc_setting_outputs_.args["x0"].size2() << std::endl;
            
            std::cout << "lbx size: " << node.mpc_setting_outputs_.args["lbx"].size1() 
                    << " x " << node.mpc_setting_outputs_.args["lbx"].size2() << std::endl;

            std::cout << "ubx size: " << node.mpc_setting_outputs_.args["ubx"].size1() 
                    << " x " << node.mpc_setting_outputs_.args["ubx"].size2() << std::endl;

            std::cout << "lbg size: " << node.mpc_setting_outputs_.args["lbg"].size1() 
                    << " x " << node.mpc_setting_outputs_.args["lbg"].size2() << std::endl;

            std::cout << "ubg size: " << node.mpc_setting_outputs_.args["ubg"].size1() 
                    << " x " << node.mpc_setting_outputs_.args["ubg"].size2() << std::endl;

            std::cout << "p size: " << node.mpc_setting_outputs_.args["p"].size1() 
                    << " x " << node.mpc_setting_outputs_.args["p"].size2() << std::endl;

            // Debug values
            std::cout << "x0 values: " << node.mpc_setting_outputs_.args["x0"] << std::endl;
            std::cout << "lbx values: " << node.mpc_setting_outputs_.args["lbx"] << std::endl;
            std::cout << "ubx values: " << node.mpc_setting_outputs_.args["ubx"] << std::endl;
            std::cout << "lbg values: " << node.mpc_setting_outputs_.args["lbg"] << std::endl;
            std::cout << "ubg values: " << node.mpc_setting_outputs_.args["ubg"] << std::endl;
            std::cout << "p values: " << node.mpc_setting_outputs_.args["p"] << std::endl;

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
            std::cout << "Solver setup: " << node.mpc_setting_outputs_.solver << std::endl;
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

            DM sliced_sol_x = sol["x"](Slice(0, node.mpc_setting_outputs_.n_states * (node.initial_settings_.N + 1))).T();
            DM reshaped_X0 = reshape(sliced_sol_x, node.mpc_setting_outputs_.n_states, node.initial_settings_.N + 1);
            node.mpc_setting_outputs_.X0 = reshaped_X0.T();

            // Extract all rows except the first one
            DM X0_shifted_up = node.mpc_setting_outputs_.X0(Slice(1, node.mpc_setting_outputs_.X0.rows()), Slice());

            // Extract the last row and reshape it to 1 row by N columns
            DM last_row_reshaped = reshape(node.mpc_setting_outputs_.X0(node.mpc_setting_outputs_.X0.rows()-1, Slice()), 1, node.mpc_setting_outputs_.X0.columns());

            // Vertically concatenate the shifted matrix with the reshaped last row
            node.mpc_setting_outputs_.X0 = vertcat(X0_shifted_up, last_row_reshaped);

            std::cout << "node.mpc_setting_outputs_.X0"<< node.mpc_setting_outputs_.X0 << std::endl;

            if (node.car_behaviour_state_ == "keep_lane")
            {
                std::cout <<"test"<<std::endl;
            }


            /*
            * This part just for holding all of the data's all control outputs and states.
            */
            // for (int i = 0; i < node.initial_settings_.N; ++i) {
            //     // Update cat_states
            //     node.mpc_setting_outputs_.cat_states = horzcat(node.mpc_setting_outputs_.cat_states, node.mpc_setting_outputs_.X0);  // This stacks X0 horizontally, but for "layers", you'd store separately

            //     // Update cat_controls
            //     DM new_control = node.mpc_setting_outputs_.u0(0, Slice());  // Get the first row of u0
            //     node.mpc_setting_outputs_.cat_controls = vertcat(node.mpc_setting_outputs_.cat_controls, new_control);  // Append the new control to cat_controls
            // }

            
        } catch (const std::exception& e) {
            std::cerr << "Solver error: " << e.what() << std::endl;
            return;
        }

    }

};