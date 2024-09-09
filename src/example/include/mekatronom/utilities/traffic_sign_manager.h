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

class TrafficSignManager{
public:
    TrafficSignManager(MpcNode& node)
    {
        std::cout << "TrafficSignManager constructor" << std::endl;

        node.checking_counter_++;

        if ((node.car_behaviour_state_ == "keep_lane") && 
            (node.initial_settings_.obstacles_array.size() > 0) && 
            (node.checking_counter_ > 50) &&
            (node.djikstra_outputs_.pass_through == 0))
            {
                std::cout << "Obstacle moved out of the way clearing the data" << std::endl;
                for (const auto& value : node.initial_settings_.obstacles_array) {
                    auto obstacle_id = std::find(node.initial_settings_.excluded_nodes.begin(), node.initial_settings_.excluded_nodes.end(), value); 
                    if (obstacle_id != node.initial_settings_.excluded_nodes.end()) {
                        node.initial_settings_.excluded_nodes.erase(obstacle_id);
                    }
                }
                node.initial_settings_.obstacles_array.clear();

                processAndPublishPath processAndPublishPath(node.graphml_file_path_, node.closest_node_id_original_, node.initial_settings_.target_node, node); 
            }     

        else
        {
            node.car_behaviour_state_ = "keep_lane";
            ROS_INFO("The car behaviour state is ", node.car_behaviour_state_);
        }

        //  TODO: For now other sections of parking motorway endofmotorway settings will be disabled.  It does need yolo implementation.
        //  TODO: Might be I can run it with directly python script. IDK yet.

    }
};

