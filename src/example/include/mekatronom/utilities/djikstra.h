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

// enable this for verbose debug information
// #define DEBUG

class processAndPublishPath  {
public:
    processAndPublishPath(std::string& graphml_file_path_,std::string source_node_, std::string target_node_, MpcNode& mpc_node)
    {
        
        clock_t stx = clock();
        pugi::xml_document doc;
        pugi::xml_parse_result result = doc.load_file(graphml_file_path_.c_str());  
        ROS_INFO("RESULT: %s", result.description());

        if (!result) {
            ROS_ERROR("Error loading XML file: %s", result.description());
            return;
        }
        pugi::xml_node root = doc.child("graphml");  
        if (!root) {
            ROS_ERROR("Error: no root node found in the XML file.");
            return;
        }
        pugi::xml_node graph = root.child("graph");
        if (!graph) {
            ROS_ERROR("Error: no graph node found in the XML file.");
            return;
        }

        mpc_node.nodes_data_ = extract_nodes_data(graph);

        for (const auto& node : mpc_node.nodes_data_ ) {
            const auto& node_id = std::get<0>(node);
            if (std::find(mpc_node.initial_settings_.parking_nodes_id.begin(), 
                mpc_node.initial_settings_.parking_nodes_id.end(), node_id) != mpc_node.initial_settings_.parking_nodes_id.end()) {
                double x = std::get<1>(node);
                double y = std::get<2>(node);
                mpc_node.djikstra_outputs_.obstacle_node_positions[node_id] = std::make_pair(x, y);
            }
        }

        mpc_node.edges_data_ = extract_edges_data(graph);
        
        mpc_node.djikstra_outputs_.pass_through = false;
        for (const auto& edge : mpc_node.edges_data_) {
            for (const auto& obs : mpc_node.initial_settings_.excluded_nodes) {
                if (std::get<2>(edge)) {
                    mpc_node.djikstra_outputs_.pass_through = true;
                    break;
                }
            }
            if (mpc_node.djikstra_outputs_.pass_through) break;
        }

        if (!mpc_node.djikstra_outputs_.pass_through) {
            mpc_node.djikstra_outputs_.pass_through = false;
        }

#ifdef DEBUG
        std::cout << "\n\n\nNext section is mpc_node.nodes_data_:\n ";
        for (const auto& node : mpc_node.nodes_data_) {
            std::cout << std::get<0>(node) << ": (" << std::get<1>(node) << ", " << std::get<2>(node) << ") ";
        }
        std::cout << std::endl;

        std::cout << "\n\n\next section is nmpc_node.edges_data_: \n";
        for (const auto& edge : mpc_node.edges_data_) {
            std::cout << std::get<0>(edge) << " -> " << std::get<1>(edge) << " (" << (std::get<2>(edge) ? "true" : "false") << ") ";
        }
        std::cout << std::endl;
#endif
        std::map<std::string, MpcNode::NodeInfo> noded;
        std::map<std::string, std::vector<std::pair<std::string, double>>> edged;
        std::tie(noded, edged) = extract_graph(mpc_node.nodes_data_, mpc_node.edges_data_);
        
#ifdef DEBUG
        std::cout << "\n\n\nnoded size: " << noded.size() << " edged size: " << edged.size() << std::endl;

        std::cout << "\n\n\nOutputs of noded:" << std::endl;
        for (const auto& [key, value] : noded) {
            std::cout << key << ": ("
                    << "distance: " << value.distance << ", "
                    << "pass_through: " << std::boolalpha << value.pass_through << ", "
                    << "x: " << value.x << ", "
                    << "y: " << value.y << ", "
                    << "atalist: [";

            for (size_t i = 0; i < value.atalist.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << value.atalist[i];
            }
            std::cout << "])" << std::endl;
        }

        std::cout << "\n\n\nOutputs of edged:" << std::endl;
        for (const auto& [key, value] : edged) {
            std::cout << key << ": ";
            for (const auto& v : value) {
                std::cout << "(" << v.first << ", " << v.second << ") ";
            }
            std::cout << " ";
        }
        std::cout << std::endl;
#endif
        
        mpc_node.shortest_path_ = dijkstra(source_node_, target_node_, noded, edged, mpc_node.initial_settings_.excluded_nodes, mpc_node);
        
        mpc_node.pathOriginal_ = stformat(mpc_node.shortest_path_);            
        mpc_node.djikstra_outputs_.node_dict = noded;
        

        auto [new_node_data, stlist] = beizer(mpc_node.shortest_path_, noded, mpc_node);
        mpc_node.new_node_data_ = new_node_data;
        
        ROS_INFO("\n\nCalculated path is :", mpc_node.shortest_path_);

#ifdef DEBUG
        std::cout << "\n\n\nmpc_node.shortest_path_ "<< mpc_node.shortest_path_ << std::endl;

        std::cout << "\n\n\nnoded size: " << noded.size() << " edged size: " << edged.size() << std::endl;

        std::cout << "\n\n\nOutputs of noded:" << std::endl;
        for (const auto& [key, value] : noded) {
            std::cout << key << ": ("
                    << "distance: " << value.distance << ", "
                    << "pass_through: " << std::boolalpha << value.pass_through << ", "
                    << "x: " << value.x << ", "
                    << "y: " << value.y << ", "
                    << "atalist: [";

            for (size_t i = 0; i < value.atalist.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << value.atalist[i];
            }
            std::cout << "])" << std::endl;
        }

        std::cout << "\n\n\nOutputs of edged:" << std::endl;
        for (const auto& [key, value] : edged) {
            std::cout << key << ": ";
            for (const auto& v : value) {
                std::cout << "(" << v.first << ", " << v.second << ") ";
            }
            std::cout << " ";
        }
        std::cout << std::endl;


        std::cout << "\n\n\nNew node data:" << mpc_node.new_node_data_ << std::endl;
        std::cout << "\n\n\nShortest path:" << mpc_node.shortest_path_ << std::endl;
        std::cout << "\n\n\nPath original:" <<  mpc_node.pathOriginal_ << std::endl;
#endif

        for (const auto& edge : stlist) {
            if (std::get<2>(edge)) {
                mpc_node.djikstra_outputs_.SourceTargetNodes.emplace_back(std::get<0>(edge), std::get<1>(edge));
            }
        }

        for (const auto& edge : mpc_node.pathOriginal_) {
            if (std::get<2>(edge)) {
                mpc_node.djikstra_outputs_.SourceTargetNodesOriginal.emplace_back(std::get<0>(edge), std::get<1>(edge));
            }
        }
        
        std::vector<std::tuple<int, double, double>> path;
        std::vector<std::tuple<int, double, double>> path_original;

        for (const auto& [source_id, target_id] : mpc_node.djikstra_outputs_.SourceTargetNodesOriginal) {
            if (mpc_node.new_node_data_.find(source_id) != mpc_node.new_node_data_.end()) {
                const auto& source_coords = mpc_node.new_node_data_.at(source_id);
                path_original.emplace_back(std::stoi(source_id), source_coords.first, source_coords.second); 
            }
            if (mpc_node.new_node_data_.find(target_id) != mpc_node.new_node_data_.end()) {
                const auto& coords = mpc_node.new_node_data_.at(target_id);
                path_original.emplace_back(std::stoi(target_id), coords.first, coords.second);
            }
        }

        for (const auto& [source_id, target_id] : mpc_node.djikstra_outputs_.SourceTargetNodes) {
            try {
                if (mpc_node.new_node_data_.find(target_id) != mpc_node.new_node_data_.end()) {
                    const auto& coords = mpc_node.new_node_data_.at(target_id);
                    path.emplace_back(std::stoi(target_id), coords.first, coords.second); 
                } else {
                    ROS_ERROR("target_id %s not found in obs_dict_", target_id.c_str());
                }
            } catch (const std::invalid_argument& e) {
                ROS_ERROR("Invalid target_id: %s", target_id.c_str());
            } catch (const std::out_of_range& e) {
                ROS_ERROR("target_id out of range: %s", target_id.c_str());
            }
        }

        std::vector<double> angles, anglesOriginal;

        for (size_t i = 0; i < path_original.size() - 1; ++i) {
            try {
                double dx = std::get<1>(path_original[i + 1]) - std::get<1>(path_original[i]);
                double dy = std::get<2>(path_original[i + 1]) - std::get<2>(path_original[i]);

                double angle = std::atan2(dy, dx);
                anglesOriginal.push_back(angle);
            } catch (const std::invalid_argument& e) {
                ROS_ERROR("Invalid argument: %s at index %d", e.what(), i);
                continue;
            } catch (const std::out_of_range& e) {
                ROS_ERROR("Out of range: %s at index %d", e.what(), i);
                continue;
            }
        }

        for (size_t i = 0; i < path.size() - 1; ++i) {
            double dx = std::get<1>(path[i+1]) - std::get<1>(path[i]);
            double dy = std::get<2>(path[i+1]) - std::get<2>(path[i]);

            double angle = std::atan2(dy, dx);
            angles.push_back(angle);
        }

        if (!angles.empty()) {
            angles.push_back(angles.back());
        }

        if (!anglesOriginal.empty()) {
            anglesOriginal.push_back(anglesOriginal.back());
        }

        for (size_t i = 0; i < path.size(); ++i) {
            mpc_node.djikstra_outputs_.pathGoalsYawDegree.emplace_back(std::get<0>(path[i]), std::get<1>(path[i]), std::get<2>(path[i]), angles[i]);
        }

        for (size_t i = 0; i < path_original.size(); ++i) {
            mpc_node.djikstra_outputs_.pathGoalsYawDegreeOriginal.emplace_back(std::get<0>(path_original[i]), std::get<1>(path_original[i]), std::get<2>(path_original[i]), anglesOriginal[i]);
        }

#ifdef DEBUG
        std::cout << "\n\n\nOutputs of Path:" << std::endl;
        for (const auto& [id, x, y] : path) {
            std::cout << "Node ID: " << id << ", X: " << x << ", Y: " << y << std::endl;
        }

        std::cout << "\n\n\nOutputs of path_original:" << std::endl;
        for(const auto& [id, x, y] : path_original)
        {
            std::cout << "ID: " << id << ", X: " << x << ", Y: " << y << std::endl;
        }

        std::cout << "\n\n\nOutputs of angles:" << std::endl;
        for (const auto& angle : angles) {
            std::cout << "Angle: " << angle << std::endl;
        }

        std::cout << "\n\n\nOutputs of pathGoalsYawDegree_:" << std::endl;       
        for (const auto& [id, x, y, angle] : mpc_node.djikstra_outputs_.pathGoalsYawDegree) {
            std::cout << "Node ID1: " << id << ", X1: " << x << ", Y1: " << y << ", Angle1: " << angle << std::endl;
        }

        std::cout << "\n\n\nOutputs of pathGoalsYawDegreeOriginal_:" << std::endl;
        for (const auto& [id, x, y, angle] : mpc_node.djikstra_outputs_.pathGoalsYawDegreeOriginal) {
            std::cout << "Node ID2: " << id << ", X2: " << x << ", Y2: " << y << ", Angle2: " << angle << std::endl;
        }        

        clock_t etx = clock();
        double elapsed_timex = double(etx - stx) / CLOCKS_PER_SEC;
        std::cout << "\n\n\nPasted time for DJikstra calculation:" << elapsed_timex << "seconds" << std::endl;
#endif
        if (!mpc_node.pathGoalsYawDegreecalled_) {
            mpc_node.djikstra_outputs_.pathGoalsYawDegreeCopy = mpc_node.djikstra_outputs_.pathGoalsYawDegreeOriginal;
            mpc_node.djikstra_outputs_.SourceTargetNodesCopy = mpc_node.djikstra_outputs_.SourceTargetNodesOriginal;
            mpc_node.pathGoalsYawDegreecalled_ = true;
        }

    }


    std::vector<std::string> dijkstra(const std::string& source, const std::string& target,
                                            std::map<std::string, MpcNode::NodeInfo>& nodedict,
                                            std::map<std::string, std::vector<std::pair<std::string, double>>>& edgedict,
                                            const std::vector<std::string>& excluded_nodes,
                                            MpcNode& mpc_node) {

        std::cout << "____________________" << source << std::endl;
        std::cout << "____________________" << target << std::endl; 

        std::vector<std::string> nowaypoints_source;
        std::vector<std::string> nowaypoints_target;

        for (auto& ed : edgedict) {
            for (auto it = ed.second.begin(); it != ed.second.end(); ) {
                if (std::find(excluded_nodes.begin(), excluded_nodes.end(), it->first) != excluded_nodes.end()) {
                    nowaypoints_source.push_back(ed.first);
                    nowaypoints_target.push_back(it->first);
                    it = ed.second.erase(it);  
                } else {
                    ++it;
                }
            }
        }

        std::unordered_map<std::string, double> unvisited;
        std::unordered_map<std::string, double> visited;
        std::unordered_map<std::string, std::vector<std::string>> paths;

        for (const auto& edge : edgedict) {
            unvisited[edge.first] = std::numeric_limits<double>::infinity();
        }
        unvisited[source] = 0;
        paths[source] = {source};  

        while (!unvisited.empty()) {
            // Find the node with the minimum distance
            auto minNode = *std::min_element(unvisited.begin(), unvisited.end(),
                                            [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });

            visited[minNode.first] = minNode.second;

            // Update distances to neighbors
            for (const auto& neighbor : edgedict[minNode.first]) {
                if (visited.find(neighbor.first) != visited.end()) continue;

                double newDistance = visited[minNode.first] + neighbor.second; 

                if (newDistance < unvisited[neighbor.first]) {
                    unvisited[neighbor.first] = newDistance;
                    paths[neighbor.first] = paths[minNode.first];  
                    paths[neighbor.first].push_back(neighbor.first); 

                    nodedict[neighbor.first].distance = newDistance;
                    nodedict[neighbor.first].atalist = paths[neighbor.first];
                }
            }

            unvisited.erase(minNode.first);  
        }

        std::vector<std::string> path = nodedict[target].atalist;

        if (std::find(path.begin(), path.end(), source) != path.end() &&
            std::find(path.begin(), path.end(), target) != path.end()) {

            bool traffic_light_state = false;
            bool pedesterian = false;
            bool is_past_path_is_shortest = false;

            
            if (traffic_light_state || pedesterian || is_past_path_is_shortest) {

                std::string bagendxx;
                for (const auto& tempstopxx : excluded_nodes) {
                    if (std::find(mpc_node.expath_.begin(), mpc_node.expath_.end(), tempstopxx) != mpc_node.expath_.end()) {
                        bagendxx = tempstopxx;
                        break;
                    }
                }

                auto indexnoEXxx = std::find(mpc_node.expath_.begin(), mpc_node.expath_.end(), bagendxx) - mpc_node.expath_.begin();

                std::string befbagendxx;
                if (mpc_node.expath_.size() > 2) {
                    befbagendxx = mpc_node.expath_[indexnoEXxx - 1];
                } else {
                    befbagendxx = mpc_node.expath_[indexnoEXxx];
                }

                nodedict[befbagendxx].atalist.push_back(befbagendxx);
                return nodedict[befbagendxx].atalist;
            } else {
                auto yolvar = true;
                mpc_node.expath_ = nodedict[target].atalist;
                return nodedict[target].atalist;
            }
        } else {
            std::string bagend;
            for (const auto& tempstopx : excluded_nodes) {
                if (std::find(mpc_node.expath_.begin(), mpc_node.expath_.end(), tempstopx) != mpc_node.expath_.end()) {
                    bagend = tempstopx;
                    break;
                }
            }
            auto indexnoEX = std::find(mpc_node.expath_.begin(), mpc_node.expath_.end(), bagend) - mpc_node.expath_.begin();
            std::string befbagend;
            if (mpc_node.expath_.size() > 2) {
                befbagend = mpc_node.expath_[indexnoEX - 2];
            } else {
                befbagend = mpc_node.expath_[indexnoEX];
            }

            nodedict[befbagend].atalist.push_back(befbagend);
            return nodedict[befbagend].atalist;
        }

        return nodedict[target].atalist;
    }

    std::vector<std::string> convertPathToString(const std::vector<int>& int_path) {
        std::vector<std::string> string_path;
        for (int node : int_path) {
            string_path.push_back(std::to_string(node));
        }
        return string_path;
    }

    std::pair<std::map<std::string, std::pair<double, double>>, std::vector<std::tuple<std::string, std::string, bool>>>
    beizer(const std::vector<std::string>& path, std::map<std::string, MpcNode::NodeInfo>& node_d, MpcNode& mpc_node) {
        
        std::map<std::string, std::pair<double, double>> new_node_data;
        for (const auto& [idkey, node] : node_d) {
            new_node_data[idkey] = std::make_pair(node.x, node.y);
        }

        int new_point_counter = 600;
        std::vector<std::string> new_path = path;
        size_t path_length = path.size();
        int ppbuk = 0;

        for (size_t f = 0; f < path_length - 2; ++f) {
            double angel_rad1, angel_rad2;

            if (node_d[path[f]].x == node_d[path[f + 1]].x) {
                angel_rad1 = 1.57;
            } else {
                angel_rad1 = std::atan((node_d[path[f]].y - node_d[path[f + 1]].y) / (node_d[path[f]].x - node_d[path[f + 1]].x));
            }
            double angel_deg1 = angel_rad1 * 57.3;

            if (node_d[path[f + 1]].x == node_d[path[f + 2]].x) {
                angel_rad2 = 1.57;
            } else {
                angel_rad2 = std::atan((node_d[path[f + 1]].y - node_d[path[f + 2]].y) / (node_d[path[f + 1]].x - node_d[path[f + 2]].x));
            }
            double angel_deg2 = angel_rad2 * 57.3;

            double b_andgel = std::abs(angel_deg1 - angel_deg2);
            int numout = 1;

            if (b_andgel > 55 && b_andgel < 110) {
                ppbuk = f;

                std::vector<std::string> temp_new_nodelist;

                if (true) { 
                    int numPts = 2;
                    numout = 2;

                    std::vector<std::pair<double, double>> controlPts = {
                        {node_d[path[f]].x, node_d[path[f]].y},
                        {node_d[path[f + 1]].x, node_d[path[f + 1]].y},
                        {node_d[path[f + 2]].x, node_d[path[f + 2]].y}
                    };
                    
                    std::vector<double> t(numPts + 1);
                    for (int i = 0; i <= numPts; ++i) {
                        t[i] = i * 1.0 / numPts;
                    }

                    std::vector<double> B_x(numPts + 1), B_y(numPts + 1);
                    for (int i = 0; i <= numPts; ++i) {
                        B_x[i] = (1 - t[i]) * ((1 - t[i]) * controlPts[0].first + t[i] * controlPts[1].first) + t[i] * ((1 - t[i]) * controlPts[1].first + t[i] * controlPts[2].first);
                        B_y[i] = (1 - t[i]) * ((1 - t[i]) * controlPts[0].second + t[i] * controlPts[1].second) + t[i] * ((1 - t[i]) * controlPts[1].second + t[i] * controlPts[2].second);
                    }

                    for (int new_p = 1; new_p < numPts; ++new_p) {
                        new_point_counter++;
                        std::string new_point_str = std::to_string(new_point_counter);

                        new_node_data[new_point_str] = std::make_pair(B_x[new_p], B_y[new_p]);
                        temp_new_nodelist.push_back(new_point_str);
                    }
                    new_path.erase(new_path.begin() + f + 1);
                }

                size_t temp_index = f + 1;
                for (const auto& insrt : temp_new_nodelist) {
                    new_path.insert(new_path.begin() + temp_index, insrt);
                    temp_index++;
                }
            }
        }

        std::vector<std::tuple<std::string, std::string, bool>> source_target;
        path_length = new_path.size();
        for (size_t n_edge = 0; n_edge < path_length - 1; ++n_edge) {
            source_target.emplace_back(new_path[n_edge], new_path[n_edge + 1], true);
        }

        return {new_node_data, source_target};
    }

    std::vector<std::tuple<std::string, std::string, bool>> stformat(const std::vector<std::string>& new_path) {
        std::vector<std::tuple<std::string, std::string, bool>> source_target;
        size_t pathlen = new_path.size();

        for (size_t n_edge = 0; n_edge < pathlen - 1; ++n_edge) {
            source_target.emplace_back(new_path[n_edge], new_path[n_edge + 1], true);
        }

        return source_target;
    }

    std::vector<std::tuple<std::string, double, double>> extract_nodes_data(pugi::xml_node root) {
        std::vector<std::tuple<std::string, double, double>> nodes_data;

        // Define the namespace URI
        const char* ns = "http://graphml.graphdrawing.org/xmlns";

        std::vector<std::string>  no_way_points;
        std::vector<std::string>  no_way_points_second;

        for (pugi::xml_node node : root.children("node")) {
            std::string node_id = node.attribute("id").value();
            double d0 = 0.0;
            double d1 = 0.0;
            bool d0_found = false, d1_found = false;

            for (pugi::xml_node data : node.children()) {
                std::string key = data.attribute("key").value();
                if (key == "d0") {
                    d0 = data.text().as_double();
                    d0_found = true;
                } else if (key == "d1") {
                    d1 = data.text().as_double();
                    d1_found = true;
                }
            }

            if (d0_found && d1_found && !node_id.empty()) {
                nodes_data.emplace_back(node_id, d0, d1);
            } else {
                ROS_ERROR("Error: Incomplete or missing data for node id: %s", node_id.c_str());
            }
        }

        return nodes_data;
    }

    std::vector<std::tuple<std::string, std::string, bool>> extract_edges_data(pugi::xml_node root) {
        std::vector<std::tuple<std::string, std::string, bool>> edges_data;

        for (pugi::xml_node edge : root.children("edge")) {
            std::string source_id = edge.attribute("source").value();
            std::string target_id = edge.attribute("target").value();
            pugi::xml_node data_d2 = edge.find_child_by_attribute("data", "key", "d2");
            bool d2_value = (data_d2 && (std::string(data_d2.child_value()) == "True"));

            if (!source_id.empty() && !target_id.empty()) {
                edges_data.emplace_back(source_id, target_id, d2_value);
            } else {
                ROS_ERROR("Error: Incomplete edge data source: %s target: %s", source_id.c_str(), target_id.c_str());
            }
        }
        return edges_data;
    }
    
    std::pair<std::map<std::string, MpcNode::NodeInfo>, std::map<std::string, std::vector<std::pair<std::string, double>>>>
    extract_graph(const std::vector<std::tuple<std::string, double, double>>& nodes_data_,
                  const std::vector<std::tuple<std::string, std::string, bool>>& edges_data_) {

        const double inf = std::numeric_limits<double>::infinity();

        std::map<std::string, MpcNode::NodeInfo> nodedict;
        std::map<std::string, std::vector<std::pair<std::string, double>>> edgedict;

        // Fill nodedict
        for (const auto& node : nodes_data_) {
            bool pass_through_current_node = false;
            for (const auto& delete_node : edges_data_) {
                if (std::get<0>(delete_node) == std::get<0>(node)) {
                    pass_through_current_node = !std::get<1>(delete_node).empty();
                    break;
                }
            }
            nodedict[std::get<0>(node)] = {inf, {}, pass_through_current_node, std::get<1>(node), std::get<2>(node)};
        }

        // Fill edgedict
        for (const auto& edge : edges_data_) {
            std::string source = std::get<0>(edge);
            std::string target = std::get<1>(edge);
            bool d2_value = std::get<2>(edge);

            if (edgedict.find(source) != edgedict.end()) {
                edgedict[source].emplace_back(target, d2_value);
            } else {
                edgedict[source] = {{target, d2_value}};
            }
        }

        return {nodedict, edgedict};
    }

};

