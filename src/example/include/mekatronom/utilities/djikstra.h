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
#include "mekatronom/MpcNode.hpp"

class Djikstra  {
public:
    Djikstra(std::string& graphml_file_path_,std::string source_node_, std::string target_node_, MpcNode& mpc_node)
    {
        
        clock_t stx = clock();
        pugi::xml_document doc;
        pugi::xml_parse_result result = doc.load_file(graphml_file_path_.c_str());  // Load the XML file
        std::cout << "Result: " << result.description() << std::endl;

        if (!result) {
            std::cerr << "Error loading XML file: " << result.description() << std::endl;
            return;
        }
        pugi::xml_node root = doc.child("graphml");  // Adjust the root node name as per your XML structure
        if (!root) {
            std::cerr << "Error: no root node found in the XML file." << std::endl;
            return;
        }
        pugi::xml_node graph = root.child("graph");
        if (!graph) {
            std::cerr << "Error: no graph node found in the XML file." << std::endl;
            return;
        }
        mpc_node.nodes_data_ = extract_nodes_data(graph);

        for (const auto& node : mpc_node.nodes_data_ ) {
            const auto& node_id = std::get<0>(node);
            if (std::find(mpc_node.parking_nodes_id_.begin(), mpc_node.parking_nodes_id_.end(), node_id) != mpc_node.parking_nodes_id_.end()) {
                double x = std::get<1>(node);
                double y = std::get<2>(node);
                mpc_node.obstacle_node_positions_[node_id] = std::make_pair(x, y);
            }
        }

        mpc_node.edges_data_ = extract_edges_data(graph);
        
        auto flag_solla = false;
        for (const auto& edge : mpc_node.edges_data_) {
            for (const auto& obs : mpc_node.obs_dontuse_) {
                if (std::get<2>(edge)) {
                    flag_solla = true;
                    break;
                }
            }
            if (flag_solla) break;
        }

        if (!flag_solla) {
            flag_solla = false;
        }

        /*
        * For debugging purpose
        */
        std::cout << "mpc_node.nodes_data_: ";
        for (const auto& node : mpc_node.nodes_data_) {
            std::cout << std::get<0>(node) << ": (" << std::get<1>(node) << ", " << std::get<2>(node) << ") ";
        }
        std::cout << std::endl;

        std::cout << "mpc_node.edges_data_: ";
        for (const auto& edge : mpc_node.edges_data_) {
            std::cout << std::get<0>(edge) << " -> " << std::get<1>(edge) << " (" << (std::get<2>(edge) ? "true" : "false") << ") ";
        }
        std::cout << std::endl;
        /*
        * For debugging purpose
        */

        clock_t etx = clock();
        double elapsed_timex = double(etx - stx) / CLOCKS_PER_SEC;
        std::cout << "Execution time:" << elapsed_timex << " seconds##################-----------------####################" << std::endl;

        // if (flag_solla) {
        //     pugi::xml_document doc2;
        //     if (!doc2.load_file(graphml_file_path_.c_str())) {
        //         std::cerr << "Error loading second XML file!" << std::endl;
        //         return;
        //     }
        //     pugi::xml_node root2 = doc2.child("root2");  // Adjust the root node name as per your XML structure
        //     mpc_node.nodes_data_  = extract_nodes_data(root2);
        //     mpc_node.edges_data_ = extract_edges_data(root2);
        // }
        ROS_INFO_ONCE("Starting to extract_graph");
        auto [noded, edged] = extract_graph(mpc_node.nodes_data_, mpc_node.edges_data_);

        /*
        * For debugging purpose
        */
        std::cout << "noded size: " << noded.size() << " edged size: " << edged.size() << std::endl;
        std::cout << "noded: ";
        for (const auto& [key, value] : noded) {
            std::cout << key << ": (" << value.x << ", " << value.y << ") ";
        }
        std::cout << std::endl;
        std::cout << "edged: ";
        for (const auto& [key, value] : edged) {
            std::cout << key << ": ";
            for (const auto& v : value) {
                std::cout << "(" << v.first << ", " << v.second << ") ";
            }
            std::cout << " ";
        }
        std::cout << std::endl;
        /*
        * For debugging purpose
        */
        mpc_node.shortest_path_ = dijkstra(source_node_, target_node_, noded, edged, mpc_node.obs_dontuse_, mpc_node);
        std::cout << "Shortest path:" << mpc_node.shortest_path_ << std::endl;
        mpc_node.pathOriginal_ = stformat(mpc_node.shortest_path_);            
        std::cout << "Path original:" <<  mpc_node.pathOriginal_ << std::endl;

        // In the constructor, ensure you properly initialize new_node_data and source_target using structured bindings
        auto [new_node_data, stlist] = beizer(mpc_node.shortest_path_, noded, mpc_node);
        mpc_node.new_node_data_ = new_node_data;

        std::cout << "New node data:" << new_node_data << std::endl;
        std::cout << "ST list:" << stlist << std::endl;

        for (const auto& edge : stlist) {
            if (std::get<2>(edge)) {
                mpc_node.SourceTargetNodes.emplace_back(std::get<0>(edge), std::get<1>(edge));
            }
        }

        for (const auto& edge : mpc_node.pathOriginal_) {
            if (std::get<2>(edge)) {
                mpc_node.SourceTargetNodesOriginal.emplace_back(std::get<0>(edge), std::get<1>(edge));
            }
        }
        
        std::vector<std::tuple<int, double, double>>  path;
        std::vector<std::tuple<std::string, std::string, bool>> path_original;

        for (const auto& [source_id, target_id] : mpc_node.SourceTargetNodesOriginal) {
            if (mpc_node.new_node_data_.find(source_id) != mpc_node.new_node_data_.end()) {
                const auto& source_coords = mpc_node.new_node_data_.at(source_id);
                // Correcting insertion for pathOriginal_ which expects std::string, std::string, bool
                path_original.emplace_back(source_id, target_id, true); // Assuming you want a boolean flag as 'true'.
            }
            if (mpc_node.new_node_data_.find(target_id) != mpc_node.new_node_data_.end()) {
                const auto& coords = mpc_node.new_node_data_.at(target_id);
                // Correcting insertion for path_ which expects int, double, double
                mpc_node.path_.emplace_back(std::stoi(target_id), coords.first, coords.second); // Ensure target_id is an integer string.
                try {
                    int target_id_int = std::stoi(target_id);
                    mpc_node.path_.emplace_back(target_id_int, coords.first, coords.second);
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Invalid target_id: " << target_id << std::endl;
                } catch (const std::out_of_range& e) {
                    std::cerr << "target_id out of range: " << target_id << std::endl;
                }
            }
        }

        for (const auto& [source_id, target_id] : mpc_node.SourceTargetNodes) {
            try {
                int target_id_int = std::stoi(target_id); // Convert target_id to int
                std::cout << "Converted target_id: " << target_id_int << std::endl; // Debug statement

                if (mpc_node.obs_dict_.find(target_id_int) != mpc_node.obs_dict_.end()) {
                    const auto& coords = mpc_node.obs_dict_.at(target_id_int);
                    std::cout << "Found coordinates: (" << coords.first << ", " << coords.second << ") for target_id: " << target_id_int << std::endl; // Debug statement
                    path.emplace_back(target_id_int, coords.first, coords.second);
                } else {
                    std::cout << "target_id_int " << target_id_int << " not found in obs_dict_" << std::endl;
                }
            } catch (const std::invalid_argument& e) {
                std::cerr << "Invalid target_id: " << target_id << std::endl;
            } catch (const std::out_of_range& e) {
                std::cerr << "target_id out of range: " << target_id << std::endl;
            }
        }

        std::cout << "test" << std::endl;

        // Print path
        for (const auto& [id, x, y] : path) {
            std::cout << "Node ID: " << id << ", X: " << x << ", Y: " << y << std::endl;
        }

        std::cout << "test" << std::endl;

        

        // for (const auto& [source_id, target_id] : SourceTargetNodes) {
        //     if (new_node_data.find(target_id) != new_node_data.end()) {
        //         const auto& coords = new_node_data[target_id];
        //         path.emplace_back(target_id, coords.first, coords.second);
        //     }
        // }

        // // Her bir nokta için bir sonraki nokta ile arasındaki açıyı hesaplama
        // std::vector<double> angles, anglesOriginal;

        // for (size_t i = 0; i < pathOriginal.size() - 1; ++i) {
        //     double dx = pathOriginal[i + 1].second - pathOriginal[i].second;
        //     double dy = pathOriginal[i + 1].third - pathOriginal[i].third;
        //     double angle = std::atan2(dy, dx);
        //     anglesOriginal.push_back(angle);
        // }

        // for (size_t i = 0; i < path.size() - 1; ++i) {
        //     double dx = path[i + 1].second - path[i].second;
        //     double dy = path[i + 1].third - path[i].third;
        //     double angle = std::atan2(dy, dx);
        //     angles.push_back(angle);
        // }

        // if (!angles.empty()) {
        //     angles.push_back(angles.back());
        // }

        // if (!anglesOriginal.empty()) {
        //     anglesOriginal.push_back(anglesOriginal.back());
        // }

        // for (size_t i = 0; i < path.size(); ++i) {
        //     pathGoalsYawDegree.emplace_back(path[i].first, path[i].second, path[i].third, angles[i]);
        // }

        // for (size_t i = 0; i < pathOriginal.size(); ++i) {
        //     pathGoalsYawDegreeOriginal.emplace_back(pathOriginal[i].first, pathOriginal[i].second, pathOriginal[i].third, anglesOriginal[i]);
        // }

        // if (!pathGoalsYawDegreecalled) {
        //     pathGoalsYawDegreeCopy = pathGoalsYawDegreeOriginal;
        //     SourceTargetNodesCopy = SourceTargetNodesOriginal;
        //     pathGoalsYawDegreecalled = true;
        // }

        // std::string data_message = toString(edges_data_true);

    }

    std::vector<std::string> dijkstra(const std::string& source, const std::string& target,
                                            std::map<std::string, MpcNode::NodeInfo>& nodedictt,
                                            std::map<std::string, std::vector<std::pair<std::string, double>>>& edgedictt,
                                            const std::vector<std::string>& yasaklistesi,
                                            MpcNode& mpc_node) {

        std::cout << "____________________" << source << std::endl;
        std::cout << "____________________" << target << std::endl; 

        std::vector<std::string> nowaypoints;
        std::vector<std::string> nowaypoints2;

        // Remove blocked paths
        for (auto& ed : edgedictt) {
            for (auto it = ed.second.begin(); it != ed.second.end(); ) {
                if (std::find(yasaklistesi.begin(), yasaklistesi.end(), it->first) != yasaklistesi.end()) {
                    nowaypoints.push_back(ed.first);
                    nowaypoints2.push_back(it->first);
                    it = ed.second.erase(it);  // Remove the blocked path
                } else {
                    ++it;
                }
            }
        }

        std::unordered_map<std::string, double> unvisited;
        std::unordered_map<std::string, double> visited;
        std::unordered_map<std::string, std::vector<std::string>> paths;

        // Initialize unvisited nodes with infinite distance
        for (const auto& edge : edgedictt) {
            unvisited[edge.first] = std::numeric_limits<double>::infinity();
        }
        unvisited[source] = 0;
        paths[source] = {source};  // Initialize the path for the source

        std::cout << "Initial source node: " << source << std::endl;

        while (!unvisited.empty()) {
            // Find the node with the minimum distance
            auto minNode = *std::min_element(unvisited.begin(), unvisited.end(),
                                            [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });

            visited[minNode.first] = minNode.second;

            // Update distances to neighbors
            for (const auto& neighbor : edgedictt[minNode.first]) {
                if (visited.find(neighbor.first) != visited.end()) continue;

                double newDistance = visited[minNode.first] + neighbor.second; // assuming neighbor.second is the distance

                if (newDistance < unvisited[neighbor.first]) {
                    unvisited[neighbor.first] = newDistance;
                    paths[neighbor.first] = paths[minNode.first];  // Copy the path to the current node
                    paths[neighbor.first].push_back(neighbor.first);  // Add the neighbor to the path

                    nodedictt[neighbor.first].distance = newDistance;
                    nodedictt[neighbor.first].atalist = paths[neighbor.first];
                }
            }

            unvisited.erase(minNode.first);  // Remove the current node from unvisited
        }

        std::vector<std::string> yolll = nodedictt[target].atalist;


        std::cout << "Final path to target: ";
        for (const auto& step : yolll) {
            std::cout << step << " ";
        }
        std::cout << std::endl;
            
        if (std::find(yolll.begin(), yolll.end(), source) != yolll.end() &&
            std::find(yolll.begin(), yolll.end(), target) != yolll.end()) {
            std::cout << "yol var@@@@@@@@@@@@@@@@@@@" << std::endl;

            bool trflstate = false;
            bool yayastate = false;
            bool expathstate = false;

            
            if (trflstate || yayastate || expathstate) {
                std::cout << "kırmızı ışık bekliycem eski yolu kullanıcam yeni yol çıkarmıcam " << std::endl;

                std::string bagendxx;
                for (const auto& tempstopxx : yasaklistesi) {
                    if (std::find(mpc_node.expath_.begin(), mpc_node.expath_.end(), tempstopxx) != mpc_node.expath_.end()) {
                        bagendxx = tempstopxx;
                        break;
                    }
                }

                auto indexnoEXxx = std::find(mpc_node.expath_.begin(), mpc_node.expath_.end(), bagendxx) - mpc_node.expath_.begin();
                std::cout << "ex path: ";
                for (const auto& p : mpc_node.expath_) {
                    std::cout << p << " ";
                }
                std::cout << std::endl;

                std::string befbagendxx;
                if (mpc_node.expath_.size() > 2) {
                    befbagendxx = mpc_node.expath_[indexnoEXxx - 1];
                } else {
                    befbagendxx = mpc_node.expath_[indexnoEXxx];
                }
                std::cout << "before bagend" << befbagendxx << std::endl;
                std::cout << "bagend :::" << bagendxx << std::endl;

                nodedictt[befbagendxx].atalist.push_back(befbagendxx);
                return nodedictt[befbagendxx].atalist;
            } else {
                auto yolvar = true;
                mpc_node.expath_ = nodedictt[target].atalist;
                std::cout << "*************************" << std::endl;
                std::cout << "*************************" << std::endl;
                for (const auto& p : mpc_node.expath_) {
                    std::cout << p << " ";
                }
                std::cout << std::endl;
                std::cout << "*************************" << std::endl;
                std::cout << "*************************" << std::endl;
                return nodedictt[target].atalist;
            }
        } else {
            std::cout << "yol yok###################" << std::endl;

            std::string bagend;
            for (const auto& tempstopx : yasaklistesi) {
                if (std::find(mpc_node.expath_.begin(), mpc_node.expath_.end(), tempstopx) != mpc_node.expath_.end()) {
                    bagend = tempstopx;
                    break;
                }
            }

            auto indexnoEX = std::find(mpc_node.expath_.begin(), mpc_node.expath_.end(), bagend) - mpc_node.expath_.begin();
            std::cout << "ex path: ";
            for (const auto& p : mpc_node.expath_) {
                std::cout << p << " ";
            }
            std::cout << std::endl;

            std::string befbagend;
            if (mpc_node.expath_.size() > 2) {
                befbagend = mpc_node.expath_[indexnoEX - 2];
            } else {
                befbagend = mpc_node.expath_[indexnoEX];
            }
            std::cout << "before bagend" << befbagend << std::endl;
            std::cout << "bagend :::" << bagend << std::endl;

            nodedictt[befbagend].atalist.push_back(befbagend);
            return nodedictt[befbagend].atalist;
        }

        return nodedictt[target].atalist;
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

                if (true) { // You can add your condition here
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
        // Iterate over all "node" elements in the XML with the correct namespace
        for (pugi::xml_node node : root.children("node")) {
            std::string node_id = node.attribute("id").value();
            double d0 = 0.0;
            double d1 = 0.0;
            bool d0_found = false, d1_found = false;

            // Iterate over all child elements of the node
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
                // std::cout << "Extracted node: " << node_id << " (" << d0 << ", " << d1 << ")" << std::endl;
            } else {
                std::cerr << "Error: Incomplete or missing data for node id: " << node_id << std::endl;
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
                // std::cout << "Extracted edge: " << source_id << " -> " << target_id << " (dotted: " << d2_value << ")" << std::endl;
            } else {
                std::cerr << "Error: Incomplete edge data source: " << source_id << " target: " << target_id << std::endl;
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
            bool sollacurrent = false;
            for (const auto& ciz : edges_data_) {
                if (std::get<0>(ciz) == std::get<0>(node)) {
                    sollacurrent = !std::get<1>(ciz).empty();
                    break;
                }
            }
            nodedict[std::get<0>(node)] = {inf, {}, sollacurrent, std::get<1>(node), std::get<2>(node)};
        }

        // Print nodedict
        std::cout << "Node Dictionary:" << std::endl;
        for (const auto& node : nodedict) {
            std::cout << "Node: " << node.first << " - Distance: " << node.second.distance << " - Atalist: ";
            for (const auto& atalistNode : node.second.atalist) {
                std::cout << atalistNode << " ";
            }
            std::cout << "- Pass Through: " << node.second.pass_through << " - X: " << node.second.x << " - Y: " << node.second.y << std::endl;
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
            // Print edgedict
            std::cout << "Edge: " << source << " -> ";
            for (const auto& target : edgedict[source]) {
                std::cout << "(" << target.first << ", " << target.second << ") ";
            }
            std::cout << std::endl;
        }

        return {nodedict, edgedict};
    }

};

