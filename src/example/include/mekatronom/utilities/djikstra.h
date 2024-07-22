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

        // Convert nodes_data_ and edges_data_ to the required types
        std::map<std::string, MpcNode::NodeInfo> nodes_data_map;
        for (const auto& node : mpc_node.nodes_data_) {
            MpcNode::NodeInfo node_info = {std::numeric_limits<double>::infinity(), {}, false, std::get<1>(node), std::get<2>(node)};
            nodes_data_map[std::get<0>(node)] = node_info;
        }

        std::map<std::string, std::vector<std::pair<std::string, double>>> edges_data_map;
        for (const auto& edge : mpc_node.edges_data_) {
            std::string source = std::get<0>(edge);
            std::string target = std::get<1>(edge);
            double distance = 1.0;

            if (edges_data_map.find(source) != edges_data_map.end()) {
                edges_data_map[source].emplace_back(target, distance);
            } else {
                edges_data_map[source] = {{target, distance}};
            }
        }
        

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
        auto [noded, edged] = extract_graph(nodes_data_map, edges_data_map);

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
    
        // ROS_INFO_ONCE("Finished to extract_graph");
        // std::vector<int> path_short = finding_path(source_node_, target_node_,  mpc_node.nodes_data_, mpc_node.edges_data_, mpc_node.obs_dontuse_);
        // ROS_INFO_ONCE("Finished to finding_path");
        // auto pathOriginal = stformat(path_short);
        // ROS_INFO_ONCE("Finished to stformat");
        // std::vector<std::string> string_path = convertPathToString(path_short);
        // ROS_INFO_ONCE("Finished to convertPathToString");
        // auto [newnodedictionary, stlist] = beizer(string_path, noded, mpc_node);
        // ROS_INFO_ONCE("Finished to beizer");

        // std::cout << "string_path: ";
        // for (const auto& s : string_path) {
        //     std::cout << s << " ";
        // }
        // std::cout << std::endl;

        // std::cout << "newnodedictionary: ";
        // for (const auto& [key, value] : newnodedictionary) {
        //     std::cout << key << ": (" << value.first << ", " << value.second << ") ";
        // }
        // std::cout << std::endl;

        // std::cout << "stlist: ";
        // for (const auto& t : stlist) {
        //     std::cout << t << " ";
        // }
        // std::cout << std::endl;

        // std::cout << "pathOriginal: ";
        // for (const auto& t : pathOriginal) {
        //     std::cout << t << " ";
        // }
        // std::cout << std::endl;

        // std::cout << "path_short: ";
        // for (const auto& p : path_short) {
        //     std::cout << p << " ";
        // }
        // std::cout << std::endl;

        // std::cout << "noded: ";
        // for (const auto& [key, value] : noded) {
        //     std::cout << key << ": (" << value.x << ", " << value.y << ") ";
        // }
        // std::cout << std::endl;

        // std::cout << "edged: ";
        // for (const auto& [key, value] : edged) {
        //     std::cout << key << ": ";
        //     for (const auto& v : value) {
        //         std::cout << "(" << v.first << ", " << v.second << ") ";
        //     }
        //     std::cout << " ";
        // }
        // std::cout << std::endl;

        // std::cout << "mpc_node.obs_dontuse_: ";
        // for (const auto& obs : mpc_node.obs_dontuse_) {
        //     std::cout << obs << " ";
        // }
        // std::cout << std::endl;

        // obs_dict = newnodedictionary;
        // edges_data_true = stlist;

        // SourceTargetNodesOriginal.clear();
        // for (const auto& edge : pathOriginal) {
        //     if (std::get<2>(edge)) {
        //         SourceTargetNodesOriginal.emplace_back(std::get<0>(edge), std::get<1>(edge));
        //     }
        // }

        // SourceTargetNodes.clear();
        // for (const auto& edge : edges_data_true) {
        //     if (std::get<2>(edge)) {
        //         SourceTargetNodes.emplace_back(std::get<0>(edge), std::get<1>(edge));
        //     }
        // }

        // path.clear();
        // for (const auto& [source_id, target_id] : SourceTargetNodesOriginal) {
        //     if (obs_dict.find(source_id) != obs_dict.end()) {
        //         auto [x, y] = obs_dict[source_id];
        //         pathOriginal.emplace_back(source_id, x, y);
        //     }
        //     if (obs_dict.find(target_id) != obs_dict.end()) {
        //         auto [x, y] = obs_dict[target_id];
        //         pathOriginal.emplace_back(target_id, x, y);
        //     }
        // }

        // for (const auto& [source_id, target_id] : SourceTargetNodes) {
        //     if (obs_dict.find(target_id) != obs_dict.end()) {
        //         auto [x, y] = obs_dict[target_id];
        //         path.emplace_back(target_id, x, y);
        //     }
        // }

        // std::vector<double> angles, anglesOriginal;

        // for (size_t i = 0; i < pathOriginal.size() - 1; ++i) {
        //     double dx = std::get<1>(pathOriginal[i + 1]) - std::get<1>(pathOriginal[i]);
        //     double dy = std::get<2>(pathOriginal[i + 1]) - std::get<2>(pathOriginal[i]);
        //     double angle = std::atan2(dy, dx);
        //     anglesOriginal.push_back(angle);
        // }

        // for (size_t i = 0; i < path.size() - 1; ++i) {
        //     double dx = std::get<1>(path[i + 1]) - std::get<1>(path[i]);
        //     double dy = std::get<2>(path[i + 1]) - std::get<2>(path[i]);
        //     double angle = std::atan2(dy, dx);
        //     angles.push_back(angle);
        // }

        // if (!angles.empty()) {
        //     angles.push_back(angles.back());
        // }

        // if (!anglesOriginal.empty()) {
        //     anglesOriginal.push_back(anglesOriginal.back());
        // }

        // std::vector<std::tuple<int, double, double, double>> pathGoalsYawDegree, pathGoalsYawDegreeOriginal;

        // for (size_t i = 0; i < path.size(); ++i) {
        //     pathGoalsYawDegree.emplace_back(std::get<0>(path[i]), std::get<1>(path[i]), std::get<2>(path[i]), angles[i]);
        // }

        // for (size_t i = 0; i < pathOriginal.size(); ++i) {
        //     pathGoalsYawDegreeOriginal.emplace_back(std::get<0>(pathOriginal[i]), std::get<1>(pathOriginal[i]), std::get<2>(pathOriginal[i]), anglesOriginal[i]);
        // }

        // // Data publishing simulation
        // std::string data_message;
        // for (const auto& edge : edges_data_true) {
        //     data_message += std::to_string(std::get<0>(edge)) + "," + std::to_string(std::get<1>(edge)) + "," + (std::get<2>(edge) ? "true" : "false") + ";";
        // }
        // std::cout << "Graph data published:" << data_message << std::endl;
     
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
        for (const auto& [key, value] : node_d) {
            new_node_data[key] = {value.x, value.y};
        }

        std::vector<std::string> new_path = path;
        size_t path_length = path.size();

        for (size_t f = 0; f < path_length - 2; ++f) {
            double angel_rad1, angel_rad2, angel_deg1, angel_deg2;

            if (node_d[path[f]].x == node_d[path[f + 1]].x) {
                angel_rad1 = 1.57;
            } else {
                angel_rad1 = atan((node_d[path[f]].y - node_d[path[f + 1]].y) /
                                (node_d[path[f]].x - node_d[path[f + 1]].x));
            }
            angel_deg1 = angel_rad1 * 57.3;

            if (node_d[path[f + 1]].x == node_d[path[f + 2]].x) {
                angel_rad2 = 1.57;
            } else {
                angel_rad2 = atan((node_d[path[f + 1]].y - node_d[path[f + 2]].y) /
                                (node_d[path[f + 1]].x - node_d[path[f + 2]].x));
            }
            angel_deg2 = angel_rad2 * 57.3;

            double b_andgel = abs(angel_deg1 - angel_deg2);

            if (b_andgel > 55 && b_andgel < 110) {
                std::vector<std::pair<double, double>> controlPts = {
                    {node_d[path[f]].x, node_d[path[f]].y},
                    {node_d[path[f + 1]].x, node_d[path[f + 1]].y},
                    {node_d[path[f + 2]].x, node_d[path[f + 2]].y}
                };

                int numPts = 2;
                std::vector<double> t(numPts + 1);
                std::generate(t.begin(), t.end(), [n = 0, numPts]() mutable { return n++ * (1.0 / numPts); });

                std::vector<double> B_x(numPts + 1), B_y(numPts + 1);
                for (int i = 0; i <= numPts; ++i) {
                    B_x[i] = (1 - t[i]) * ((1 - t[i]) * controlPts[0].first + t[i] * controlPts[1].first) +
                            t[i] * ((1 - t[i]) * controlPts[1].first + t[i] * controlPts[2].first);
                    B_y[i] = (1 - t[i]) * ((1 - t[i]) * controlPts[0].second + t[i] * controlPts[1].second) +
                            t[i] * ((1 - t[i]) * controlPts[1].second + t[i] * controlPts[2].second);
                }

                std::vector<std::string> temp_new_nodelist;
                for (int new_p = 1; new_p < numPts; ++new_p) {
                    mpc_node.new_point_counter_++;
                    std::string new_point_str = std::to_string(mpc_node.new_point_counter_);
                    new_node_data[new_point_str] = {B_x[new_p], B_y[new_p]};
                    if (node_d[path[f]].pass_through) {
                        node_d[new_point_str] = {std::numeric_limits<double>::infinity(), {}, true, B_x[new_p], B_y[new_p]};
                    }
                    temp_new_nodelist.push_back(new_point_str);
                }
                new_path.erase(std::remove(new_path.begin(), new_path.end(), path[f + 1]), new_path.end());
                auto it = std::find(new_path.begin(), new_path.end(), path[f + 1]);
                new_path.insert(it, temp_new_nodelist.begin(), temp_new_nodelist.end());
            }
        }

        std::vector<std::tuple<std::string, std::string, bool>> source_target;
        for (size_t n_edge = 0; n_edge < new_path.size() - 1; ++n_edge) {
            source_target.emplace_back(new_path[n_edge], new_path[n_edge + 1], true);
        }

        return {new_node_data, source_target};
    }



    std::vector<std::tuple<int, int, bool>> stformat(const std::vector<int>& new_path) {
        std::vector<std::tuple<int, int, bool>> source_target;
        size_t pathlen = new_path.size();

        for (size_t n_edge = 0; n_edge < pathlen - 1; ++n_edge) {
            source_target.emplace_back(new_path[n_edge], new_path[n_edge + 1], true);
        }

        return source_target;
    }



    // std::vector<int> finding_path(const std::string& source, const std::string& target,
    //                                 const std::vector<std::tuple<std::string, double, double>>& nodes_data,
    //                                 const std::vector<std::tuple<std::string, std::string, bool>>& edges_data,
    //                                 const std::vector<std::string>& obs_dontuse) {
    //     if (source.empty() || target.empty()) {
    //         std::cerr << "Error: source or target node is empty. Source: " << source << ", Target: " << target << std::endl;
    //         return {};
    //     }
    //     if (std::find(nodes_data.begin(), nodes_data.end(), source) == nodes_data.end() || std::find(nodes_data.begin(), nodes_data.end(), target) == nodes_data.end()) {
    //         std::cerr << "Error: source or target node does not exist in the graph. Source: " << source << ", Target: " << target << std::endl;
    //         return {};
    //     }


    //     // std::map<std::string, double> unvisited;
    //     // for (const auto& [node, _] : edges_data) {
    //     //     if (!node.empty()) {
    //     //         unvisited[node] = std::numeric_limits<double>::infinity();
    //     //     } else {
    //     //         std::cerr << "Error: empty node encountered in edge dictionary" << std::endl;
    //     //     }
    //     // }
    //     // unvisited[source] = 0;
    //     // std::map<std::string, double> visited;

    //     std::vector<std::string> nowaypoints;
    //     std::vector<std::string> nowaypoints2;

    //     for (const auto& ed : edges_data) {
    //         for (const auto& jkl : std::get<1>(ed)) {
    //             for (const auto& dont : obs_dontuse) {
    //                 std::string edg = std::get<0>(ed);
    //                 if (std::find_if(edges_data.begin(), edges_data.end(), [edg](const auto& edge) { return std::get<0>(edge) == edg; }) != edges_data.end()) {
    //                     if (std::get<0>(jkl) == dont) {
    //                         nowaypoints.push_back(edg);
    //                         nowaypoints2.push_back(std::get<0>(jkl));
    //                         auto& temppp = edgedict[std::get<0>(ed)];
    //                         for (auto it = temppp.begin(); it != temppp.end(); ) {
    //                             if (*it == jkl) {
    //                                 it = temppp.erase(it);
    //                             } else {
    //                                 ++it;
    //                             }
    //                         }
    //                         edgedict[std::get<0>(ed)] = temppp;
    //                     }
    //                 }
    //             }
    //         }
    //     }



    //     // while (!unvisited.empty()) {
    //     //     auto minNodeIter = std::min_element(unvisited.begin(), unvisited.end(),
    //     //                                         [](const auto& lhs, const auto& rhs) {
    //     //                                             return lhs.second < rhs.second;
    //     //                                         });
    //     //     std::string minNode = minNodeIter->first;
    //     //     visited[minNode] = minNodeIter->second;

    //     //     for (const auto& neighbor : edgedict[minNode]) {
    //     //         const auto& distance = neighbor.second;
    //     //         if (visited.find(neighbor.first) != visited.end()) continue;
    //     //         double newCost = nodedict[minNode].distance + 1;
    //     //         if (newCost < unvisited[neighbor.first]) {
    //     //             nodedict[neighbor.first].distance = newCost;
    //     //             nodedict[neighbor.first].atalist = nodedict[minNode].atalist;
    //     //             if (!minNode.empty()) {
    //     //                 nodedict[neighbor.first].atalist.push_back(minNode);
    //     //             } else {
    //     //                 std::cerr << "Error: minNode is empty when adding to neighbor's atalist" << std::endl;
    //     //             }
    //     //             unvisited[neighbor.first] = newCost;
    //     //         }
    //     //     }

    //     //     unvisited.erase(minNodeIter);
    //     // }

    //     // if (!target.empty()) {
    //     //     nodedict[target].atalist.push_back(target);
    //     // } else {
    //     //     std::cerr << "Error: target node is empty" << std::endl;
    //     //     return {};
    //     // }

    //     // std::cout << "Final atalist for target " << target << ": ";
    //     // for (const auto& node : nodedict[target].atalist) {
    //     //     std::cout << node << " ";
    //     // }
    //     // std::cout << std::endl;

    //     // if (std::find(nodedict[target].atalist.begin(), nodedict[target].atalist.end(), source) != nodedict[target].atalist.end() &&
    //     //     std::find(nodedict[target].atalist.begin(), nodedict[target].atalist.end(), target) != nodedict[target].atalist.end()) {
    //     //     std::vector<int> int_path;
    //     //     for (const auto& node : nodedict[target].atalist) {
    //     //         if (node.empty()) {
    //     //             std::cerr << "Error: encountered empty node in atalist" << std::endl;
    //     //             return {};
    //     //         }
    //     //         std::cout << "Converting node: " << node << std::endl;
    //     //         try {
    //     //             int_path.push_back(std::stoi(node));
    //     //         } catch (const std::invalid_argument& e) {
    //     //             std::cerr << "Invalid argument: cannot convert " << node << " to an integer" << std::endl;
    //     //             return {};
    //     //         } catch (const std::out_of_range& e) {
    //     //             std::cerr << "Out of range: " << node << " is out of integer range" << std::endl;
    //     //             return {};
    //     //         }
    //     //     }
    //     //     return int_path;
    //     // } else {
    //     //     std::cerr << "No path found from " << source << " to " << target << std::endl;
    //     //     return {};
    //     // }
    // }


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
    extract_graph(const std::map<std::string, MpcNode::NodeInfo>& nodes_data, const std::map<std::string, std::vector<std::pair<std::string, double>>>& edges_data) {

        const double inf = std::numeric_limits<double>::infinity();

        std::map<std::string, MpcNode::NodeInfo> nodedict;
        std::map<std::string, std::vector<std::pair<std::string, double>>> edgedict;

        // Fill nodedict
        for (const auto& node : nodes_data) {
            bool sollacurrent = false;
            for (const auto& edge : edges_data) {
                if (edge.first == node.first) {
                    for (const auto& target : edge.second) {
                        sollacurrent = true; // Assuming any edge starting from this node sets solla to true
                        break;
                    }
                }
            }
            nodedict[node.first] = {inf, {}, sollacurrent, node.second.x, node.second.y};
        }

        // Fill edgedict
        for (const auto& edge : edges_data) {
            std::string source = edge.first;
            for (const auto& target : edge.second) {
            if (edgedict.find(source) != edgedict.end()) {
                edgedict[source].emplace_back(target.first, target.second);
            } else {
                edgedict[source] = {{target.first, target.second}};
            }
            
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

