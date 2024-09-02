#pragma once

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "utils/IMU.h"           // Assuming 'utils' is the package name and 'IMU' is the message type
#include "utils/localisation.h"  // Assuming 'localisation' is the message type
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include "casadi/casadi.hpp"
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <pugixml.hpp>
#include <cmath> 
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <utility> 
#include <map>
#include <functional>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <filesystem>
#include <locale.h>  
#include <nlohmann/json.hpp>
// #inclzude <mpc_start_setting2.h>

using namespace casadi;
using json = nlohmann::json;

namespace fs = std::filesystem;

class MpcNode {
public:
  // Constructor and Destructor
  MpcNode(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~MpcNode();
  
  ros::Publisher carControl_pub_;

  // std::vector<std::tuple<std::string, double, double>> extract_nodes_data(pugi::xml_node root);
  // std::vector<std::tuple<std::string, std::string, bool>> extract_edges_data(pugi::xml_node root);
  // std::vector<int> finding_path(int temp_source_, int temp_target_, std::map<int, std::pair<double, double>>& noded_, std::map<int, std::pair<int, int>>& edged_, std::vector<std::string>& obs_dontuse_);
  double new_point_counter_ = 600;

  typedef struct {
    double distance;
    std::vector<std::string> atalist;
    bool pass_through;
    double x;
    double y;
  } NodeInfo;
  NodeInfo node_info_;

  typedef struct {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
  } Pose;
  Pose localisation_data_;

  typedef struct {
    int obstacles_checking;
    int watchdogTimer;
    int went_node;
  } Timers;
  Timers last_update_time_;

  typedef struct {
    double Q_x;
    double Q_y;
    double Q_theta;
    double R1;
    double R2;
    double step_horizon;
    int N;
    double rob_diam;
    double wheel_radius;
    double L_value;
    double Ly;
    double v_max;
    double v_min;
    double omega_max;
    double omega_min;
    int iteration;
    double goal_checker{0.2};
    std::vector<std::string> parking_nodes_id{"900","901","902","903","904","910","911","912","913","914"};
    std::vector<std::string> parking_spot_is_full;
    std::string source_node{"263"};
    std::string target_node{"244"};
    std::vector<std::string> dont_check_obstacles_this_nodes{"228","229","230","231","232","233","234","235","236","237","238","239","240"};
    std::vector<std::string> excluded_nodes = {"273"};
    std::vector<std::string> obstacles_array = {" "};
    std::vector<std::string> parkings_are_available{" " };
  } Settings;
  Settings initial_settings_;

  typedef struct {
    std::vector<std::pair<std::string, std::string>> SourceTargetNodesOriginal;
    std::vector<std::pair<std::string, std::string>> SourceTargetNodes;
    std::vector<std::pair<std::string, std::string>> SourceTargetNodesCopy;
    std::vector<std::tuple<int, double, double, double>> pathGoalsYawDegree;
    std::vector<std::tuple<int, double, double, double>> pathGoalsYawDegreeOriginal;
    std::vector<std::tuple<int, double, double, double>> pathGoalsYawDegreeCopy;
    std::map<std::string, std::pair<double, double>> obstacle_node_positions;
    bool pass_through{false};
    std::map<std::string, MpcNode::NodeInfo> node_dict;
  } djikstra_outputs;
  djikstra_outputs djikstra_outputs_; 

  typedef struct {
    DM u0;
    DM X0;
    DM state_init;
    DM state_target;
    DM next_state;
    int n_states;
    int n_controls;
    std::map<std::string, DM> args;
    Function solver;
    DM cat_states;
    DM cat_controls;
    Function f;
    double steerAngle;
    double steerLateral;
    DM u;
    int last_path_index;
  } Control;
  Control mpc_setting_outputs_;

  std::vector<std::tuple<std::string, double, double>> nodes_data_;
  std::vector<std::tuple<std::string, std::string, bool>> edges_data_;
  std::vector<std::tuple<int, int, bool>> edges_data_true_ilkverisyon_;
  std::string car_behaviour_state_ = "keep_lane";

  std::vector<int> center_x_, center_y_;
  std::vector<std::tuple<std::string, std::string, bool>> pathOriginal_;
  std::vector<std::tuple<int, double, double>> path_;
  std::map<std::string, std::pair<double, double>> new_node_data_;
  std::map<std::string, std::pair<double, double>> obs_dict_;
  std::vector<std::string> expath_;
  std::vector<std::string> shortest_path_;
  std::tuple<int, double, double, double> goal_id_;

  int checking_counter_{0};
  bool pathGoalsYawDegreecalled_{false};
  bool distance_flag_{false};
  std::string closest_node_id_original_;
  std::string closest_node_id_;

  std::string graphml_file_path_;
  std::string scenerio_name_;
  std::string graphml_filename_ = "gercek2.graphml";


  // Public Functions for mpc_running.h
  void shiftTimestep(MpcNode& node);
  void carControlPublisher(MpcNode& node);
  // int calculateClosestNodeId(const std::vector<std::tuple<int, double, double, double>>& pathGoalsYawDegree,
  //                             double position_x, double position_y);


  std::string find_file(const std::string& filename) {
      fs::path current_path = fs::current_path();
      for (const auto& entry : fs::recursive_directory_iterator(current_path)) {
          if (entry.path().filename() == filename) {
              return entry.path().string();
          }
      }
      throw std::runtime_error("File not found: " + filename);
  }

private:
  
  // Private Functions
  // void imageCb(const sensor_msgs::Image::ConstPtr& msg);
  void controlCb(const ros::TimerEvent&);
  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
  void localisationCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg); 
  void process_and_publish_data(int temp_source, int temp_target);
  
  void function_one();
  void get_parameters();

  // Private Variables
  int control_rate_;
  bool mpc_started_{ false };
  bool imu_data_received_ = false;
  bool localisation_data_received_ = false;
  bool graphml_file_path_is_set_ = false;
  // Timers
  ros::Timer control_timer_;

  // Subscribers
  ros::Subscriber imu_sub_;
  ros::Subscriber localisation_sub_;  
  ros::Subscriber image_sub_;
  // Publishers
  ros::Publisher cmd_pub_;
  // Node Handle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  sensor_msgs::Image image_data_;



};

template <typename T, std::size_t... Is>
void print_tuple_impl(std::ostream& os, const T& t, std::index_sequence<Is...>) {
    ((os << (Is == 0 ? "" : ", ") << std::get<Is>(t)), ...);
}

template <typename... Args>
std::ostream& operator<<(std::ostream& os, const std::tuple<Args...>& t) {
    os << "(";
    print_tuple_impl(os, t, std::index_sequence_for<Args...>{});
    os << ")";
    return os;
}

#include "mekatronom/utilities/djikstra.h"
#include "mekatronom/utilities/traffic_sign_manager.h"
#include "mekatronom/utilities/mpc_running.h"
#include "mekatronom/utilities/mpc_start_setting.h"

