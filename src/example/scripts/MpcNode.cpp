#include "mekatronom/MpcNode.hpp"

// Constructor
MpcNode::MpcNode(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) 
{
    // Get all parameters
    get_parameters();

    // Subscribers
    // image_sub_ = nh_.subscribe("automobile/image_raw", 1, &MpcNode::imageCb, this);
    imu_sub_ = nh_.subscribe("automobile/IMU", 1, &MpcNode::imuCb, this);
    localisation_sub_ = nh_.subscribe("automobile/localisation", 1, &MpcNode::localisationCb, this);
    obstacle_detection_sub_ = nh_.subscribe("obstacles", 1, &MpcNode::obstacleDetectionCb, this);
    carControl_pub_ = nh_.advertise<std_msgs::String>("automobile/command", 2);

    // Publishers
    // cmd_pub_ = nh_.advertise<std_msgs::Bool>("bool_msg", 1);

    // Timer
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_), &MpcNode::controlCb, this);
}

// Destructor
MpcNode::~MpcNode()
{
    
    ros::shutdown();
}

void MpcNode::controlCb(const ros::TimerEvent& event)
{
    try{
        if (imu_data_received_ && localisation_data_received_ && graphml_file_path_is_set_) {

            imu_data_received_ = false;
            localisation_data_received_ = false;
            std::cout << "\n\nmpc_started_: " << mpc_started_ << std::endl;
            if (!mpc_started_) {

                ROS_INFO("scenerio_name_: %s", scenerio_name_.c_str());
                MpcStartSetting mpc_start_setting(scenerio_name_, *this); 

                mpc_started_ = true;
                ROS_INFO("MPC started");
                // args["p"] = DM::vertcat(mpc_start_setting., state_target);
            }

            // ROS_ERROR("\n\nControl Callback is called\n\n");
            MpcRunning mpc_running(*this);            
            // ROS_ERROR("\n\nControl Callback is finished\n\n");

            // args["x0"] = vertcat(
            //     reshape(X0.T(), n_states * (N + 1), 1),
            //     reshape(u0.T(), n_controls * N, 1)
            // );

            // // Uncomment for debugging
            // // std::cout << "args['x0']: " << args["x0"] << std::endl;
            // // std::cout << "args['p']: " << args["p"] << std::endl;
            // // std::cout << "u0: " << u0.T() << std::endl;
            // // std::cout << "X0: " << X0.T() << std::endl;

            // std::map<std::string, DM> sol = solver(
            //     {{"x0", args["x0"]},
            //     {"lbx", args["lbx"]},
            //     {"ubx", args["ubx"]},
            //     {"lbg", args["lbg"]},
            //     {"ubg", args["ubg"]},
            //     {"p", args["p"]}}
            // );

            // u = reshape((sol["x"](Slice(n_states * (N + 1), sol["x"].size1()))).T(), n_controls, N).T();

        }
    }
    catch (const std::exception& e) {
        ROS_FATAL("Exception caught during processing and publishing data: %s", e.what());
        return;
    }
}

void MpcNode::get_parameters()
{
    try {
        std::cout << "\nGetting parameters" << std::endl;
        
        nh_.param("control_rate", control_rate_, 15);
        nh_.param("graph_file_path", graphml_filename_, std::string("fixed2.graphml"));
        nh_.param("scenerio_name", scenerio_name_, std::string("initial_settings"));
        nh_.param("Q_x", initial_settings_.Q_x, 1.0);
        nh_.param("Q_y", initial_settings_.Q_y, 2.0);
        nh_.param("Q_theta", initial_settings_.Q_theta, 0.1);
        nh_.param("R1", initial_settings_.R1, 0.5);
        nh_.param("R2", initial_settings_.R2, 0.05);
        nh_.param("step_horizon", initial_settings_.step_horizon, 0.3);
        nh_.param("N", initial_settings_.N, 1);
        nh_.param("rob_diam", initial_settings_.rob_diam, 0.354);
        nh_.param("wheel_radius", initial_settings_.wheel_radius, 1.0);
        nh_.param("L_value", initial_settings_.L_value, 0.1);
        nh_.param("Ly", initial_settings_.Ly, 0.03);
        nh_.param("v_max", initial_settings_.v_max, 0.25);
        nh_.param("v_min", initial_settings_.v_min, -0.15);
        nh_.param("omega_max", initial_settings_.omega_max, M_PI/10); //degree
        nh_.param("omega_min", initial_settings_.omega_min, -M_PI/10); //degree

        if (!graphml_file_path_is_set_) {
            try {
                
                graphml_file_path_ = find_file(graphml_filename_);
                graphml_file_path_is_set_ = true;
                ROS_INFO("GraphML file path: %s", graphml_file_path_.c_str());
                processAndPublishPath processAndPublishPath(graphml_file_path_,initial_settings_.source_node,initial_settings_.target_node, *this);
                // Debug the pathGoalsYawDegreeCopy_
                if (!djikstra_outputs_.pathGoalsYawDegree.empty()) {
                    std::cout << "Path Goals Yaw Degree Copy:" << std::endl;
                    for (const auto& [id, x, y, angle] : djikstra_outputs_.pathGoalsYawDegree) {
                        std::cout << "Node ID: " << id << ", X: " << x << ", Y: " << y << ", Angle: " << angle << std::endl;
                    }
                } else {
                    std::cout << "Path Goals Yaw Degree Copy is empty!" << std::endl;
                }
            } catch (const std::runtime_error& e) {
                ROS_FATAL("%s", e.what());
                ros::shutdown();
                return;
            }
        } 


    } catch (const std::exception& e) {
        ROS_FATAL("Exception caught during processing and publishing data: %s", e.what());
        return;
    }
}


// void MpcNode::imageCb(const sensor_msgs::Image::ConstPtr& msg)
// {
//   try {
//       cv_bridge::CvImagePtr cv_ptr;
//       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

//   } catch (cv_bridge::Exception& e) {
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//       return;
//   }
// }

void MpcNode::obstacleDetectionCb(const obstacle_detector::Obstacles::ConstPtr& msg)
{
    try {
        center_x_.clear();
        center_y_.clear();
        if (msg->circles.empty()) {
            ROS_INFO_THROTTLE(1, "\nNo obstacles detected");
            return;
        }
        else {
            ROS_INFO_THROTTLE(1, "\nObstacle data received");
            for (const auto& obs : msg->circles) {
                center_x_.push_back(obs.center.x);
                center_y_.push_back(obs.center.y);
                std::cout << "Obstacle center: " << obs.center.x << ", " << obs.center.y << std::endl;
            }
        }

    } catch (const std::exception& e) {
        ROS_FATAL("Exception caught during processing and publishing data: %s", e.what());
        return;
    }
}

void MpcNode::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{

    try{
        ROS_INFO_ONCE("\n\nIMU data received", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Quaternion quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(localisation_data_.roll, localisation_data_.pitch, localisation_data_.yaw);
        imu_data_received_ = true;
    } catch (const std::exception& e) {
        imu_data_received_ = false;
        ROS_FATAL("Exception caught during processing and publishing data: %s", e.what());
        return;
    }

}

void MpcNode::localisationCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    try {
        ROS_INFO_ONCE("\n\nLocalisation data received: X = %f, Y = %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
        localisation_data_.x = msg->pose.pose.position.x;
        localisation_data_.y = msg->pose.pose.position.y;
        localisation_data_received_ = true;
    } catch (const std::exception& e) {
        localisation_data_received_ = false;
        ROS_FATAL("Exception caught during processing and publishing data: %s", e.what());
        return;
    }
}

