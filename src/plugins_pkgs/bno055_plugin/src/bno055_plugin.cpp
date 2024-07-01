#include "bno055_plugin.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>  // For tf::createQuaternionMsgFromRollPitchYaw

#define DEBUG false

namespace gazebo
{
    namespace bno055
    {   
        BNO055::BNO055() : ModelPlugin() {}

        void BNO055::Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr)
        {
            nh = boost::make_shared<ros::NodeHandle>();  
            timer = nh->createTimer(ros::Duration(0.01), std::bind(&BNO055::OnUpdate, this));

            // Save a pointer to the model for later use
            this->m_model = model_ptr;
            
            // Create topic name
            std::string topic_name = "/automobile/IMU";
            
            // Initialize ros, if it has not already been initialized.
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client_bno", ros::init_options::NoSigintHandler);
            }

            this->m_ros_node.reset(new ::ros::NodeHandle("/bnoNODEvirt"));
            this->m_pubBNO = this->m_ros_node->advertise<sensor_msgs::Imu>(topic_name, 2);

            if(DEBUG)
            {
                std::cerr << "\n\n";
                ROS_INFO_STREAM("====================================================================");
                ROS_INFO_STREAM("[bno055_plugin] attached to: " << this->m_model->GetName());
                ROS_INFO_STREAM("[bno055_plugin] publish to: "  << topic_name);
                ROS_INFO_STREAM("[bno055_plugin] Useful data: linear z, angular x, angular y, angular z");
                ROS_INFO_STREAM("====================================================================\n\n");
            }
        }

        void BNO055::OnUpdate()
        {        
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "automobile_chassis_link";

            // Set linear acceleration
            imu_msg.linear_acceleration.x = this->m_model->RelativeLinearVel().X();  
            imu_msg.linear_acceleration.y = this->m_model->RelativeLinearVel().Y();  
            imu_msg.linear_acceleration.z = this->m_model->RelativeLinearVel().Z();
            imu_msg.angular_velocity.x = this->m_model->RelativeAngularVel().X();
            imu_msg.angular_velocity.y = this->m_model->RelativeAngularVel().Y();
            imu_msg.angular_velocity.z = this->m_model->RelativeAngularVel().Z();
            // Set angular velocity 
            // Set orientation from roll, pitch, yaw
            double roll = this->m_model->RelativePose().Rot().Roll();
            double pitch = this->m_model->RelativePose().Rot().Pitch();
            double yaw = this->m_model->RelativePose().Rot().Yaw();
            tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
            tf::quaternionTFToMsg(q, imu_msg.orientation);

            // Set covariance for orientation and angular velocity
            std::fill(std::begin(imu_msg.orientation_covariance), std::end(imu_msg.orientation_covariance), 0.005);
            std::fill(std::begin(imu_msg.angular_velocity_covariance), std::end(imu_msg.angular_velocity_covariance), 0.01);
            
            // Adjusted linear acceleration covariance
            imu_msg.linear_acceleration_covariance[0] = 0.01;  // X
            imu_msg.linear_acceleration_covariance[4] = 0.01;  // Y
            imu_msg.linear_acceleration_covariance[8] = 1.0;   // Z, higher uncertainty
            imu_msg.orientation_covariance[8] = 0.01;          // Yaw
            imu_msg.angular_velocity_covariance[8] = 0.01;     // Z


            this->m_pubBNO.publish(imu_msg);
        };      
    }; // namespace bno055
    GZ_REGISTER_MODEL_PLUGIN(bno055::BNO055)
}; // namespace gazebo
