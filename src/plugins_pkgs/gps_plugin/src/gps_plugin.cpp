#include "gps_plugin.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

#define DEBUG false

namespace gazebo
{
    class GPS : public ModelPlugin
    {
    public:
        GPS() : ModelPlugin() {}

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                 "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            this->model = _model;
            this->world = _model->GetWorld();

            this->nodeHandle.reset(new ros::NodeHandle("localisationNODEvirt"));
            this->localisationPublisher = this->nodeHandle->advertise<geometry_msgs::PoseWithCovarianceStamped>("/automobile/localisation", 10);

            // Set up a ROS timer to call OnUpdate if you want more realistic increase the duration
            this->timer = this->nodeHandle->createTimer(ros::Duration(0.00001), &GPS::OnTimerEvent, this);
        }

        void OnTimerEvent(const ros::TimerEvent&)
        {
            geometry_msgs::PoseWithCovarianceStamped localisationValue;

            localisationValue.header.frame_id = "map";
            localisationValue.header.stamp = ros::Time::now();

            // Assuming m_gps_pose is previously updated with your GPS logic
            localisationValue.pose.pose.position.x = this->model->RelativePose().Pos().X() + (static_cast<float>(rand()) / RAND_MAX * 0.2f) - 0.1f;
            localisationValue.pose.pose.position.y = abs(this->model->RelativePose().Pos().Y()) + (static_cast<float>(rand()) / RAND_MAX * 0.2f) - 0.1f;
            double yaw = this->model->RelativePose().Rot().Yaw();

            tf::Quaternion quaternion = tf::createQuaternionFromYaw(yaw);
            geometry_msgs::Quaternion q_msg;
            tf::quaternionTFToMsg(quaternion, q_msg);

            localisationValue.pose.pose.orientation = q_msg;

            // Set covariance matrix for position x, y
            double covariance[36] = {0};
            covariance[0] = 0.04; // Variance of x
            covariance[7] = 0.04; // Variance of y
            covariance[14] = 0.0; // Variance of z, very high since we don't measure it
            covariance[21] = 999999; // Variance of roll
            covariance[28] = 999999; // Variance of pitch
            covariance[35] = 0.005; // Variance of yaw

            for (int i = 0; i < 36; ++i) {
                localisationValue.pose.covariance[i] = covariance[i];
            }

            this->localisationPublisher.publish(localisationValue);
        }

    private:
        std::unique_ptr<ros::NodeHandle> nodeHandle;
        ros::Publisher localisationPublisher;
        ros::Timer timer;  // ROS timer to handle scheduled updates
        physics::ModelPtr model;
        physics::WorldPtr world;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(GPS)
} // namespace gazebo