#!/usr/bin/env python3
# coding: utf-8

import rospy
import tf2_ros
import geometry_msgs.msg
from gazebo_msgs.msg import LinkStates

br = tf2_ros.TransformBroadcaster()

# def publish_fixed_transform():
#     t = geometry_msgs.msg.TransformStamped()
#     t.header.stamp = rospy.Time.now()
#     t.header.frame_id = "automobile_chassis_link"
#     t.child_frame_id = "automobile_laser_frame"
#     t.transform.translation.x = 0.0
#     t.transform.translation.y = -0.22
#     t.transform.translation.z = -0.005
#     # Sabit dönüşüm için quaternion rpy'den (0, 0, 0) dönüştürülür
#     t.transform.rotation.x = 0
#     t.transform.rotation.y = 0
#     t.transform.rotation.z = 0
#     t.transform.rotation.w = 1

#     br.sendTransform(t)

def link_states_callback(msg):
    t = geometry_msgs.msg.TransformStamped()
    
    # publish_fixed_transform()

    for i in range(len(msg.name)):
        if "automobile::chassis::link" in msg.name[i]:
            t.header.stamp = rospy.Time.now() + rospy.Duration(0, 100000000*(i+1))
            t.header.frame_id = "map"  # Odometri çerçevesinden
            t.child_frame_id = "automobile_chassis_link"  # aracın şasisine transform
            # Pose verilerini al
            t.transform.translation.x = msg.pose[i].position.x
            t.transform.translation.y = msg.pose[i].position.y
            t.transform.translation.z = msg.pose[i].position.z
            t.transform.rotation.x = msg.pose[i].orientation.x
            t.transform.rotation.y = msg.pose[i].orientation.y
            t.transform.rotation.z = msg.pose[i].orientation.z
            t.transform.rotation.w = msg.pose[i].orientation.w

            br.sendTransform(t)

if __name__ == '__main__':
    try:
        rospy.init_node('gazebo_link_states_to_tf', anonymous=True)
        rospy.set_param('/use_sim_time', True)
        rospy.Subscriber('/gazebo/link_states', LinkStates, link_states_callback)
        rate = rospy.Rate(10.0)  
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

