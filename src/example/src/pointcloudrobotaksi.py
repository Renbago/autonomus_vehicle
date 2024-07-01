#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

def callback(data):
    data.header.frame_id = 'automobile_chassis_link'
    pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('change_frame_id', anonymous=True)
    rospy.Subscriber('/cloud_pcd', PointCloud2, callback)
    pub = rospy.Publisher('/cloud_pcd_transformed', PointCloud2, queue_size=10)
    rospy.spin()
