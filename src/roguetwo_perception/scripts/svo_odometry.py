#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros


class SVOPoseNode(object):
    # set publishers
    pub_odom = rospy.Publisher('/svo_odom', Odometry, queue_size=1)

    def __init__(self):
        # set subscribers
        rospy.Subscriber('/svo/pose_cam/0', PoseStamped, self.svo_pose_callback)

    def svo_pose_callback(self, svo_pose):
        # position is in wrong format
        ros_format_pose = PoseStamped()
        pose = svo_pose.pose
        position = pose.position
        orientation = pose.orientation
        ros_format_pose.pose.position.x = position.z
        ros_format_pose.pose.position.y = -position.x
        ros_format_pose.pose.position.z = -position.y
        orientation = svo_pose.pose.orientation
        ros_format_pose.pose.orientation = orientation
        

        header = Header()
        header.stamp = rospy.Time.now()
        ros_format_pose.header = header
        self.pub_odom.publish(ros_format_pose)


# start the node
if __name__ == '__main__':
    rospy.init_node("svo_pose_node")
    node = SVOPoseNode()
    rospy.spin()
