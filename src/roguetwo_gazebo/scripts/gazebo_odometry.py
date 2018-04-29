#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
from roguetwo_perception.msg import SE2
from tf.transformations import euler_from_quaternion

import numpy as np
import math
import tf2_ros


class OdometryNode(object):
    # set publishers
    pub_odom = rospy.Publisher('/encoder/odometry', Odometry, queue_size=1)
    #pub_se2 = rospy.Publisher('/se2_state', SE2, queue_size=1)

    def __init__(self):
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_received_stamp = None

        # set the update rate
        rospy.Timer(rospy.Duration(0.05), self.timer_callback)  # 20 hz

        self.tf_pub = tf2_ros.TransformBroadcaster()

        # set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        # find index of the rover
        try:
            arrayIndex = msg.name.index('roguetwo::base_link')
        except ValueError as e:
            pass
        else:
            # extract our current position information
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
        self.last_received_stamp = rospy.Time.now()

    def timer_callback(self, event):
        if self.last_received_stamp is None:
            return

        cmd = Odometry()
        cmd.header.stamp = self.last_received_stamp
        cmd.header.frame_id = 'odom'
        cmd.child_frame_id = 'base_link'
        cmd.pose.pose = self.last_received_pose
        cmd.twist.twist = self.last_received_twist
        self.pub_odom.publish(cmd)

        tf = TransformStamped(
            header=Header(
                frame_id=cmd.header.frame_id,
                stamp=cmd.header.stamp
            ),
            child_frame_id=cmd.child_frame_id,
            transform=Transform(
                translation=cmd.pose.pose.position,
                rotation=cmd.pose.pose.orientation
            )
        )
        self.tf_pub.sendTransform(tf)

        orientation = self.last_received_pose.orientation
        quaternion_arr = np.array([orientation.x,
                                   orientation.y,
                                   orientation.z,
                                   orientation.w])
        euler = euler_from_quaternion(quaternion_arr)

        se2_msg = SE2()
        se2_msg.x = self.last_received_pose.position.x
        se2_msg.y = self.last_received_pose.position.y
        se2_msg.yaw = euler[1]
        print ("Curr Yaw: ", se2_msg.yaw)


# start the node
if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode()
    rospy.spin()
