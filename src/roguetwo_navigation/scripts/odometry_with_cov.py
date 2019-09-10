#!/usr/env/bin python

import rospy
from nav_msgs.msg import Odometry

class OdomWithCov(object):
    def __init__(self):
        self.pub = rospy.Publisher("/odometry", Odometry, queue_size=1)
        rospy.Subscriber("/encoder/odometry", Odometry, self.pub_odom)

    def pub_odom(self, odometry_msg):
        odometry = Odometry()
        odometry.header = odometry_msg.header
        odometry.child_frame_id = "base_link"

        odometry.pose.pose = odometry_msg.pose.pose 

        covariance = [0]*36
        covariance[0] = 0.1
        covariance[1] = 0.1
        covariance[5] = 0.00001
        odometry.pose.covariance = covariance

        odometry.twist = odometry_msg.twist

        self.pub.publish(odometry)


if __name__ == "__main__":
    rospy.init_node("odom_with_cov")
    OdomWithCov()
    rospy.spin()