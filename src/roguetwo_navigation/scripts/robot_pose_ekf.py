#!/usr/bin/env python
import sys, select, termios, tty

import rospy
from sensor_msgs.msg import Imu, NavSatFix   # for IMU
from nav_msgs.msg import Odometry  # for wheel and visual odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

"""
This file handles subscribing to the vo and imu publishers and then feeding
them into the robot_pose_ekf.  The output should be a more accurate odometry
estimate
"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def imu_callback(imu):
    publish_imu(imu)


def vo_callback(data):
    raise NotImplementedError


def imu_listener():
    rospy.Subscriber("/imu", Imu, imu_callback)


def publish_imu(imu):
    pub.publish(imu)


def odom_callback(odom_combined):
    print (odom_combined)


def pose_listener():
    rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, odom_callback)


def gps_listener():
    rospy.Subscriber("/gps/fix", NavSatFix)


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('/imu_data', Imu)
    rospy.init_node('ekf')
    pose_listener()
    imu_listener()
    while True:
        key = getKey()
        if (key == '\x03'):
            break
