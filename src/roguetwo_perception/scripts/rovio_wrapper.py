#!/usr/bin/env python
import sys, select, termios, tty

import cv2
import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image, PointCloud2, Imu
from nav_msgs.msg import Odometry  # for wheel and visual odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge, CvBridgeError


class RovioWrapper(object):
    """
    All this class does take the camera feeds, undistorts them, and then publishes
    to the ROVIO topis.
    """
    def __init__(self):
        self.camL_sub = rospy.Subscriber('/rogue_two/camera_stereo_left/image_raw', Image, self.imageL_rectify)
        self.camL_pub = rospy.Publisher('/cam0/image_raw', Image)
        self.bridge = CvBridge()

    def imageL_rectify(self, image_l):
        # rectify code will go here on the real robot
        image_l_cv = self.bridge.imgmsg_to_cv2(image_l, desired_encoding="rgb8")
        image_l_cv = cv2.cvtColor(image_l_cv, cv2.COLOR_RGB2GRAY)
        image_l = self.bridge.cv2_to_imgmsg(image_l_cv)
        self.camL_pub.publish(image_l)

    def imageR_rectify(self, image_r):
        # rectify code will go here on the real robot
        image_r_cv = self.bridge.imgmsg_to_cv2(image_r, desired_encoding="rgb8")
        image_r_cv = cv2.cvtColor(image_r_cv, cv2.COLOR_RGB2GRAY)
        image_r = self.bridge.cv2_to_imgmsg(image_r_cv)
        self.camR_pub.publish(image_r)

    def publish_imu(self, imu_data):
        self.imu_pub.publish(imu_data)


if __name__ == '__main__':
    rospy.init_node('rovio_wrapper')
    rovio_wrapper = RovioWrapper()
    rospy.spin()


