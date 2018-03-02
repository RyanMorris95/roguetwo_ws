#!/usr/bin/env python
import sys, select, termios, tty

import cv2
import rospy
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
        self.camL_sub = rospy.Subscriber('/stereo/camera/left/image_raw', Image, self.imageL_rectify)
        self.camR_sub = rospy.Subscriber('/stereo/camera/right/image_raw', Image, self.imageR_rectify)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.publish_imu)
        #self.odometry_sub = rospy.Subscriber('/rovio/odometry', Odometry, )
        #self.pcl_sub = rospy.Subscriber('/rovio/pcl', PointCloud2, )
        #self.pose_sub = rospy.Subscriber('/rovio/pose_with_covariance_stamped', PoseWithCovarianceStamped, )
        self.camL_pub = rospy.Publisher('/cam0/image_raw', Image)
        self.camR_pub = rospy.Publisher('/cam1/image_raw', Image)
        self.imu_pub = rospy.Publisher('/imu0', Imu)
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


