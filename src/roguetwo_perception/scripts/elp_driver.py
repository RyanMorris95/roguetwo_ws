#!/usr/bin/env python
import cv2
import rospy
import cv_bridge

from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header


CAMERA_WIDTH = 2560
CAMERA_HEIGHT = 960
FPS = 60

class ELPDriver(object):
    def __init__(self):
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.camera.set(cv2.CAP_PROP_FPS, FPS)

        self.bridge = cv_bridge.CvBridge()
        self.left_pub = rospy.Publisher('/left_image', Image, queue_size=1)
        self.rigth_pub = rospy.Publisher('/right_image', Image, queue_size=1)
        self.imu_pub = rospy.Publisher('/imu_sync', Imu, queue_size=1)
        rospy.Subscriber('/imu/data', Imu, self.publish_imu)

        self.timer = rospy.Timer(rospy.Duration(0.016667), self.split_frames)
        self.header = Header()

        self.imu_msg = None

    def publish_imu(self, imu_msg):
        # imu_msg.header = self.header
        # self.imu_pub.publish(imu_msg)
        self.imu_msg = imu_msg

    def split_frames(self, event):
        if not self.camera.grab():
            print ("No frames to grab.")

        _, frame = self.camera.retrieve()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        width = frame.shape[1]

        left_frame = frame[0:CAMERA_HEIGHT, 0:int(width/2)]
        right_frame = frame[0:CAMERA_HEIGHT, int(width/2):width]

        self.header.stamp = rospy.Time.now()
        left_frame_msg = self.bridge.cv2_to_imgmsg(left_frame, 'mono8')
        left_frame_msg.header = self.header

        right_frame_msg = self.bridge.cv2_to_imgmsg(right_frame, 'mono8')
        right_frame_msg.header = self.header

        self.imu_msg.header = self.header

        self.imu_pub.publish(self.imu_msg)
        self.left_pub.publish(left_frame_msg)
        self.rigth_pub.publish(right_frame_msg)


if __name__ == "__main__":
    rospy.init_node('elp_driver')
    node = ELPDriver()
    rospy.spin()

