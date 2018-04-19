#!/usr/bin/env python
import rospy
import PyKDL
import numpy as np 

from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import Quaternion

class RelativePose(object):
    def __init__(self):
        self.initial_pose = np.array([False])
        self.initial_x = 0
        self.initial_y = 0
        self.initial_quaternion = None
        
        self.relative_pub = rospy.Publisher("/odometry/relative", Odometry, queue_size=1)

        rospy.Subscriber("/odometry/filtered", Odometry, self.get_relative)

    def get_relative(self, odometry):
        pose = odometry.pose 
        x = pose.pose.position.x 
        y = pose.pose.position.y 
        
        orientation = pose.pose.orientation
        quaternion = PyKDL.Rotation.Quaternion(orientation.x, 
                                                orientation.y, 
                                                orientation.z, 
                                                orientation.w)
        if not self.initial_quaternion:
            self.initial_quaternion = quaternion
            self.initial_x = x
            self.initial_y = x
        else:
            relative_x = x - self.initial_x
            relative_y = y - self.initial_y

            relative_quaternion = self.initial_quaternion * quaternion.Inverse()
            relative_quaternion = relative_quaternion.GetQuaternion()

            odometry = Odometry()
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "odom"
            odometry.header = header
            odometry.child_frame_id = "base_link"
            odometry.pose.pose.position.x = relative_x
            odometry.pose.pose.position.y = relative_y

            orientation = Quaternion()
            orientation.x = relative_quaternion[0]
            orientation.y = relative_quaternion[1]
            orientation.z = relative_quaternion[2]
            orientation.w = relative_quaternion[3]
            odometry.pose.pose.orientation = orientation

            self.relative_pub.publish(odometry)


if __name__=="__main__":
    rospy.init_node("relative_pose")
    RelativePose()
    rospy.spin()