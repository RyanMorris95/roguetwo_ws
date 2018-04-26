#!/usr/bin/env python

import rospy
import numpy as np 
import math
import PyKDL
import copy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range 


class Lidar(object):
    def __init__(self, height=0.0):
        self.dist = 0
        self.prev_dist = 0
        self.height = height


class FilterLidar(object):
    def __init__(self):
        self.fl_lidar = Lidar(height=0.17)
        self.fr_lidar = Lidar(height=0.17)
        self.bl_lidar = Lidar(height=0.17)
        self.br_lidar = Lidar(height=0.17)

        self.curr_pitch = 0

        self.fl_pub = rospy.Publisher("/lidar_front_left_filtered", Range, queue_size=1)
        self.fr_pub = rospy.Publisher("/lidar_front_right_filtered", Range, queue_size=1)
        self.bl_pub = rospy.Publisher("/lidar_left_filtered", Range, queue_size=1)
        self.br_pub = rospy.Publisher("/lidar_right_filtered", Range, queue_size=1)

        #rospy.Subscriber("/lidar_front_left", Range, self.update_fl_lidar, queue_size=1)
        #rospy.Subscriber("/lidar_front_right", Range, self.update_fr_lidar, queue_size=1)
        rospy.Subscriber("/lidar_left", Range, self.update_bl_lidar, queue_size=1)
        #rospy.Subscriber("/lidar_right", Range, self.update_br_lidar, queue_size=1)
        rospy.Subscriber("/encoder/odometry", Odometry, self.update_odometry, queue_size=1)

    def hit_ground(self, lidar):
        y_dist = lidar.dist * math.sin(self.curr_pitch)
        if y_dist < lidar.height and y_dist != 0:
            print (y_dist)
            return False
        else:
            return True 

    def between(self, min_dist, max_dist, dist):
        if min_dist < dist < max_dist:
            return True
        else:
            return False

    def update_lidar(self, lidar, pub, range_msg):
        lidar.dist = range_msg.range
        if lidar.dist != 0 and lidar.dist ==lidar.prev_dist:
            return lidar

        if not self.between(0.5, 4, range_msg.range):
            return lidar

        if self.hit_ground(lidar):
            return lidar

        lidar.prev_dist = lidar.dist

        filt_range_msg = Range()
        filt_range_msg.range = lidar.dist
        pub.publish(filt_range_msg)
        print ("Publishing: ", range_msg.range)
        return lidar
        
    def update_fl_lidar(self, range_msg):
        self.fl_lidar = self.update_lidar(self.fl_lidar, self.fl_pub, range_msg)
        
    def update_fr_lidar(self, range_msg):
        self.fr_lidar = self.update_lidar(self.fr_lidar, self.fr_pub, range_msg)

    def update_bl_lidar(self, range_msg):
        self.bl_lidar = self.update_lidar(self.bl_lidar, self.bl_pub, range_msg)

    def update_br_lidar(self, range_msg):
        self.br_lidar = self.update_lidar(self.br_lidar, self.br_pub, range_msg)

    def update_odometry(self, odometry_msg):
        pose = odometry_msg.pose 
        
        orientation = pose.pose.orientation
        quaternion = PyKDL.Rotation.Quaternion(orientation.x, 
                                                orientation.y, 
                                                orientation.z, 
                                                orientation.w)

        self.curr_pitch = quaternion.GetRPY()[1]


if __name__=="__main__":
    rospy.init_node("filter_lidar")
    node = FilterLidar()
    rospy.spin()
        