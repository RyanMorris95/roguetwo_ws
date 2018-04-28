#!/usr/bin/env python

import rospy
import numpy as np 
import math
import PyKDL
import copy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range 
from collections import deque


class Lidar(object):
    def __init__(self, height=0.0, pitch=0.0):
        self.dist = 0
        self.prev_dist = 0
        self.height = height
        self.pitch = 0
        self.lidar_queue = deque(maxlen=10)
        self.avg_dist = 0

    def reset(self):
        self.dist = 0
        self.prev_dist = 0

class FilterLidar(object):
    def __init__(self):
        self.max_dist = 2.5
        self.min_dist = 0.5

        self.fl_lidar = Lidar(height=0.15)
        self.fr_lidar = Lidar(height=0.15)
        self.bl_lidar = Lidar(height=0.10, pitch=-10)
        self.br_lidar = Lidar(height=0.10, pitch=-10)

        self.curr_pitch = 0

        self.fl_pub = rospy.Publisher("/lidar_front_left_filtered", Range, queue_size=1)
        self.fr_pub = rospy.Publisher("/lidar_front_right_filtered", Range, queue_size=1)
        self.bl_pub = rospy.Publisher("/lidar_left_filtered", Range, queue_size=1)
        self.br_pub = rospy.Publisher("/lidar_right_filtered", Range, queue_size=1)

        rospy.Subscriber("/lidar_front_left", Range, self.update_fl_lidar, queue_size=1)
        #rospy.Subscriber("/lidar_front_right", Range, self.update_fr_lidar, queue_size=1)
        #rospy.Subscriber("/lidar_left", Range, self.update_bl_lidar, queue_size=1)
        #rospy.Subscriber("/lidar_right", Range, self.update_br_lidar, queue_size=1)
        rospy.Subscriber("/encoder/odometry", Odometry, self.update_odometry, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.publish_lidar)

    def hit_ground(self, lidar):
        y_dist = lidar.dist * math.sin(self.curr_pitch + math.radians(lidar.pitch))
        print ("y_dist traveled: ", y_dist)
        if y_dist < lidar.height and y_dist != 0:
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
            lidar.reset()
            return lidar

        if not self.between(self.min_dist, self.max_dist, range_msg.range):
            lidar.reset()
            return lidar

        if self.hit_ground(lidar):
            lidar.reset()
            return lidar

        lidar.prev_dist = lidar.dist
        lidar.lidar_queue.append = lidar.dist 

        return lidar

    def publish_lidar(self, event):
        filt_range_msg = Range()
        filt_range_msg.range = self.fl_lidar.dist
        self.fl_pub.publish(filt_range_msg)   
        filt_range_msg.range = self.fr_lidar.dist
        self.fr_pub.publish(filt_range_msg)
        filt_range_msg.range = self.bl_lidar.dist
        self.bl_pub.publish(filt_range_msg)
        filt_range_msg.range = self.br_lidar.dist
        self.br_pub.publish(filt_range_msg)      

        self.fl_lidar.reset()
        self.fr_lidar.reset()
        self.bl_lidar.reset()
        self.br_lidar.reset()
        
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
        