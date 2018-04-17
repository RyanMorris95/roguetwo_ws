#!/usr/bin/env python
import rospy
import numpy as np 
import time 
import RPi.GPIO as GPIO
import PyKDL
import math

from std_msgs.msg import Float32, Header
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist



class RotaryEncoder(object):
    def __init__(self, m_per_revolution=0.05, pin=16, rate=10, debug=False):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN)
        GPIO.add_event_detect(pin, GPIO.FALLING, callback=self.one_revolution)

        # initialize the odometry values
        self.m_per_revolution = m_per_revolution
        self.meters = 0  # robots local distance traveled
        self.meters_per_second = 0
        self.counter = 0
        self.debug = debug
        self.rate = rate
        self.last_time = 0
        
        # robot position on a catesian global grid
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.orientation = 0
        self.prev_yaw = 0

        # initialize ros interface 
        self.distance_pub = rospy.Publisher("/encoder/distance", Float32, queue_size=1)
        self.speed_pub = rospy.Publisher("/encoder/speed", Float32, queue_size=1)
        self.odometry_pub = rospy.Publisher("/encoder/odometry", Odometry, queue_size=1)
        rospy.Subscriber("/imu/data", Imu, self.update_yaw)
        rospy.Subscriber("/imu", Imu, self.update_yaw)

    def __del__(self):
        rospy.loginfo("encoder: shutting down encoder node.")
        self.shutdown()
    
    def debug_test(self):
        while True:
            print (GPIO.input(pin))
   

    def update_yaw(self, imu_msg):
        # geometry quaternion message
        self.orientation = imu_msg.orientation

        # convert quaternion message to PyKDL quaternion
        quaternion = PyKDL.Rotation.Quaternion(orientation.x, 
                                                orientation.y, 
                                                orientation.z, 
                                                orientation.w)

        self.yaw = quaternion.GetRPY()[2]

    def one_revolution(self, channel):
        self.counter += 1

    def calibrate(self):
        while not rospy.is_shutdown():
            print (self.counter)

    def update(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # save the ticks and the counter
            revolutions = self.counter
            self.counter = 0

            # save off the last time interval and reset the timer
            start_time = self.last_time
            end_time = time.time()
            self.last_time = end_time

            # calculate elapsed time and distance traveled
            seconds = end_time - start_time
            distance = revolutions * self.m_per_revolution
            velocity = distance / seconds

            # update the odometry values
            self.meters += distance 
            self.meters_per_second = velocity

            # update the robots global position on the grid
            self.x += distance * math.cos(self.yaw)
            self.y += distance * math.sin(self.yaw)

            # calculate velocity 
            x_velocity = velocity * math.cos(self.yaw)
            y_velocity = velocity * math.sin(self.yaw)
            yaw_velocity = (self.prev_yaw - self.yaw) * seconds
            self.prev_yaw = yaw

            # publish the data
            msg = Float32()
            msg.data = self.meters
            self.distance_pub.publish(msg)
            msg.data = self.meters_per_second
            self.speed_pub.publish(msg)

            odometry = Odometry()
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "odom"
            odometry.header = header
            odometry.child_frame_id = "base_link"
            odometry.pose.pose.position.x = self.x
            odometry.pose.pose.position.y = self.y
            odometry.pose.pose.orientation = self.orientation

            twist = Twist()
            twist.linear.x = x_velocity
            twist.linear.y = y_velocity
            odometry.twist = twist

            self.odometry_pub.publish(odometry) 
            
            if self.debug:
                rospy.loginfo("encoder: seconds: " + str(seconds))
                rospy.loginfo("encoder: distance: " + str(distance))
                rospy.loginfo("encoder: velocity: " + str(velocity))
                rospy.loginfo("encoder: distance (m): " + str(round(self.meters, 4)))

            rate.sleep()

        self.shutdown()

    def shutdown(self):
        GPIO.cleanup()


if __name__ == '__main__':
    rospy.init_node("encoder")
    node = RotaryEncoder(m_per_revolution=0.05, 
                        pin=16, 
                        rate=10,
                        debug=True)
    #node.debug_test()
    node.update()
    #node.calibrate()
