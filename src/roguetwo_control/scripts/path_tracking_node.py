#!/usr/bin/env python
from __future__ import division

import numpy as np
import rospy
import pure_pursuit
import PyKDL
import math
import stanley_controller

from roguetwo_navigation.msg import Path
from roguetwo_perception.msg import SE2
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry


class PathTrackingNode(object):
    pub_ackermann_cmd = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)

    def __init__(self):
        # setup subscribers
        rospy.Subscriber('/local_path', Path, self.update_local_path, queue_size=1)
        #rospy.Subscriber('/se2_state_filtered', SE2, self.update_se2, queue_size=1)
        #rospy.Subscriber('/velocity', Float32, self.update_velocity, queue_size=1)
        rospy.Subscriber('/encoder/odometry', Odometry, self.update_odometry, queue_size=1)
        #rospy.Subscriber('/odometry/filtered', Odometry, self.update_odometry, queue_size=1)

        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0
        self.t = 0
        self.target_index = 0
        self.target_speed = 0.5  # m/s
        self.state = pure_pursuit.State(x=0, y=0, yaw=0)
        self.current_se2 = None
        self.path = None
        self.dt = 0.1
        self.max_steering_angle = math.radians(30)

        self.path_tracking_timer = rospy.Timer(rospy.Duration(self.dt), self.path_tracking)

    def update_odometry(self, odometry):
        pose = odometry.pose 
        self.x = pose.pose.position.x 
        self.y = pose.pose.position.y 
        
        orientation = pose.pose.orientation
        quaternion = PyKDL.Rotation.Quaternion(orientation.x, 
                                                orientation.y, 
                                                orientation.z, 
                                                orientation.w)

        self.yaw = quaternion.GetRPY()[2]

        # convert x velocity to local robot velocity
        if self.yaw:
            self.v = odometry.twist.twist.linear.x / self.yaw

        self.state.x = self.x 
        self.state.y = self.y 
        self.state.yaw = self.yaw

    def update_velocity(self, velocity):
        v = velocity.data

    def update_se2(self, se2):
        """
        Update the se2 state of the robot whenever possible
        :return:
        """
        self.current_se2 = [se2.x, se2.y, se2.yaw]
        self.state.x = se2.x
        self.state.y = se2.y
        self.state.yaw = se2.yaw

    def update_local_path(self, path):
        self.path = path

    def stop_vehicle(self):
        msg = AckermannDrive()
        msg.speed = 0
        msg.acceleration = 0
        msg.jerk = 0
        msg.steering_angle = 0
        msg.steering_angle_velocity = 0
        self.pub_ackermann_cmd.publish(msg)
        rospy.signal_shutdown('Made it to goal')

    def path_tracking(self, event):
        if self.path and self.state:
            if self.path.x_states[0] == -100:  # shutdown procedure
                self.stop_vehicle()
            else:
                cx = self.path.x_states
                cy = self.path.y_states
                cyaw = self.path.yaw_states
                #self.target_index = pure_pursuit.calculate_target_index(self.state, cx, cy)
                target_index, mind = stanley_controller.calc_target_index(self.state, cx, cy)

                ai = pure_pursuit.pid_control(self.target_speed, self.state.v)
                #di, self.target_index = pure_pursuit.pure_pursuit_control(self.state,
                #                                                          cx,
                #                                                          cy,
                #                                                          self.target_index)
                di, target_index = stanley_controller.stanley_control(self.state, cx, cy, cyaw, target_index)
                
                print (self.state.v, di)
                self.state.v = self.state.v + ai * self.dt
                if di > self.max_steering_angle:
                    di = self.max_steering_angle
                if di < -self.max_steering_angle:
                    di = -self.max_steering_angle

                if self.state.v > self.target_speed:
                    self.state.v = self.target_speed

                rospy.loginfo("path_tracking_node: v: " + str(self.state.v) + " steering: " + str(di))
                msg = AckermannDrive()
                msg.speed = self.state.v
                msg.acceleration = 0.5
                msg.jerk = 1.0
                msg.steering_angle = di
                msg.steering_angle_velocity = 1

                self.pub_ackermann_cmd.publish(msg)


if __name__ == '__main__':
    rospy.init_node("path_tracking_node")
    node = PathTrackingNode()
    while not rospy.is_shutdown():
        pass
