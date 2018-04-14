#!/usr/bin/env python

from __future__ import division

import rospy
import serial
import math
import time
import Adafruit_PCA9685

from sklearn import preprocessing
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32


class ArduinoComm(object):
    def __init__(self):
        rospy.Subscriber('/ackermann_cmd', AckermannDrive, self.send_pwm, queue_size=1)
        rospy.Subscriber('/max_motor_vel', Float32, self.update_speed_bounds, queue_size=1)

				# self.ser = serial.Serial('/dev/ttyUSB0', 9600)
        self.min_speed = -0.5
        self.max_speed = 0.5
        self.min_steering = math.radians(-45)
        self.max_steering = math.radians(45)

        self.min_pwm_steering = 90
        self.max_pwm_steering = 180
        self.min_pwm_motor = -80
        self.max_pwm_motor = 80

        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)

    def update_speed_bounds(self, speed_msg):
        new_speed_bound = speed_msg.data
        diff = self.max_speed - new_speed_bound
        self.min_speed = -1 * new_speed_bound
        self.max_speed = 1 * new_speed_bound

        self.min_pwm_motor -= diff * 10
        self.max_pwm_motor += diff * 10
        

    def remap(self, x, oMin, oMax, nMin, nMax):
        # range check
        if oMin == oMax:
            print "Warning: Zero input range"
            return None

        if nMin == nMax:
            print "Warning: Zero output range"
            return None

        # check reversed input range
        reverseInput = False
        oldMin = min(oMin, oMax)
        oldMax = max(oMin, oMax)
        if not oldMin == oMin:
            reverseInput = True

        # check reversed output range
        reverseOutput = False
        newMin = min(nMin, nMax)
        newMax = max(nMin, nMax)
        if not newMin == nMin:
            reverseOutput = True

        portion = (x - oldMin) * (newMax - newMin) / (oldMax - oldMin)
        if reverseInput:
            portion = (oldMax - x) * (newMax - newMin) / (oldMax - oldMin)

        result = portion + newMin
        if reverseOutput:
            result = newMax - portion

        return result

    def convert_speed_to_pwm(self, speed):
        speed_pwm = self.remap(speed,
                               self.min_speed,
                               self.max_speed,
                               self.min_pwm_motor,
                               self.max_pwm_motor)
        return speed_pwm

    def convert_steering_to_pwm(self, steering):
        steering_pwm = self.remap(steering,
                                  self.min_steering,
                                  self.max_steering,
                                  self.min_pwm_steering,
                                  self.max_pwm_steering)
        return steering_pwm

    def send_pwm(self, ackermann_msg):
        speed = ackermann_msg.speed
        steering_angle = ackermann_msg.steering_angle

        speed_pwm = self.convert_speed_to_pwm(speed)
        steering_pwm = self.convert_steering_to_pwm(steering_angle)

        if abs(speed_pwm) > 20:
            pwm_message = str(speed_pwm) + " " + str(steering_pwm) + "\n"
            self.set_motor_pulse(speed_pwm)
            self.set_steering_pulse(steering_pwm)
            rospy.loginfo_throttle(30, "Motor_Comm: " + pwm_message)
        else:
            self.set_motor_pulse(0)
            self.set_steering_pulse(0)
            rospy.loginfo_throttle(60, "Motor_Comm: Need to apply pwm to steer.")

    def set_steering_pulse(self, pwm_command):
        self.pwm.set_pwm(0, 0, pwm_command)

    def set_motor_pulse(self, pwm_command):
        self.pwm.set_pwm(1, 0, pwm_command)

if __name__ == "__main__":
    rospy.init_node('motor_servo_comm')
    node = ArduinoComm()
    rospy.spin()

