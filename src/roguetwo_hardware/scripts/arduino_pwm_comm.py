#!/usr/bin/env python
from __future__ import division 

import rospy
import serial

from sklearn import preprocessing
from ackermann_msgs.msg import AckermannDrive


class ArduinoPWMComm(object):
	def __init__(self):
		rospy.Subscriber('/ackermann_cmd', AckermannDrive, self.send_pwm, queue_size=1)
		#self.ser = serial.Serial('/dev/ttyUSB0', 9600)
		self.min_pwm_steering = 90
		self.max_pwm_steering = 180
		self.min_pwm_motor = 0
		self.max_pwm_motor = 255

		self.min_speed = -0.5
		self.max_speed = 0.5
		self.min_steering = -0.25
		self.max_steering = 0.25

	def convert_speed_to_pwm(self, speed):
		speed_pwm = speed + abs(self.min_speed)
		speed_pwm *= self.max_pwm_motor
		return speed_pwm

	def convert_steering_to_pwm(self, steering):
		steering_pwm = steering + self.min_steering + (self.min_steering * 2)
		steering_pwm *= self.max_pwm_steering
		return abs(steering_pwm)

	def send_pwm(self, ackermann_msg):
		speed = ackermann_msg.speed
		steering_angle = ackermann_msg.steering_angle

		speed_pwm = self.convert_speed_to_pwm(speed)
		steering_pwm = self.convert_steering_to_pwm(steering_angle)

		pwm_message = str(speed_pwm) + " " + str(steering_pwm)
		print (pwm_message)

		#self.ser.write(pwm_message)


if __name__ == "__main__":
    rospy.init_node("arduino_pwm_comm")
    node = ArduinoPWMComm()
    rospy.spin()