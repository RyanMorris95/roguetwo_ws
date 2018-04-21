#!/usr/bin/env python
from __future__ import division

import rospy
import serial
import math
import time
import Adafruit_PCA9685
import RPi.GPIO as GPIO
import PyKDL

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry


class MotorServoComm(object):
    def __init__(self, direction_pin=21, motor_pin=0, steering_pin=0):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(direction_pin, GPIO.OUT)

        rospy.Subscriber('/ackermann_cmd', AckermannDrive, self.send_pwm, queue_size=1)
        rospy.Subscriber('/max_motor_vel', Float32, self.update_speed_bounds, queue_size=1)
        #rospy.Subscriber('/encoder/velocity', Float32, self.update_velocity, queue_size=1)
        rospy.Subscriber('/encoder/odometry', Odometry, self.update_velocity, queue_size=1)

        self.min_speed = 0.0
        self.max_speed = 0.5
        self.min_steering = math.radians(-30)
        self.max_steering = math.radians(30)

        self.no_steering_pwm = 400
        self.min_pwm_steering = 225  # goes 30 degrees left
        self.max_pwm_steering = 700  # goes 30 degrees right

        self.min_pwm_motor = 0
        self.max_pwm_motor = 2000
        #self.min_pwm_motor = -2400  # correct
        #self.max_pwm_motor = 2400  # correct

        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)

        self.prev_steer = 0
        self.prev_motor = 0
        self.direction_pin = direction_pin
        self.motor_pin = motor_pin
        self.steering_pin = steering_pin

        # PID parameters
        self.velocity = 0  # current velocity of robot, m/s
        self.kP = 500.0
        self.kI = 0.0
        self.kD = 0.0
        self.last_error = 0.0
        self.integral_sum = 0.0
        self.yaw = 0
        self.prev_time = 0

    def pwm_pid(self, target_velocity):
        curr_time = time.time()
        dt = curr_time - self.prev_time
        self.prev_time = curr_time

        error = target_velocity - self.velocity
        p = self.kP * error
        i = self.integral_sum + self.kI * error * dt
        d = self.kD * (error - self.last_error) / dt

        self.last_error = error
        self.integral_sum = i

        new_pwm = p + i + d 
        if new_pwm > self.max_pwm_motor:
            new_pwm = self.max_pwm_motor
        elif new_pwm < self.min_pwm_motor:
            new_pwm = self.min_pwm_motor

        return new_pwm

    def update_velocity(self, odometry_msg):
        # geometry quaternion message
        orientation = odometry_msg.pose.pose.orientation

        # convert quaternion message to PyKDL quaternion
        quaternion = PyKDL.Rotation.Quaternion(orientation.x, 
                                                orientation.y, 
                                                orientation.z, 
                                                orientation.w)

        self.yaw = quaternion.GetRPY()[2]

        self.velocity = odometry_msg.twist.twist.linear.x / math.cos(self.yaw)


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
        steering_angle *= -1
        #speed_pwm = int(self.pwm_pid(speed))
        speed_pwm = int(self.convert_speed_to_pwm(abs(speed)))
        steering_pwm = int(self.convert_steering_to_pwm(steering_angle))

        if True:
            pwm_message = str(speed_pwm) + " " + str(steering_pwm) + "\n"
            self.set_motor_pulse(speed_pwm)
            if (steering_angle != 0 and speed_pwm > 1000):
                self.set_steering_pulse(steering_pwm)
            else:
                self.set_steering_pulse(self.no_steering_pwm)
            if speed < 0:
                GPIO.output(self.direction_pin, 0)
            else:
                GPIO.output(self.direction_pin, 1)

            rospy.loginfo_throttle(30, "Motor_Comm: " + pwm_message)
        else:
            self.set_motor_pulse(0)
            self.set_steering_pulse(self.no_steering_pwm)
            rospy.loginfo_throttle(60, "Motor_Comm: Need to apply pwm to steer.")

    def set_steering_pulse(self, pwm_command):
        self.pwm.set_pwm(self.steering_pin, 0, int(pwm_command))
        self.prev_steer = pwm_command

    def set_motor_pulse(self, pwm_command):
        self.pwm.set_pwm(self.motor_pin, 0, int(pwm_command))

if __name__ == "__main__":
    rospy.init_node('motor_servo_comm')
    node = MotorServoComm(direction_pin=21, 
                        motor_pin=4, 
                        steering_pin=0)
    rospy.spin()

