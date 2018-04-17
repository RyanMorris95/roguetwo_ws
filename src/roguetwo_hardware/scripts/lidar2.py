#!/usr/bin/env python
import serial
import time
import RPi.GPIO as GPIO
import rospy
import sys, select, termios, tty
import serial
import numpy as np

from sensor_msgs.msg import Range


class Lidar(object):
    def __init__(self, pin):
        LEDpin = 11

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(LEDpin,GPIO.OUT)
        GPIO.output(LEDpin,GPIO.LOW)

        ser = serial.Serial('/dev/ttyAMA0',115200,timeout = 1)
        Dist_Total = 0
        Dist_L = 0
        Dist_H = 0

        distance = Range()

        timer = rospy.Timer(rospy.Duration(0.1), self.run_lidar)

    def setup(self):
        #ser.write(0x42)
        ser.write(bytes(b'B'))

        #ser.write(0x57)
        ser.write(bytes(b'W'))

        #ser.write(0x02)
        ser.write(bytes(2))

        #ser.write(0x00)
        ser.write(bytes(0))

        #ser.write(0x00)
        ser.write(bytes(0))

        #ser.write(0x00)
        ser.write(bytes(0))
                
        #ser.write(0x01)
        ser.write(bytes(1))
                
        #ser.write(0x06)
        ser.write(bytes(6))
