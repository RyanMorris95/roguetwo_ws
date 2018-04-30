#!/usr/bin/env python
import serial
import time
import rospy
import sys, select, termios, tty
import serial
import numpy as np
import argparse

from sensor_msgs.msg import Range

parser = argparse.ArgumentParser()
parser.add_argument("node_name")
parser.add_argument('-s', '--serial_device', default="")
parser.add_argument('-p', '--publisher_name', default="")

class Lidar(object):
    def __init__(self, ser_device, pub):
        self.ser = serial.Serial(ser_device,115200,timeout = 1)
        self.distance = 0
        self.strength = 0
        self.distance_pub = rospy.Publisher(pub, Range, queue_size=1)
        rospy.Timer(rospy.Duration(0.01), self.run_lidar)

    def __del__(self):
        self.ser.close()

    def setup(self):
        self.ser.write(bytes(b'B'))
        self.ser.write(bytes(b'W'))
        self.ser.write(bytes(2))
        self.ser.write(bytes(0))
        self.ser.write(bytes(0))
        self.ser.write(bytes(0)) 
        self.ser.write(bytes(1))  
        self.ser.write(bytes(6))

    def run_lidar(self, event):
        if self.ser.in_waiting >= 9:
            if (b'Y' == self.ser.read()) and ( b'Y' == self.ser.read()):
                dist_l = self.ser.read()
                dist_h = self.ser.read()
                self.distance = (ord(dist_h) * 256) + (ord(dist_l))
                strength_l = self.ser.read()
                strength_h = self.ser.read()
                self.strength = (ord(strength_h) * 256) + (ord(strength_l))
                for i in range (0, 3):
                    self.ser.read()
            
            distance_msg = Range()
            distance_msg.range = float(self.distance) / 100
            distance_msg.min_range = 0.30
            distance_msg.max_range = 7.0
            self.distance_pub.publish(distance_msg)

            self.ser.flush()

if __name__=="__main__":
    args = parser.parse_args()

    device_dict = {"lidar1": "/dev/ttyUSB0",
                "lidar2": "/dev/ttyUSB1",
                "lidar3": "/dev/ttyUSB2",
                "lidar4": "/dev/ttyUSB3"}
    
    publisher_dict = {"lidar1": "/lidar_front_left",
                    "lidar2": "/lidar_left",
                    "lidar3": "/lidar_right",
                    "lidar4": "/lidar_front_right"}

    rospy.init_node(args.node_name)

    if args.serial_device:
        serial_device = args.serial_device
    else:
        serial_device = device_dict[args.node_name]

    if args.publisher_name:
        publisher_name = args.publisher_name
    else:
        publisher_name = publisher_dict[args.node_name]

    print (args.node_name, serial_device, publisher_name)
    node = Lidar(serial_device, publisher_name)
    rospy.spin()
