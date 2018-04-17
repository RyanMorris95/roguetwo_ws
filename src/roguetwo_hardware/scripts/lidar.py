#!/usr/bin/env python
import serial
import time
import RPi.GPIO as GPIO

import rospy
import sys, select, termios, tty

from sensor_msgs.msg import Range
import serial
import numpy as np

LEDpin = 11

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(LEDpin,GPIO.OUT)
GPIO.output(LEDpin,GPIO.LOW)

ser = serial.Serial('/dev/ttyAMA0',115200,timeout = 1)

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

Dist_Total = 0
Dist_L = 0
Dist_H = 0

distance = Range()

pub = rospy.Publisher('sonar_frontR_distance', Range, queue_size = 10)
rospy.init_node('Lidar', anonymous = True)

while(True and not rospy.is_shutdown()):
    while(ser.in_waiting >= 9 and not rospy.is_shutdown()):
        #print (ser.read())
        #time.sleep(0.1)

        if((b'Y' == ser.read()) and ( b'Y' == ser.read())):
            
            GPIO.output(LEDpin, GPIO.LOW)
            Dist_L = ser.read()
            Dist_H = ser.read()
            Dist_Total = (ord(Dist_H) * 256) + (ord(Dist_L))
            for i in range (0,5):
                ser.read()
        
        distance.range = float(Dist_Total)        
        print(Dist_Total)
	
		#range_msg.range = float(Dist_Total)
        pub.publish(distance)

        ser.flush()
        if(Dist_Total < 20):
            GPIO.output(LEDpin, GPIO.HIGH)
