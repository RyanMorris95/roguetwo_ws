#!/usr/bin/env python
import argparse
import os

parser = argparse.ArgumentParser(description='Set IP')
parser.add_argument('--ip', dest='ip')

args = parser.parse_args()

print ('Setting ROS IP addresses to: ' + args.ip)
ip = args.ip

ros_ip_command = 'export ROS_IP=' + ip
ros_hostname_command = 'export ROS_HOSTNAME=' + ip
ros_master_uri_command = 'export ROS_MASTER_URI=http://' + ip + ':11311'
os.system(ros_ip_command)
os.system(ros_hostname_command)
os.system(ros_master_uri_command)
print ('Ran: '+ ros_ip_command)
print ('Ran: '+ ros_hostname_command)
print ('Ran: '+ ros_master_uri_command)