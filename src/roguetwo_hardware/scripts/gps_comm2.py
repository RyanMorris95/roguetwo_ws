#!/usr/bin/env python
import rospy 

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from gps import *


class GPSComm(object):
    def __init__(self):
        self.navsat_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=1)
        self.gpsd = gps(mode=WATCH_ENABLE)  # starting the gps stream of info
        self.gps_update = rospy.Timer(rospy.Duration(0.1), self.run)

    def run(self, event):
        self.gpsd.next()


    def publish_gps(self):
        navsat = NavSatFix()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_footprint"
        navsat.header = header
        navsat_status = NavSatStatus()
        navsat_status.status = 0
        navsat_status.service = 1
        navsat.status = navsat_status
        navsat.latitude = self.gpsd.fix.latitude
        navsat.longitude = self.gpsd.fix.longitude
        navsat.altitude = self.gpsd.fix.altitude
        navsat.position_covariance_type = 2
        navsat.position_covariance = [2.5, 0, 0,
                                    0, 2.5, 0,
                                    0, 0, 2.5]

        self.navsat_pub.publish(navsat)


if __name__ == '__main__':
    rospy.init_node("gps_comm")
    gps_comm = GPSComm()
    rospy.spin()