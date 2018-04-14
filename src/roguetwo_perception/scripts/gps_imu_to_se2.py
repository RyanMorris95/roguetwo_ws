#!/usr/bin/env python

import rospy
import geonav_conversions
import PyKDL
import numpy as np

from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import Quaternion
from roguetwo_perception.msg import SE2
from tf.transformations import euler_from_quaternion, quaternion_from_euler



class ToSE2(object):
    """
    This class takes the GPS measurements and converts it to 
    meters traveled.  Then it takes the yaw from the imu.
    Then the two are combined to for an SE2 measurement.
    """
    def __init__(self):
        rospy.Subscriber("/gps", NavSatFix, self.update_position, queue_size=1)
        rospy.Subscriber("/imu/data", Imu, self.update_yaw)
        rospy.Subscriber("/gps/fix", NavSatFix, self.update_position, queue_size=1)
        rospy.Subscriber("/imu", Imu, self.update_yaw)

        self.se2_pub = rospy.Publisher("/se2_state", SE2, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_se2)

        self.origin_lat = None
        self.origin_lon = None
        
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.num_gps_readings = 10
        self.gps_list = []

        self.okay_to_publish = False

    def update_position(self, navsat_msg):
        if len(self.gps_list) > self.num_gps_readings:
            # if not self.origin_lat and not self.origin_lon:
            #     self.origin_lat = navsat_msg.latitude
            #     self.origin_lon = navsat_msg.longitude

            current_lat = navsat_msg.latitude
            current_lon = navsat_msg.longitude

            (self.current_x, self.current_y) = geonav_conversions.ll2xy(current_lat, 
                                                                        current_lon, 
                                                                        self.origin_lat, 
                                                                        self.origin_lon)
        elif len(self.gps_list) < self.num_gps_readings:
            rospy.loginfo('gps_imu_to_se2: Have gotten ' + str(len(self.gps_list)) + ' gps reading')
            self.gps_list.append([navsat_msg.latitude, navsat_msg.longitude])
        else:
            self.gps_list.append([navsat_msg.latitude, navsat_msg.longitude])
            gps_array = np.array(self.gps_list)
            gps_mean = np.mean(gps_array, axis=0)
            print (gps_mean)

            self.origin_lat = gps_mean[0]
            self.origin_lon = gps_mean[1]
            self.okay_to_publish = True


    def update_yaw(self, imu_msg):
        # gemoetry quaternion message
        orientation = imu_msg.orientation

        # convert quaternion message to PyKDL quaternion
        quaternion = PyKDL.Rotation.Quaternion(orientation.x, 
                                                orientation.y, 
                                                orientation.z, 
                                                orientation.w)

        self.current_yaw = quaternion.GetRPY()[2]
        #self.current_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        #self.current_yaw = orientation.z

    def publish_se2(self, event):
        if self.current_x and self.current_y and self.current_yaw and self.okay_to_publish:
            se2 = SE2()
            se2.x = self.current_x
            se2.y = self.current_y
            se2.yaw = self.current_yaw

            self.se2_pub.publish(se2)


if __name__ == '__main__':
    rospy.init_node("gps_imu_to_se2")
    node = ToSE2()
    rospy.spin()
