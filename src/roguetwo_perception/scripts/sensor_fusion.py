#!/usr/bin/env python
import rospy
import numpy as np

from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import TwistStamped, Vector3
from std_msgs.msg import Header, Float32
from nav_msgs.msg import Odometry
from roguetwo_perception.msg import SE2
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

class Filter():
    def __init__(self, dt=0.01, imu_var=0.001, gps_var=1.58):
        # tracking position and velocity, provided by GPS
        self.tracker = KalmanFilter(dim_x=2, dim_z=2, dim_u=1)
        # state transition matrix
        self.tracker.F = np.array([[1, dt],
                                [0, 1]])

        # process variance matrix
        self.tracker.Q = Q_discrete_white_noise(dim=2, dt=dt, var=imu_var)

        # control matrix --> imu provides accleration data
        self.tracker.B = np.array([[0.5*dt**2],
                                    [dt]])

        # measurement matrix
        self.tracker.H = np.array([[1, 0], [0, 0]])

        # measurement noise matrix
        self.tracker.R = np.array([[gps_var**2, 0],
                                [0, gps_var**2]])

        # initial condition
        self.tracker.x = np.array([0.0, 0.0]).T

        # covariance matrix
        self.tracker.P = np.eye(2) * 0.1

    
class SensorFusion(object):
    def __init__(self):
        # setup kalman filters for latitude and longitude
        self.x_kalman = Filter()
        self.y_kalman = Filter()
        self.curr_x = 0
        self.curr_y = 0
        self.curr_x_vel = 0
        self.curr_y_vel = 0
        self.curr_yaw = 0

        rospy.Subscriber("/se2_state", SE2, self.update_positions)
        rospy.Subscriber("/imu", Imu, self.update_accelerations)
        # rospy.Subscriber("/gps/fix_velocity", TwistStamped, self.update_velocities) 

        self.se2_filtered_pub = rospy.Publisher("/se2_state_filtered", SE2, queue_size=1)

    def publish_filtered_se2(self):
        (x, x_vel) = self.x_kalman.tracker.x
        (y, y_vel) = self.y_kalman.tracker.y
        se2_msg = SE2()
        se2_msg.x = x
        se2_msg.y = y
        se2_msg.yaw = self.curr_yaw

    def update_accelerations(self, imu_msg):
        linear_acceleration = imu_msg.linear_acceleration
        x_accel = linear_acceleration.x 
        y_accel = linear_acceleration.y
        
        self.x_kalman.tracker.predict(u=np.array([[x_accel, x_accel]]))
        self.y_kalman.tracker.predict(u=np.array([[y_accel, y_accel]]))

    def update_positions(self, se2_msg):
        self.curr_x = se2_msg.x
        self.curr_y = se2_msg.y

        x_measurement = np.array([[self.curr_x, 0]]).T
        print (x_measurement.shape)
        self.x_kalman.tracker.update(x_measurement)

        y_measurement = np.array([[self.curr_y, 0]]).T

        self.y_kalman.tracker.update(y_measurement)
        self.curr_yaw = se2_msg.yaw

    def update_velocities(self, twist_msg):
        self.curr_lat_vel = twist_msg.vector.x 
        self.curr_lon_vel = twist_msg.vector.y


if __name__ == "__main__":
    rospy.init_node("sensor_fusion")
    node = SensorFusion()
    rospy.spin()
            
                                