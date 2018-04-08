#!/usr/bin/env python

import rospy
import numpy as np
import math
import tf2_ros
import PyKDL

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header, Float64MultiArray
from tf.transformations import euler_from_quaternion
from roguetwo_perception.msg import SE2


class SVOWrapper(object):
    """
    SVO publishes the camera pose in z-forward x-right y-down when
    ros needs x-forward y-left z-down.  This class provides
    the transformation for that to happen.

    Also the class publishes the SE2 State space (x, y, yaw) of the
    vehicle.
    """
    # set publishers
    pub_odom = rospy.Publisher('/cam_pose', PoseStamped, queue_size=1)
    pub_se2 = rospy.Publisher('/se2_state', SE2, queue_size=1)

    def __init__(self, use_imu):
        # set subscribers
        rospy.Subscriber('/svo/pose_cam/0', PoseStamped, self.svo_pose_callback)

        self.initial_rotation = None
        self.use_imu = use_imu

    def svo_pose_callback(self, svo_pose):
        # convert position to standard ROS XYZ
        ros_format_pose = PoseStamped()
        pose = svo_pose.pose
        position = pose.position
        orientation = pose.orientation

        if True:
            ros_format_pose.pose.position.x = position.z
            ros_format_pose.pose.position.y = -position.x
            ros_format_pose.pose.position.z = -position.y
        else:
            ros_format_pose.pose.position.x = position.x
            ros_format_pose.pose.position.y = position.y
            ros_format_pose.pose.position.z = position.z

        orientation = svo_pose.pose.orientation
        ros_format_pose.pose.orientation = orientation
        
        # build pose message
        header = Header()
        header.stamp = rospy.Time.now()
        ros_format_pose.header = header
        self.pub_odom.publish(ros_format_pose)

        if not self.initial_rotation:
            self.initial_rotation = PyKDL.Rotation.Quaternion(orientation.x, 
                                                            orientation.y, 
                                                            orientation.z, 
                                                            orientation.w)

        current_rotation = PyKDL.Rotation.Quaternion(orientation.x, 
                                                            orientation.y, 
                                                            orientation.z, 
                                                            orientation.w)

        relative_rotation = self.initial_rotation * current_rotation.Inverse()
        yaw = relative_rotation.GetRPY()[2]

        # # convert quaternion to euler to get vehicle yaw
        # quaternion_arr = np.array([orientation.x,
        #                            orientation.y,
        #                            orientation.z,
        #                            orientation.w])
        # euler = euler_from_quaternion(quaternion_arr)

        # convert the xyz to se2 state space for path planing
        se2_state = SE2()
        se2_state.x = ros_format_pose.pose.position.x
        se2_state.y = ros_format_pose.pose.position.y
        se2_state.yaw = yaw

        self.pub_se2.publish(se2_state)


# start the node
if __name__ == '__main__':
    rospy.init_node("svo_wrapper")
    use_imu = rospy.get_param("~use_imu")
    node = SVOWrapper(use_imu)
    rospy.spin()
