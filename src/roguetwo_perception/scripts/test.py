import numpy as np 

from tf.transformations import *


yaw_r = 2.49
x_r = 5.44
y_r = -1.82
robot_rot = euler_matrix(0, 0, yaw_r)
robot_pose = np.zeros((4, 4))
robot_pose[:4, :4] = robot_rot
robot_pose[:4, 3] = [x_r, y_r, 0, 1]
print ("Robot Pose:")
print (robot_pose)

# sensor pose
yaw_s = 0
x_s = 0.0
y_s = 0.21
sensor_rot = euler_matrix(0, 0, yaw_s)
sensor_pose = np.zeros((4, 4))
sensor_pose[:4, :4] = sensor_rot
sensor_pose[:4, 3] = [x_s, y_s, 0, 1]
print ("sensor Pose:")
print (sensor_pose)

# global sensor pose
global_sensor_pose = np.dot(sensor_pose, robot_pose)
print ("global sensor pose: ")
print (global_sensor_pose)

# global sensor yaw
dist = 2.7225
ax, ay, sensor_yaw = euler_from_matrix(global_sensor_pose[:3, :3])
x_p = (dist * math.cos(sensor_yaw)) + global_sensor_pose[0, 3]
y_p = (dist * math.sin(sensor_yaw)) + global_sensor_pose[1, 3]
print ("New Point")
print (x_p, y_p)