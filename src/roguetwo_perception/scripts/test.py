import numpy as np 

from tf.transformations import *


yaw_r = 0.617542
x_r = 7.20833
y_r = 3.75481
robot_rot = euler_matrix(0, 0, yaw_r)
robot_pose = np.zeros((4, 4))
robot_pose[:4, :4] = robot_rot
robot_pose[:4, 3] = [x_r, y_r, 0, 1]
print ("Robot Pose:")
print (robot_pose)

x_p = 2.00725
y_p = -0.105195
point_pose = np.identity(4)
point_pose[:4, 3] = [x_p, y_p, 0, 1]
print ("Point Pose:")
print (point_pose)

point_tf = np.dot(point_pose, robot_pose)
print ("Point transformed: ")
print (point_tf)



# sensor pose
yaw_s = math.radians(-3)
x_s = 0.0
y_s = 0.115
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
dist = 3
ax, ay, sensor_yaw = euler_from_matrix(global_sensor_pose[:3, :3])
x_p = (dist * math.cos(sensor_yaw)) + global_sensor_pose[0, 3]
y_p = (dist * math.sin(sensor_yaw)) + global_sensor_pose[1, 3]
print ("New Point")
print (x_p, y_p)