import rospy
import matplotlib.pyplot as plt
import PyKDL
import math 
import time
import numpy as np
from roguetwo_perception.msg import SE2
from nav_msgs.msg import Odometry, OccupancyGrid
from roguetwo_navigation.msg import Path

class PlotSE2(object):
    def __init__(self):

        self.fig = plt.figure()

        self.se2_list = []
        self.se2_filtered_list = []
        self.se2_x = [0]
        self.se2_y = [0]
        self.se2_x_f = []
        self.se2_y_f = []
        self.se2_x_ekf = [0]
        self.se2_y_ekf = [0]

        self.vel_x = 0
        self.vel_y = 0
        self.vel_x2 = 0
        self.vel_y2 = 0
        self.speed = 0
        self.speed2 = 0

        self.yaw = 0
        self.yaw2 = 0

        self.x_states = []
        self.y_states = []

        self.obstacles = np.array(False)

        self.last_time = 0

        rospy.Subscriber("/encoder/odometry", Odometry, self.update_se2, queue_size=1)
        rospy.Subscriber("/odometry/filtered", Odometry, self.update_se2_ekf, queue_size=1)
        rospy.Subscriber("/local_path", Path, self.update_path, queue_size=1)
        rospy.Subscriber("/occupancy_grid", OccupancyGrid, self.update_grid, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.plot)


    def update_grid(self, occupancy_grid_msg):
        self.obstacles = []

        for width in range(occupancy_grid_msg.info.width):
            for height in range(occupancy_grid_msg.info.height):
                if occupancy_grid_msg.data[height*occupancy_grid_msg.info.width + width] == 100:
                    y = width * occupancy_grid_msg.info.resolution + occupancy_grid_msg.info.resolution / 2.
                    y -= occupancy_grid_msg.info.origin.position.y
                    x = height * occupancy_grid_msg.info.resolution + occupancy_grid_msg.info.resolution / 2.
                    x -= occupancy_grid_msg.info.origin.position.x
                    self.obstacles.append([float(y), float(x)])
                    print (y, x)

        self.obstacles = np.array(self.obstacles)


    def update_path(self, path):
        self.x_states = path.x_states
        self.y_states = path.y_states

    def get_yaw(self, odometry):
        pose = odometry.pose 
        
        orientation = pose.pose.orientation
        quaternion = PyKDL.Rotation.Quaternion(orientation.x, 
                                                orientation.y, 
                                                orientation.z, 
                                                orientation.w)

        yaw = quaternion.GetRPY()[2]
        return yaw 

    def update_se2(self, odometry):
        self.se2_x.append(odometry.pose.pose.position.x)
        self.se2_y.append(odometry.pose.pose.position.y)
        curr_time = time.time()
        dt = curr_time - self.last_time
        self.last_time = curr_time

        self.vel_x = (self.se2_x[-1] - self.se2_x[-2]) / dt
        self.vel_y = (self.se2_y[-1] - self.se2_y[-2]) / dt
        self.yaw = self.get_yaw(odometry)
        self.speed = self.vel_x * math.cos(self.yaw)
        print (self.yaw)


    def update_se2_ekf(self, odometry):
        self.se2_x_ekf.append(odometry.pose.pose.position.x)
        self.se2_y_ekf.append(odometry.pose.pose.position.y)

        self.yaw2 = self.get_yaw(odometry)
        self.vel_x2 = odometry.twist.twist.linear.x 
        self.vel_y2 = odometry.twist.twist.linear.y
        self.speed2 = self.vel_x2 * math.cos(self.yaw2)

    def plot(self, event):
        plt.clf()
        try:
            plt.scatter(self.se2_x, self.se2_y, c='red', label='No Filter', s=4)
            plt.scatter(self.se2_x_ekf, self.se2_y_ekf, c='green', label='EKF', s=2)
            plt.scatter(self.x_states, self.y_states, c='blue', label='Path', s=1, alpha=0.3)
            if self.obstacles.all() and self.obstacles.shape[0] > 0:
                plt.scatter(self.obstacles[:, 0], self.obstacles[:, 1])
            plt.arrow(self.se2_x[-1], self.se2_y[-1],
                        0.5 * math.cos(self.yaw),
                        0.5 * math.sin(self.yaw),
                        head_width=0.1,
                        head_length=0.05,
                        fc='k', ec='k')
            plt.arrow(self.se2_x_ekf[-1], self.se2_y_ekf[-1],
                        0.5 * math.cos(self.yaw2),
                        0.5 * math.sin(self.yaw2),
                        head_width=0.1,
                        head_length=0.05,
                        fc='k', ec='k')
            vel_str = "Vx: " + str(round(self.vel_x, 2)) + "  Vy: " + str(round(self.vel_y, 2)) + "  Vx2: " + str(round(self.vel_x2, 2)) + "  Vy2: " + str(round(self.vel_y2, 2))
            speed_str = "Speed: " + str(round(self.speed, 2)) + "  Speed2: " + str(round(self.speed2, 2))
            #plt.title(vel_str)
            plt.title(speed_str)

            plt.legend()
            plt.grid()

            if self.obstacles.all() and self.obstacles.shape[0] > 0:
                minx = min([min(self.se2_x), min(self.obstacles[:,0])])
                maxx = max([max(self.se2_x), max(self.obstacles[:,0])])
            else:
                minx = min(self.se2_x)
                maxx = max(self.se2_x)

            plt.xlim(minx-3, maxx+3)
            plt.ylim(min(self.se2_y)-3, max(self.se2_y)+3)

            self.fig.canvas.draw()
        except:
            pass


if __name__ == '__main__':
    rospy.init_node("plot_path")
    node = PlotSE2()
    plt.show()
