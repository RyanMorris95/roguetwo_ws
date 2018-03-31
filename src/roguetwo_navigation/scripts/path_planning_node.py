#!/usr/bin/env python

import frenet_optimal_trajectory
import dynamic_window_planner
import rospy
import numpy as np
import matplotlib.pyplot as plt
import math
import sys, select, termios, tty
import time

from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from roguetwo_navigation.msg import Path
from roguetwo_perception.msg import SE2


class PathPlanningNode(object):
    """
    This node handles the global and local path planning.  It publishes
    the local trajectory.
    """
    # setup publishers
    pub_local_path = rospy.Publisher('/local_path', Path, queue_size=1)

    def __init__(self):
        # setup subscribers
        rospy.Subscriber('/se2_state', SE2, self.update_se2)
        rospy.Subscriber('/projected_map', OccupancyGrid, self.update_obstacles)
        rospy.Subscriber('/start_autonomous', Bool, self.start_autonomous)

        # Frenet Optimal Trajectory variables
        self.current_se2 = [0.0, 0.0, 0.0]
        self.goal_se2 = [9.0, 0.0, 0.0]
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        self.target_curvature = None
        self.global_cubic_spline = None

        self.current_speed = 25.0  # current speed, cm/s
        self.current_d = 0.0  # current lateral position, cm
        self.current_d_d = 0.0  # current lateral speed, cm/s
        self.current_d_dd = 0.0  # current lateral acceleration, cm/s
        self.s0 = 0.0  # current course position

        # Dynamic Window Planning Variables
        self.state = [0.0, 0.0, 0.0, 0.0, 0.0]  # [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.goal = [0, 0]
        self.u = np.array([0.0, 0.0])
        self.traj = np.array(self.state)
        self.dt = 0.1

        # For all planner variables
        self.obstacles = []
        self.local_path_timer = None
        self.done = False
        self.fig = plt.figure()

        # ran

    def start_autonomous(self, start):
        self.goal = [0, 0]
        self.generate_dynamic_window_global_path(self.goal)

    def update_obstacles(self, occupancy_grid_msg):
        """
        Converts the occupancy grid message into an obstacles array
        :param occupancy_grid_msg:
        :return:
        """
        self.obstacles = []

        for width in range(occupancy_grid_msg.info.width):
            for height in range(occupancy_grid_msg.info.height):
                if occupancy_grid_msg.data[height*occupancy_grid_msg.info.width + width] == 100:
                    y = width * occupancy_grid_msg.info.resolution + occupancy_grid_msg.info.resolution / 2.
                    y += occupancy_grid_msg.info.origin.position.x
                    x = height * occupancy_grid_msg.info.resolution + occupancy_grid_msg.info.resolution / 2.
                    x += occupancy_grid_msg.info.origin.position.y
                    self.obstacles.append([float(y), float(x)])

        self.obstacles = np.array(self.obstacles)
        # plt.plot(self.obstacles[:,0], self.obstacles[:,1])
        # plt.show()
        np.save('obstacles.npy', self.obstacles)
        print ('saved obstacles')

    def update_se2(self, se2):
        """
        Update the se2 state of the robot whenever possible
        :return:
        """
        self.current_se2 = [se2.x, se2.y, se2.yaw]

    def generate_frenet_global_path(self):
        # way points
        x_waypoints = np.linspace(self.current_se2[0], self.goal_se2[0], 3)
        y_waypoints = np.linspace(self.current_se2[1], self.goal_se2[1], 3)

        tx, ty, tyaw, tc, csp = frenet_optimal_trajectory.generate_target_course(x_waypoints,
                                                                                 y_waypoints)
        self.target_x = tx
        self.target_y = ty
        self.target_yaw = tyaw
        self.target_curvature = tc
        self.global_cubic_spline = csp

        self.local_path_timer = rospy.Timer(rospy.Duration(0.5), self.generate_frenet_local_path)

    def generate_dynamic_window_global_path(self, goal=[0.0, 0.0]):
        self.goal = goal
        self.local_path_timer = rospy.Timer(rospy.Duration(0.1),
                                            self.generate_dynamic_window_local_path)

    def generate_dynamic_window_local_path(self, event):
        if type(self.obstacles) == list:
            self.obstacles = np.matrix([[-100, -100]])
        self.u, ltraj, all_traj = dynamic_window_planner.dwa_control(self.state,
                                                           self.u,
                                                           self.goal,
                                                           self.obstacles)
        self.state = dynamic_window_planner.motion(self.state,
                                                   self.u,
                                                   self.dt)
        self.state[0] = self.current_se2[0]
        self.state[1] = self.current_se2[1]
        self.state[2] = self.current_se2[2]
        self.state = [float(x) for x in self.state]

        distance_from_goal = math.sqrt((self.state[0]-self.goal[0])**2 +
                                       (self.state[1] - self.goal[1])**2)
        print ('Distance From Goal: ', distance_from_goal)
        path_msg = Path()
        if distance_from_goal < 0.5:
            print ('Finished!!!!!!')
            path_msg.x_states.append(-100)
            path_msg.y_states.append(-100)
            path_msg.yaw_states.append(-100)
            self.pub_local_path.publish(path_msg)
            rospy.signal_shutdown('Made it to goal')
        else:
            for i in range(ltraj.shape[0]):
                state = ltraj[i,:]
                path_msg.x_states.append(state[0])
                path_msg.y_states.append(state[1])
                path_msg.yaw_states.append(state[2])

        self.pub_local_path.publish(path_msg)

        self.traj = np.vstack((self.traj, self.state))
        self.plot_dynamic(ltraj, self.state, self.goal, self.obstacles)

    def generate_frenet_local_path(self, event):
        path = frenet_optimal_trajectory.frenet_optimal_planning(self.global_cubic_spline,
                                                                 self.s0,
                                                                 self.current_speed,
                                                                 self.current_d,
                                                                 self.current_d_d,
                                                                 self.current_d_dd,
                                                                 self.obstacles)

        self.s0 = path.s[1]
        self.current_d = path.d[1]
        self.current_d_d = path.d_d[1]
        self.current_d_dd = path.d_dd[1]
        self.current_speed = path.s_d[1]

        path_msg = Path()
        if np.hypot(path.x[1] - self.target_x[-1],
                    path.y[1] - self.target_y[-1]) <= 1.0:
            print ("Goal!  Shutting down path planning.")
            self.done = True
            # send shutdown pattern
            path_msg.x_states.append(-100)
            path_msg.y_states.append(-100)
            path_msg.yaw_states.append(-100)
            self.pub_local_path.publish(path_msg)
        else:  # publish local path
            for x, y, yaw in zip(path.x[1:], path.y[1:], path.yaw[1:]):
                path_msg.x_states.append(x)
                path_msg.y_states.append(y)
                path_msg.yaw_states.append(yaw)

            self.pub_local_path.publish(path_msg)
            self.plot(path)

    def plot_frenet(self, path):
        area = 200
        plt.clf()
        plt.plot(self.target_x, self.target_y)
        plt.plot(path.x[1:], path.y[1:], "-or")
        plt.plot(path.x[1], path.y[1], "vc")
        plt.xlim(path.x[1] - area, path.x[1] + area)
        plt.ylim(path.y[1] - area, path.y[1] + area)
        plt.grid()
        self.fig.canvas.draw()

    def plot_dynamic(self, ltraj, x, goal, ob):
        plt.clf()
        plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
        plt.plot(x[0], x[1], "xr")
        plt.plot(goal[0], goal[1], "xb")
        plt.plot(ob[:, 0], ob[:, 1], "ok")
        self.plot_arrow(x[0], x[1], x[2])
        plt.xlim(-5, goal[0]+5)
        plt.ylim(-goal[1]-5, goal[1]+5)
        plt.grid()
        self.fig.canvas.draw()

    def plot_arrow(self, x, y, yaw, length=0.5, width=0.1):
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  head_length=width, head_width=width)
        plt.plot(x, y)

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("path_planning_node")
    node = PathPlanningNode()
    #node.generate_dynamic_window_global_path(goal=[5.0, 0.0])
    plt.show()
    while not rospy.is_shutdown():
        pass
