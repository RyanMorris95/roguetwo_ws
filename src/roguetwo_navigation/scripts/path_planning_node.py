#!/usr/bin/env python

import frenet_optimal_trajectory
import rospy
import numpy as np
import matplotlib.pyplot as plt

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
        self.current_se2 = [0.0, 0.0, 0.0]
        self.goal_se2 = [900.0, 0.0, 0.0]
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
        self.obstacles = []

        self.local_path_timer = None

        self.done = False

        self.fig = plt.figure()

    def update_se2(self, se2):
        """
        Update the se2 state of the robot whenever possible
        :return:
        """
        self.current_se2 = [se2.x, se2.y, se2.yaw]

    def generate_global_path(self):
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

        self.local_path_timer = rospy.Timer(rospy.Duration(0.5), self.generate_local_path)

    def generate_local_path(self, event):
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

        if np.hypot(path.x[1] - self.target_x[-1],
                    path.y[1] - self.target_y[-1]) <= 1.0:
            print ("Goal!  Shutting down path planning.")
            self.done = True
        else:  # publish local path
            path_msg = Path()
            for x, y, yaw in zip(path.x[1:], path.y[1:], path.yaw[1:]):
                path_msg.x_states.append(x)
                path_msg.y_states.append(y)
                path_msg.yaw_states.append(yaw)

            self.pub_local_path.publish(path_msg)
            self.plot(path)

    def plot(self, path):
        area = 200
        plt.clf()
        plt.plot(self.target_x, self.target_y)
        plt.plot(path.x[1:], path.y[1:], "-or")
        plt.plot(path.x[1], path.y[1], "vc")
        plt.xlim(path.x[1] - area, path.x[1] + area)
        plt.ylim(path.y[1] - area, path.y[1] + area)
        plt.grid()
        self.fig.canvas.draw()


if __name__ == '__main__':
    rospy.init_node("path_planning_node")
    node = PathPlanningNode()
    node.generate_global_path()
    plt.show()
    while not node.done:
        pass