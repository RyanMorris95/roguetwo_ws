#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
import math

from roguetwo_navigation.msg import Path


class PlotPath(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/global_path', Path, self.plot_path)
        self.fig = plt.figure()

    def plot_path(self, path):
        plt.clf()
        x_states = path.x_states
        y_states = path.y_states
        yaw_states = path.yaw_states

        plt.scatter(x_states, y_states, c='red')
        plt.plot(x_states, y_states, c='red')

        for i in range(len(yaw_states)):
            plt.arrow(x_states[i], y_states[i],
                      0.5 * math.cos(yaw_states[i]),
                      0.5 * math.sin(yaw_states[i]),
                      head_width=0.05,
                      head_length=0.05,
                      fc='k', ec='k')

        plt.ylim(-5, 5)

        self.fig.canvas.draw()


if __name__ == '__main__':
    rospy.init_node("plot_path")
    node = PlotPath()
    plt.show()
    rospy.spin()
