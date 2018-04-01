#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
import math

from roguetwo_navigation.msg import Path


class PlotPath(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/local_path', Path, self.plot_path)
        self.fig = plt.figure()

    def plot_arrow(self, x, y, yaw, length=0.5, width=0.1):
      plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
        head_length=width, head_width=width)
      plt.plot(x, y)


    def plot_path(self, path):
        plt.clf()
        x_states = path.x_states
        y_states = path.y_states
        yaw_states = path.yaw_states

        plt.scatter(x_states, y_states, c='red')
        plt.plot(x_states, y_states, c='red')
        #self.plot_arrow(x[0], x[1], x[2])

        plt.xlim(-5, 15)
        plt.ylim(-5, 5)

        self.fig.canvas.draw()


if __name__ == '__main__':
    rospy.init_node("plot_path")
    node = PlotPath()
    plt.show()
    rospy.spin()
