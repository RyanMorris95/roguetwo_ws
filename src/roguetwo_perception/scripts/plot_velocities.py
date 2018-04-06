#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
import math

from std_msgs.msg import Float32


class PlotVelocity(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/velocity', Path, self.plot_path)
        self.fig = plt.figure()
        self.y = []
        self.x = []
        self.count = 0

    def plot_path(self, velocity):
        plt.clf()

        self.y.append(velocity)
        self.x.append(self.count)
        self.count += 1

        plt.scatter(self.x, self.y, c='red')

        self.fig.canvas.draw()


if __name__ == '__main__':
    rospy.init_node("plot_velocity")
    node = PlotPath()
    plt.show()
    rospy.spin()
