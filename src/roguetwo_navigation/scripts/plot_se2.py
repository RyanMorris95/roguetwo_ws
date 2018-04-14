import rospy
import matplotlib.pyplot as plt
from roguetwo_perception.msg import SE2

class PlotSE2(object):
    def __init__(self):
        rospy.Subscriber("/se2_state", SE2, self.update_se2, queue_size=1)
        rospy.Subscriber("/se2_state_filtered", SE2, self.update_se2_filtered, queue_size=1)
        self.fig = plt.figure()

        self.se2_list = []
        self.se2_filtered_list = []
        self.se2_x = []
        self.se2_y = []
        self.se2_x_f = []
        self.se2_y_f = []

        rospy.Timer(rospy.Duration(0.1), self.plot)

    def update_se2(self, se2):
        self.se2_x.append(se2.x)
        self.se2_y.append(se2.y)

    def update_se2_filtered(self, se2):
        self.se2_x_f.append(se2.x)
        self.se2_y_f.append(se2.y)

    def plot(self, event):
        plt.clf()
        try:
            plt.scatter(self.se2_x, self.se2_y, c='red', label='No Filter', s=4)
            plt.scatter(self.se2_x_f, self.se2_y_f, c='blue', label='Kalman Filter', s=0.5)

            plt.legend()
            plt.grid()
            plt.xlim(-20, 20)
            plt.ylim(-20, 20)

            self.fig.canvas.draw()
        except:
            pass


if __name__ == '__main__':
    rospy.init_node("plot_path")
    node = PlotSE2()
    plt.show()
    rospy.spin()
