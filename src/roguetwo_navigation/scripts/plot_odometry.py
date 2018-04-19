import rospy
import matplotlib.pyplot as plt
import PyKDL
from roguetwo_perception.msg import SE2
from nav_msgs.msg import Odometry

class PlotSE2(object):
    def __init__(self):
        rospy.Subscriber("/odometry/relative", Odometry, self.update_se2, queue_size=1)
        rospy.Subscriber("/odometry/filtered", Odometry, self.update_se2_ekf, queue_size=1)
        self.fig = plt.figure()

        self.se2_list = []
        self.se2_filtered_list = []
        self.se2_x = []
        self.se2_y = []
        self.se2_x_f = []
        self.se2_y_f = []
        self.se2_x_ekf = []
        self.se2_y_ekf = []

        rospy.Timer(rospy.Duration(0.1), self.plot)

    def update_se2(self, odometry):
        self.se2_x.append(odometry.pose.pose.position.x)
        self.se2_y.append(odometry.pose.pose.position.y)

        pose = odometry.pose 
        
        orientation = pose.pose.orientation
        quaternion = PyKDL.Rotation.Quaternion(orientation.x, 
                                                orientation.y, 
                                                orientation.z, 
                                                orientation.w)

        yaw = quaternion.GetRPY()[2]

    def update_se2_ekf(self, odometry):
        self.se2_x_ekf.append(odometry.pose.pose.position.x)
        self.se2_y_ekf.append(odometry.pose.pose.position.y)

        pose = odometry.pose 
        
        orientation = pose.pose.orientation
        quaternion = PyKDL.Rotation.Quaternion(orientation.x, 
                                                orientation.y, 
                                                orientation.z, 
                                                orientation.w)

        yaw = quaternion.GetRPY()[2]

    def plot(self, event):
        plt.clf()
        try:
            plt.scatter(self.se2_x, self.se2_y, c='red', label='No Filter', s=4)
            plt.scatter(self.se2_x_ekf, self.se2_y_ekf, c='green', label='EKF', s=2)
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
