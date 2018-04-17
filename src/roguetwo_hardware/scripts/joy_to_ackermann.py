#!/usr/bin/env python
import rospy
import math

from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32


class JoyToAckermann(object):
    pub_ackermann = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
    pub_start_autonomous = rospy.Publisher('/start_autonomous', Bool, queue_size=1)
    pub_switch_cameras = rospy.Publisher('/switch_cameras', Bool, queue_size=1)
    pub_max_vel = rospy.Publisher('/max_motor_vel', Float32, queue_size=1)

    def __init__(self):
        rospy.Subscriber('/joy', Joy, self.convert_to_ackermann, queue_size=1)
        self.start_autonomous = False
        self.front_camera = True
        self.max_motor_vel = 0.5
        self.max_steering_angle = math.radians(30)
        self.current_cmd_msg = None

        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_cmd)

    def publish_cmd(self, event):
        if self.current_cmd_msg:
            self.pub_ackermann.publish(self.current_cmd_msg)

    def convert_to_ackermann(self, joy_msg):
        axes = joy_msg.axes
        buttons = joy_msg.buttons

        if buttons[7] == 1:  # button a clicked
            self.start_autonomous = not self.start_autonomous
            bool_msg = Bool()
            bool_msg = self.start_autonomous
            self.pub_start_autonomous.publish(bool_msg)
            
            if self.start_autonomous:
                rospy.loginfo("joy_to_ackermann: start autonomous mode")
                self.timer.shutdown()
            else:
                rospy.loginfo("joy_to_ackermann: start manual mode")
                self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_cmd)


        if buttons[3] == 1:  # button y clicked
            self.front_camera = not self.front_camera
            bool_msg = Bool()
            bool_msg = self.front_camera
            self.pub_switch_cameras.publish(bool_msg)

        if axes[-1] == 1:  # up on the D pad
            self.max_motor_vel += 0.05
            rospy.loginfo("joy_to_ackermann: max motor velocity increased to: " + str(self.max_motor_vel))
            msg = Float32()
            msg.data = self.max_motor_vel
            self.pub_max_vel.publish(msg)
        elif axes[-1] == -1:  # down on the D pad
            self.max_motor_vel -= 0.05
            rospy.loginfo("joy_to_ackermann: max motor velocity increased to: " + str(self.max_motor_vel))
            msg = Float32()
            msg.data = self.max_motor_vel
            self.pub_max_vel.publish(msg)

        if buttons[2] == 1:  # button x clicked
            motor_velocity = 0
            steering_angle = 0

        else:
            steering_axis = axes[0]
            reverse_trigger = axes[2]
            gas_trigger = axes[5]

            # convert to speed
            if gas_trigger == 1 and reverse_trigger == 1:
                motor_velocity = 0
            elif reverse_trigger != 1:
                motor_velocity = -1 * abs(reverse_trigger * self.max_motor_vel)
            else:
                motor_velocity = abs(gas_trigger * self.max_motor_vel)

            # convert to steering angle and reverse it since
            # the xbox controller returns right in the negatives
            steering_angle = steering_axis * self.max_steering_angle
            steering_angle *= -1

        msg = AckermannDrive()
        msg.speed = motor_velocity
        msg.acceleration = 1
        msg.jerk = 1
        msg.steering_angle = steering_angle
        msg.steering_angle_velocity = 1
        self.current_cmd_msg = msg
        print (msg)

if __name__ == "__main__":
    rospy.init_node('joy_to_ackermann')
    node = JoyToAckermann()
    rospy.spin()
