#!/usr/bin/env python
import rospy

from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class JoyToAckermann(object):
	pub_ackermann = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
	pub_start_autonomous = rospy.Publisher('/start_autonomous', Bool, queue_size=1)

	def __init__(self):
		rospy.Subscriber('/joy', Joy, self.convert_to_ackermann, queue_size=1)
		self.start_autonomous = False
		self.max_motor_vel = 0.5
		self.max_steering_angle = 0.25
		self.current_cmd_msg = None

		self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_cmd)

	def publish_cmd(self, event):
		if self.current_cmd_msg:
			self.pub_ackermann.publish(self.current_cmd_msg)

	def convert_to_ackermann(self, joy_msg):
		axes = joy_msg.axes
		buttons = joy_msg.buttons

		if buttons[0] == 1:  # button a clicked
			self.start_autonomous = not self.start_autonomous
			bool_msg = Bool()
			bool_msg = self.start_autonomous
			self.pub_start_autonomous.publish(bool_msg)

		motor_axis = axes[1];
		steering_axis = axes[0];

		# convert to speed
		motor_velocity = motor_axis * self.max_motor_vel
		steering_angle = steering_axis * self.max_steering_angle

		msg = AckermannDrive()
		msg.speed = motor_velocity
		msg.acceleration = 1
		msg.jerk = 1
		msg.steering_angle = steering_angle
		msg.steering_angle_velocity = 1
		self.current_cmd_msg = msg
		#self.pub_ackermann.publish(msg)
		#print (msg)


if __name__ == "__main__":
	rospy.init_node('joy_to_ackermann')
	node = JoyToAckermann()
	rospy.spin()