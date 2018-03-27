#include <iostream>
#include "follow_trajectory.hpp"

FollowTrajectory::FollowTrajectory()
{

}

void FollowTrajectory::pid()
{

}

void FollowTrajectory::input_trajectory(roguetwo_navigation::Trajectory trajectory)
{
	ackermann_msgs::AckermannDrive ackermann_msg = ackermann_msg::AckermannDrive();
	ackermann_msg.speed = cos(trajectory.x_controls[0]);
	ackermann_msg.acceleration = 1.0;
	ackermann_msg.jerk = 1.0;
	ackermann_msg.steering_angle = trajectory.yaw_controls[0];
	ackermann_msg.steering_angle_velocity = 1;

	ackermann_pub.publish(ackermann_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "follow_trajectory");
	ros::NodeHandle nh;

	FollowTrajectory follow_trajectory = FollowTrajectory();

	follow_trajectory.ackermann_pub = nh.advertise<ackermann_msgs::AckermannDrive>("ackermann_cmd", 1);
	follow_trajectory.trajectory_sub = nh.subscribe(
		"/trajectory",
		1,
		&FollowTrajectory::input_trajectory,
		&follow_trajectory);

	ros::spin();

	return 0;
}