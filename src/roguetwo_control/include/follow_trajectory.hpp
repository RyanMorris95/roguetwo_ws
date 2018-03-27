#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <roguetwo_navigation/Trajectory.h>
#include <math.h>

#ifndef FOLLOW_TRAJECTORY_H
#define FOLLOW_TRAJECTORY_H

class FollowTrajectory
{
public:
	FollowTrajectory();

	void pid();

	ros::Publisher pwm_pub;
	ros::Subscriber trajectory_sub; 
	ros::Publisher ackermann_pub;

private:
	void input_trajectory(roguetwo_navigation::Trajectory trajectory);
};


#endif