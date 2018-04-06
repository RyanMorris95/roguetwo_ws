#include <ros/ros.h>
#include "velocity_estimation_node.hpp"

VelocityEstimationNode::VelocityEstimationNode()
	: imu_delta_time(0.0), vo_delta_time(0.0),
	current_acceleration(0.0), velocity(0.0),
	prev_vo_time(0.0), prev_imu_time(0.0)
{
	this->current_se2.x = 0;
	this->current_se2.y = 0;
	this->current_se2.yaw = 0;

	this->previous_se2.x = 0;
	this->previous_se2.y = 0;
	this->previous_se2.yaw = 0;
}


// Updates and calculates the acceleration in the robot's coordinate system
//
// @param new_imu: current imu message from robot
// @return :
void VelocityEstimationNode::update_acceleration(sensor_msgs::Imu new_imu)
{
	float x_acceleration = new_imu.linear_acceleration.x;

}

// Update the SE2 state of the robot and start the calculate velocity
// timer if this is the first time executing the function.
//
// @param new_se2: current se2 state of the robot 
// @return :
void VelocityEstimationNode::update_se2(roguetwo_perception::SE2 new_se2)
{
	if (prev_vo_time == 0)
	{
		prev_vo_time = ros::Time::now().toSec();
		this->previous_se2 = new_se2;

		timer = nh.createTimer(
				ros::Duration(0.1), 
				&VelocityEstimationNode::naive_velocity_estimation,
				this,
				false);
	}
	else
	{
		this->current_se2 = new_se2;
		this->vo_delta_time = ros::Time::now().toSec() - prev_vo_time;
		this->prev_vo_time = ros::Time::now().toSec();
	}
}


// Calulate the velocity just by dividing the time.
//
// @param event: ros timer event
// @return :
void VelocityEstimationNode::naive_velocity_estimation(const ros::TimerEvent& event)
{
	float x_delta = this->current_se2.x - this->previous_se2.x;
	
	std::cout << "X Delta: " << x_delta << std::endl;
	if (x_delta > 0.001 || x_delta < -0.001)
	{
		// find velocity in the robot's coordinate frame a.k.a speed
		float velocity_x = x_delta / vo_delta_time;
		float speed = velocity_x / cos(this->current_se2.yaw);
		this->speed_estimates.push_back(speed);

		std_msgs::Float32 msg;
		msg.data = speed;
		velocity_pub.publish(msg);
	}

	this->previous_se2 = this->current_se2;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "velocity_estimation");
	VelocityEstimationNode velocity_estimation_node = VelocityEstimationNode();

	velocity_estimation_node.velocity_pub = velocity_estimation_node.nh.advertise<std_msgs::Float32>("velocity", 1);

	velocity_estimation_node.update_position_sub = velocity_estimation_node.nh.subscribe(
		"/se2_state",
		1,
		&VelocityEstimationNode::update_se2,
		&velocity_estimation_node);

	velocity_estimation_node.update_acceleration_sub = velocity_estimation_node.nh.subscribe(
		"/imu",
		1,
		&VelocityEstimationNode::update_acceleration,
		&velocity_estimation_node);

	ros::spin();

}