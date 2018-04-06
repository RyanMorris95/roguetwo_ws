#ifndef ROUGETWO_WS_VELOCITY_ESTIMATION_NODE_H
#define ROGUETWO_WS_VELOCITY_ESTIMATION_NODE_H

#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <roguetwo_perception/SE2.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>

using Eigen::MatrixXd;

// Estimate the velocity of the robot given 
// position and acceleration measurements.
class VelocityEstimationNode
{
public:
	VelocityEstimationNode();

	void update_acceleration(sensor_msgs::Imu new_imu);
	void update_se2(roguetwo_perception::SE2 new_se2);
	void naive_velocity_estimation(const ros::TimerEvent& event);

	// kalman filter functions
	void kalman_filter_velocity_estimation();
	void predict();
	void update_position();

	ros::NodeHandle nh;
	ros::Subscriber update_acceleration_sub;
	ros::Subscriber update_position_sub;
	ros::Publisher velocity_pub;

private:
	float imu_delta_time;
	float vo_delta_time;
	double prev_vo_time;
	double prev_imu_time;

	// state vector
	Eigen::VectorXd x_;

	// state transition matrix
	MatrixXd F_;

	// state covariance matrix
	MatrixXd P_;

	// process covariance matrix
	MatrixXd Q_;

	// measurement matrix
	MatrixXd H_;

	// measurement covariance matrix
	MatrixXd R_;

	ros::Timer timer;

	float current_acceleration;
	double velocity;
	std::vector<double> speed_estimates;
	roguetwo_perception::SE2 current_se2;
	roguetwo_perception::SE2 previous_se2;
};


#endif