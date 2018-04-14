#ifndef ROUGETWO_WS_SENSOR_FUSION_H
#define ROUGETWO_WS_SENSOR_FUSION_h

#include <ros/ros.h>
#include <roguetwo_perception/SE2.h>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include "kalman_filter.h"

using namespace Eigen;

class SensorFusion
{
public:
    SensorFusion();

    void predict(const sensor_msgs::Imu imu_msg);
    void update(const roguetwo_perception::SE2 se2_msg);
    void publish();

    ros::NodeHandle nh;
    ros::Subscriber update_sub;
    ros::Subscriber predict_sub;
    ros::Subscriber predict_sub2;
    
    ros::Publisher se2_filtered_pub;
    ros::Publisher velocity_pub;

private:
    KalmanFilter x_kalman;
    KalmanFilter y_kalman;

    float curr_x;
    float curr_y;
    float curr_x_vel;
    float curr_y_vel;
    float curr_yaw;

    float dt;

    double prev_seconds;

    bool predict_called;
};



#endif