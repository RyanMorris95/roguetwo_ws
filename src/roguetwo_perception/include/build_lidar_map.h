#ifndef ROUGETWO_WS_BUILD_LIDAR_MAP_H
#define ROUGETWO_WS_BUILD_LIDAR_MAP_H

#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/MapMetaData.h>
#include <sensor_msgs/Range.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <ros/ros.h>

using namespace Eigen;
typedef Matrix<int, Dynamic, Dynamic> MatriXd;

struct Point
{
    float x;
    float y;

    Point(): x(0), y(0) {}
};

struct SE2 
{
    float x;
    float y;
    float yaw;

    SE2(): x(0), y(0), yaw(0) {}
};

struct Footprint
{
    float width;
    float height;

    Footprint(): width(0), height(0) {}
};

class BuildLidarMap 
{
public:

    BuildLidarMap(float _resolution,
                int _height,
                int _width,
                float _lidar1_angle,
                float _lidar2_angle,
                float _lidar3_angle,
                float _lidar4_angle,
                Point _lidar1_offset,
                Point _lidar2_offset,
                Point _lidar3_offset,
                Point _lidar4_offset,
                Footprint _footprint);

    ~BuildLidarMap();
    
    void run();
    void set_map_publisher(ros::Publisher pub) { this->map_publisher = pub; }

    void lidar_front_left_callback(const sensor_msgs::Range::ConstPtr& range);
    void lidar_front_right_callback(const sensor_msgs::Range::ConstPtr& range);
    void lidar_back_left_callback(const sensor_msgs::Range::ConstPtr& range);
    void lidar_back_right_callback(const sensor_msgs::Range::ConstPtr& range);
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& range);
    void start_callback(const std_msgs::Bool::ConstPtr& start);
    void publish_map(const ros::TimerEvent& event);
    void update_map(const Point& point);
    bool is_between(const float min, const float max, const float curr);
    bool difference_within(const float x1, const float x2, const float thresh);
    Matrix3d get_rot(const float yaw);
    Matrix4f get_tf(const Matrix3d r, const float x, const float y);

    Point calculate_point(const float distance,
                        const float angle,
                        const Point offset);
    Point calculate_point2(const float distance,
                        const float angle,
                        const Point offset,
                        const int lidar_num);

    ros::NodeHandle nh;

private:
    MatrixXd map;
    ros::Publisher map_publisher;

    float resolution;
    int height;
    int width;
    int rows;
    int cols;
    int origin_x;
    int origin_y;

    // the angle the lidar is turned at in relation to the vehicle
    float lidar1_angle;
    float lidar2_angle;
    float lidar3_angle;
    float lidar4_angle;

    Matrix4f lidar1_robot_tf;
    Matrix4f lidar2_robot_tf;
    Matrix4f lidar3_robot_tf;
    Matrix4f lidar4_robot_tf;

    Matrix4f robot_transformation;

    // x,y offset of lidar from front,center point of the car
    Point lidar1_offset;
    Point lidar2_offset;
    Point lidar3_offset;
    Point lidar4_offset;

    // previous lidar distances
    float lidar1_prev_dist;
    float lidar2_prev_dist;
    float lidar3_prev_dist;
    float lidar4_prev_dist;

    Footprint footprint;
    SE2 curr_se2;
    
    ros::Timer timer;
};

#endif