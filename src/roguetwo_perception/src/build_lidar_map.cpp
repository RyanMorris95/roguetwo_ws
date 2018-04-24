#include "build_lidar_map.h"


BuildLidarMap::BuildLidarMap(
    float _resolution,
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
    Footprint _footprint
)
  : resolution(_resolution),
    height(_height),
    width(_width),
    lidar1_angle(_lidar1_angle),
    lidar2_angle(_lidar2_angle),
    lidar3_angle(_lidar3_angle),
    lidar4_angle(_lidar4_angle),
    lidar1_offset(_lidar1_offset),
    lidar2_offset(_lidar2_offset),
    lidar3_offset(_lidar3_offset),
    lidar4_offset(_lidar4_offset),
    footprint(_footprint)
{
    
    this->rows = (int) this->height / this->resolution;
    this->cols = (int) this->width / this->resolution;

    this->map = MatrixXd::Zero(this->rows, this->cols);

    this->origin_y = (int) this->rows / 2;
    this->origin_x = (int) this->cols / 2;
    
    this->curr_se2.x = 0;
    this->curr_se2.y = 0;
    this->curr_se2.yaw = 0;

    this->lidar1_prev_dist = 0;
    this->lidar2_prev_dist = 0;
    this->lidar3_prev_dist = 0;
    this->lidar4_prev_dist = 0;

    Matrix3d r; 
    r = this->get_rot(0);
    this->lidar1_robot_tf = this->get_tf(r, this->lidar1_offset.x, this->lidar1_offset.y);
    //r = this->get_rot(this->lidar2_angle);
    this->lidar2_robot_tf = this->get_tf(r, this->lidar2_offset.x, this->lidar2_offset.y);
    //r = this->get_rot(this->lidar3_angle);
    this->lidar3_robot_tf = this->get_tf(r, this->lidar3_offset.x, this->lidar3_offset.y);
    //r = this->get_rot(this->lidar4_angle);
    this->lidar4_robot_tf = this->get_tf(r, this->lidar4_offset.x, this->lidar4_offset.y);
    std::cout << "Lidar 4 tf: " << this->lidar4_robot_tf << std::endl;


    std::cout << "Build Lidar Map Initialization Values: " << std::endl;
    std::cout << "Resolution: " << this->resolution << std::endl;
    std::cout << "Height: " << this->height << std::endl;
    std::cout << "Width: " << this->width << std::endl; 
    std::cout << "Rows: " << this->rows << std::endl;
    std::cout << "Cols: " << this->cols << std::endl;
}


BuildLidarMap::~BuildLidarMap()
{

}


void BuildLidarMap::run()
{

}

Matrix4f BuildLidarMap::get_tf(const Matrix3d r, const float x, const float y)
{
    Matrix4f tf;
    tf << r(0,0), r(0,1), r(0,2), x,
          r(1,0), r(1,1), r(1,2), y,
          r(2,0), r(2,1), r(2,2), 0,
          0, 0, 0, 1;
    return tf;
}


Matrix3d BuildLidarMap::get_rot(const float yaw)
{
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> quat = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d r = quat.matrix();   
    
    return r;
}


// Turn the map into a occupancy grid message and publish
// over ros.
//
// @param event: timer event that triggers this function
// @return :
void BuildLidarMap::publish_map(const ros::TimerEvent& event)
{
    nav_msgs::OccupancyGrid occupancy_msg = nav_msgs::OccupancyGrid();

    std_msgs::Header header = std_msgs::Header();
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    occupancy_msg.header = header;

    nav_msgs::MapMetaData map_meta_data = nav_msgs::MapMetaData();
    map_meta_data.map_load_time = ros::Time::now();
    map_meta_data.resolution = this->resolution;
    map_meta_data.width = this->cols;
    map_meta_data.height = this->rows;
    map_meta_data.origin.position.x = this->height / 2;
    map_meta_data.origin.position.y = this->width / 2;

    occupancy_msg.info = map_meta_data;

    for (int i = 0; i < this->map.size(); i++)
    {
        occupancy_msg.data.push_back(*(this->map.data() + i));
    }

    //occupancy_msg.data = data;
    this->map_publisher.publish(occupancy_msg);
}

// Given a point update the map matrix with a 100
// which represents an occupied cell.
//
// @param point: point struct
// @return :
void BuildLidarMap::update_map(const Point& point)
{
    float x = point.x;
    float y = point.y;

    // round x and y values to the resolution 
    x = int(x / this->resolution) + this->origin_x;
    y = int(y / this->resolution) + this->origin_y;

    // make point x, y occupied in the map
    map(x, y) = 100;

}

// Updates the current SE2 pose of the robot.  Also,
// updates the transformation matrix of the robot which
// is used to project the points into the robot's frame.
//
// @param odometry: current ros odometry message
// @return :
void BuildLidarMap::odometry_callback(const nav_msgs::Odometry::ConstPtr& odometry)
{
	float x = odometry->pose.pose.position.x;
	float y = odometry->pose.pose.position.y;
	tf::Quaternion q(
		odometry->pose.pose.orientation.x,
		odometry->pose.pose.orientation.y,
		odometry->pose.pose.orientation.z,
		odometry->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

    curr_se2.x = x;
	curr_se2.y = y;
	curr_se2.yaw = yaw;

    Matrix3d r = this->get_rot(yaw);
    this->robot_transformation = this->get_tf(r, x, y);
}


// Turn the lidar's distance reading into an x, y point 
// and then into a point within the map frame.
//
// @param distance: lidar distance reading, m
// @param angle: angle of lidar, rads
// @param offset: x, y offset of lidar from robot's 0,0 , (m, m)
// @return map_point: projected lidar point onto the map frame, (m, m)
Point BuildLidarMap::calculate_point(
    const float distance,
    const float angle,
    const Point offset
)
{
    // REMEBER X IS FORWARD, Y IS LEFT
    // transform coords in relation to the sensor
    float x_sensor = distance * cos(angle);
    float y_sensor = distance * sin(angle);

    // transform coords in relation to the robot center
    float x_robot = x_sensor + offset.x;
    float y_robot = y_sensor + offset.y;

    // transform coords to the map frame
    Eigen::Matrix4f Tp;
    Tp << 1, 0, 0, x_robot,
          0, 1, 0, y_robot,
          0, 0, 1, 0,
          0, 0, 0, 1;

    Point map_point;
    Eigen::Matrix4f Tp_r = Tp * this->robot_transformation;
    
    map_point.x = Tp_r(0, 3);
    map_point.y = Tp_r(1, 3);

#if 1
    std::cout << "Distance: " << distance << std::endl;
    std::cout << "XY Sensor: " << x_sensor << " " << y_sensor << std::endl;
    std::cout << "XY Robot: " << x_robot << " " << y_robot << std::endl;
    std::cout << "Tr: " << this->robot_transformation << std::endl;
    std::cout << "Transformed Point: " << Tp_r << std::endl;
    std::cout << "Map Point: " << map_point.x << " " << map_point.y << std::endl;
    std::cout << "Robot SE2: " << this->curr_se2.x << " " << this->curr_se2.y << " " << this->curr_se2.yaw << std::endl;
#endif

    return map_point;
} 


Point BuildLidarMap::calculate_point2(
    const float distance,
    const float angle,
    const Point offset,
    const int lidar_num
)
{
    // transform to global sensor frame
    Eigen::Matrix4f lidar_tf;
    float lidar_angle;
    switch(lidar_num)
    {
        case(1): 
            lidar_tf = this->lidar1_robot_tf;
            lidar_angle = this->lidar1_angle;
            break;
        case(2): 
            lidar_tf = this->lidar2_robot_tf;
            lidar_angle = this->lidar2_angle;
            break;
        case(3): 
            lidar_tf = this->lidar3_robot_tf;
            lidar_angle = this->lidar3_angle;
            break;
        case(4): 
            lidar_tf = this->lidar4_robot_tf;
            lidar_angle = this->lidar4_angle;
            break;
    }

    Matrix4f sensor_pose = lidar_tf * this->robot_transformation;

    // get global yaw from sensor frame
    Matrix3f rot;
    rot << sensor_pose(0,0), sensor_pose(0,1), sensor_pose(0,2),
          sensor_pose(1,0), sensor_pose(1,1), sensor_pose(1,2),
          sensor_pose(2,0), sensor_pose(2,1), sensor_pose(2,2);
    Vector3f ea = rot.eulerAngles(0, 1, 2);
    float sensor_yaw = ea[2];

    // get global x, y point from lidar distance reading
    float x = (distance * cos(sensor_yaw+lidar_angle)) + sensor_pose(0,3);
    float y = (distance * sin(sensor_yaw+lidar_angle)) + sensor_pose(1,3);

    Point map_point;
    map_point.x = x;
    map_point.y = y;

#if 0
    std::cout << "Distance: " << distance << std::endl;
    std::cout << "Sensor Pose: " << sensor_pose << std::endl;
    std::cout << "Tr: " << this->robot_transformation << std::endl;
    std::cout << "Map Point: " << map_point.x << " " << map_point.y << std::endl;
    std::cout << "Sensor yaw: " << sensor_yaw << std::endl;
    std::cout << "Robot yaw: " << curr_se2.yaw << std::endl;
#endif

    return map_point;
} 

bool BuildLidarMap::difference_within(
    const float x1,
    const float x2,
    const float thresh
)
{
    if (abs(x1 - x2) < thresh) return true;
    else return false;
}

void BuildLidarMap::lidar_front_left_callback(const sensor_msgs::Range::ConstPtr& range)
{
    if (this->is_between(0.5, 4, range->range))
    {
        if (this->lidar1_prev_dist != 0)
        {
            Point point = this->calculate_point2(range->range, 
                                                this->lidar1_angle,
                                                this->lidar1_offset,
                                                1);
            this->lidar1_prev_dist = range->range;
            this->update_map(point);
        }
        else
        {
            this->lidar1_prev_dist = range->range;
        }
    }
}

void BuildLidarMap::lidar_front_right_callback(const sensor_msgs::Range::ConstPtr& range)
{
    if (this->is_between(0.5, 4, range->range))
    {
        if (this->lidar2_prev_dist != 0)
        {
            Point point = this->calculate_point2(range->range, 
                                                this->lidar2_angle,
                                                this->lidar2_offset,
                                                2);
            this->lidar2_prev_dist = range->range;
            this->update_map(point);
        }
        else
        {
            this->lidar2_prev_dist = range->range;
        }
    }
}


void BuildLidarMap::lidar_back_left_callback(const sensor_msgs::Range::ConstPtr& range)
{
    if (this->is_between(0.5, 4, range->range))
    {
        if (this->lidar3_prev_dist != 0)
        {
            Point point = this->calculate_point2(range->range, 
                                                this->lidar3_angle,
                                                this->lidar3_offset,
                                                3);
            this->lidar3_prev_dist = range->range;
            this->update_map(point);
        }
        else
        {
            this->lidar3_prev_dist = range->range;
        }
    }
}


void BuildLidarMap::lidar_back_right_callback(const sensor_msgs::Range::ConstPtr& range)
{
    if (this->is_between(0.5, 4, range->range))
    {
        if (this->lidar4_prev_dist != 0)
        {
            Point point = this->calculate_point2(range->range, 
                                                this->lidar4_angle,
                                                this->lidar4_offset,
                                                1);
            this->lidar4_prev_dist = range->range;
            this->update_map(point);
        }
        else
        {
            this->lidar4_prev_dist = range->range;
        }
    }
}

void BuildLidarMap::start_callback(const std_msgs::Bool::ConstPtr& start)
{
    this->map = MatrixXd::Zero(this->rows, this->cols);
    this->timer = this->nh.createTimer(ros::Duration(0.2),
                                      &BuildLidarMap::publish_map, 
                                       this);
}

bool BuildLidarMap::is_between(
    const float min,
    const float max,
    const float curr
)
{
    if (curr > min && curr < max) return true;
    else return false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "build_lidar_map");


    // remeber x is FORWARD and y is LEFT
// Gazebo Setup
#if 1
    float resolution = 0.2;  // m
    int height = 20;
    int width = 20;
    float lidar1_angle = 0;
    float lidar2_angle = 0;
    float lidar3_angle = 20 * M_PI / 180;
    float lidar4_angle = -20 * M_PI / 180;

    Point lidar1_offset;
    lidar1_offset.x = 0.25;
    lidar1_offset.y = 0.15;

    Point lidar2_offset;
    lidar2_offset.x = 0.25;
    lidar2_offset.y = -0.15;

    Point lidar3_offset;
    lidar3_offset.x = -0.12;
    lidar3_offset.y = 0.21;

    Point lidar4_offset;
    lidar4_offset.x = -0.12;
    lidar4_offset.y = -0.21;

    // dimensions of the robot
    Footprint robot_footprint;
    robot_footprint.width = 0.18;  // m
    robot_footprint.height = 0.18;  // m
#endif

// Robot Setup
#if 0
    float resolution = 0.1;  // m
    int height = 20;
    int width = 20;
    float lidar1_angle = 0 * M_PI / 180;
    float lidar2_angle = 0 * M_PI / 180;
    float lidar3_angle = 20 * M_PI / 180;
    float lidar4_angle = -20 * M_PI / 180;

    Point lidar1_offset;
    lidar1_offset.x = 0.0;
    lidar1_offset.y = 0.115;

    Point lidar2_offset;
    lidar2_offset.x = 0.0;
    lidar2_offset.y = -0.11;

    Point lidar3_offset;
    lidar3_offset.x = -0.24;
    lidar3_offset.y = 0.16;

    Point lidar4_offset;
    lidar4_offset.x = -0.24;
    lidar4_offset.y = -0.17;

    // dimensions of the robot
    Footprint robot_footprint;
    robot_footprint.width = 0.18;  // m
    robot_footprint.height = 0.18;  // m
#endif

    BuildLidarMap* build_lidar_map = new BuildLidarMap(resolution,
                                                    height,
                                                    width,
                                                    lidar1_angle,
                                                    lidar2_angle, 
                                                    lidar3_angle,
                                                    lidar4_angle,
                                                    lidar1_offset,
                                                    lidar2_offset,
                                                    lidar3_offset,
                                                    lidar4_offset,
                                                    robot_footprint);

    build_lidar_map->nh = ros::NodeHandle();

    ros::Subscriber odometry_sub = build_lidar_map->nh.subscribe("/encoder/odometry", 
                                        1,
                                        &BuildLidarMap::odometry_callback,
                                        build_lidar_map);
    
    ros::Publisher map_publisher = build_lidar_map->nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1);
    build_lidar_map->set_map_publisher(map_publisher);

#if 1
    ros::Subscriber lidar_front_left_sub = build_lidar_map->nh.subscribe("/sonar_frontL_distance", 
                                        1,
                                        &BuildLidarMap::lidar_front_left_callback,
                                        build_lidar_map);
    ros::Subscriber lidar_front_right_sub = build_lidar_map->nh.subscribe("/sonar_frontR_distance", 
                                        1,
                                        &BuildLidarMap::lidar_front_right_callback,
                                        build_lidar_map);
    ros::Subscriber lidar_back_left_sub = build_lidar_map->nh.subscribe("/sonar_left_distance", 
                                        1,
                                        &BuildLidarMap::lidar_back_left_callback,
                                        build_lidar_map);
    ros::Subscriber lidar_back_right_sub = build_lidar_map->nh.subscribe("/sonar_right_distance", 
                                        1,
                                        &BuildLidarMap::lidar_back_right_callback,
                                        build_lidar_map);
    // ros::Timer timer = build_lidar_map->nh.createTimer(ros::Duration(0.2),
    //                                   &BuildLidarMap::publish_map, 
    //                                   build_lidar_map);
    ros::Subscriber start_sub = build_lidar_map->nh.subscribe("start_autonomous", 
                                        1,
                                        &BuildLidarMap::start_callback,
                                        build_lidar_map);     
#endif

#if 0
    ros::Subscriber lidar_front_left_sub = build_lidar_map->nh.subscribe("/lidar_front_left_throttle", 
                                        1,
                                        &BuildLidarMap::lidar_front_left_callback,
                                        build_lidar_map);
    ros::Subscriber lidar_front_right_sub = build_lidar_map->nh.subscribe("/lidar_front_right", 
                                        1,
                                        &BuildLidarMap::lidar_front_right_callback,
                                        build_lidar_map);
    ros::Subscriber lidar_back_left_sub = build_lidar_map->nh.subscribe("/lidar_left", 
                                        1,
                                        &BuildLidarMap::lidar_back_left_callback,
                                        build_lidar_map);
    ros::Subscriber lidar_back_right_sub = build_lidar_map->nh.subscribe("/lidar_right", 
                                        1,
                                        &BuildLidarMap::lidar_back_right_callback,
                                        build_lidar_map);
    ros::Subscriber start_sub = build_lidar_map->nh.subscribe("start_autonomous", 
                                        1,
                                        &BuildLidarMap::start_callback,
                                        build_lidar_map);        

#endif 

    ros::spin();

    delete (build_lidar_map);
    return EXIT_SUCCESS;
}