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

    // store current robot transformation matrix
    Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(this->curr_se2.yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> quat = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d r = quat.matrix();   
    
    this->robot_transformation << r(0,0), r(0,1), r(0,2), this->curr_se2.x,
                                  r(1,0), r(1,1), r(1,2), this->curr_se2.y,
                                  r(2,0), r(2,1), r(2,2), 0,
                                  0, 0, 0, 1;
}

Point BuildLidarMap::calculate_point(
    const float distance,
    const float angle,
    const Point offset
)
{
    // REMEBER X IS FORWARD, Y IS LEFT
    // transform coords in relation to the sensor
    float x_sensor = 0;
    if ((abs(this->curr_se2.yaw) + angle) > 1.5)
        x_sensor = -1 * distance * cos(angle);
    else 
        x_sensor = distance * cos(angle);
    float y_sensor = distance * sin(angle);

    // transform coords in relation to the robot center
    float x_robot = x_sensor - offset.x;

    float y_robot = 0;
    if (offset.y < 0)
        y_robot = y_sensor - abs(offset.y);
    else
        y_robot = y_sensor - offset.y;

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

#if 0
    std::cout << "Distance: " << distance << std::endl;
    std::cout << "XY Sensor: " << x_sensor << " " << y_sensor << std::endl;
    std::cout << "XY Robot: " << x_robot << " " << y_robot << std::endl;
    std::cout << "Transformed Point: " << Tp_r << std::endl;
    std::cout << "Map Point: " << map_point.x << " " << map_point.y << std::endl;
#endif

    return map_point;
} 


void BuildLidarMap::lidar_front_left_callback(const sensor_msgs::Range::ConstPtr& range)
{
    if ((range->max_range - range->range) > 0.5 &&
        (range->range - range->min_range) > 0.8)
    {
        Point point = this->calculate_point(range->range, 
                                            this->lidar1_angle,
                                            this->lidar1_offset);
        this->update_map(point);
    }
}


void BuildLidarMap::lidar_front_right_callback(const sensor_msgs::Range::ConstPtr& range)
{
    if ((range->max_range - range->range) > 0.5 &&
        (range->range - range->min_range) > 0.8)
    {
        Point point = this->calculate_point(range->range, 
                                            this->lidar2_angle,
                                            this->lidar2_offset);
        this->update_map(point);
    }
}


void BuildLidarMap::lidar_back_left_callback(const sensor_msgs::Range::ConstPtr& range)
{
    if ((range->max_range - range->range) > 0.5 &&
        (range->range - range->min_range) > 0.8)
    {
        Point point = this->calculate_point(range->range, 
                                            this->lidar3_angle,
                                            this->lidar3_offset);
        this->update_map(point);
    }
}


void BuildLidarMap::lidar_back_right_callback(const sensor_msgs::Range::ConstPtr& range)
{
    if ((range->max_range - range->range) > 0.5 &&
        (range->range - range->min_range) > 0.8)
    {
        Point point = this->calculate_point(range->range, 
                                            this->lidar4_angle,
                                            this->lidar4_offset);
        this->update_map(point);
    }
}

void BuildLidarMap::start_callback(const std_msgs::Bool::ConstPtr& start)
{
    this->map = MatrixXd::Zero(this->rows, this->cols);
    this->timer = this->nh.createTimer(ros::Duration(0.2),
                                      &BuildLidarMap::publish_map, 
                                       this);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "build_lidar_map");

    float resolution = 0.1;  // m
    int height = 20;
    int width = 20;
    float lidar1_angle = 0;
    float lidar2_angle = 0;
    float lidar3_angle = 0.70;
    float lidar4_angle = -0.80;

    // remeber x is FORWARD and y is LEFT
// Gazebo Setup
#if 1
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
    ros::Subscriber odometry_sub = build_lidar_map->nh.subscribe("/encoder/odometry", 
                                        1,
                                        &BuildLidarMap::odometry_callback,
                                        build_lidar_map);
    ros::Subscriber start_sub = build_lidar_map->nh.subscribe("start_autonomous", 
                                        1,
                                        &BuildLidarMap::start_callback,
                                        build_lidar_map);        
    
    ros::Publisher map_publisher = build_lidar_map->nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1);
    build_lidar_map->set_map_publisher(map_publisher);

    // ros::Timer timer = nh.createTimer(ros::Duration(0.2),
    //                                   &BuildLidarMap::publish_map, 
    //                                   build_lidar_map);

    ros::spin();

    delete (build_lidar_map);
    return EXIT_SUCCESS;
}