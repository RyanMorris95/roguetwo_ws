#ifndef ROUGETWO_WS_PATH_PLANNING_NODE_H
#define ROUGETWO_WS_PATH_PLANNING_NODE_H

#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <roguetwo_navigation/Path.h>
#include <roguetwo_navigation/Paths.h>
#include <roguetwo_perception/SE2.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_datatypes.h>

#include "dynamic_window_planner.h"

class PathPlanningNode
{
public:
	PathPlanningNode();

	void start_autonomous(std_msgs::Bool start);
	void update_obstacles(const nav_msgs::OccupancyGrid occuancy_grid);
	void update_se2(const nav_msgs::Odometry odometry);
	void generate_dynamic_window_path(const ros::TimerEvent& event);
	void set_planner(DynamicWindowPlanner& planner) { dynamic_window_planner = planner; }

	ros::NodeHandle nh;
	ros::Subscriber update_se2_sub;
	ros::Subscriber update_obstacles_sub;
	ros::Subscriber start_autonomous_sub;
	ros::Publisher paths_pub;
	ros::Publisher local_path_pub;

private:

	DynamicWindowPlanner dynamic_window_planner;
	RobotState robot_state;
	Point goal;
	Controls controls;
	std::vector<RobotState> trajectory;
	double delta_time;

	roguetwo_perception::SE2 se2;
	std::vector<Point> obstacles;

	ros::Timer timer;
};

#endif