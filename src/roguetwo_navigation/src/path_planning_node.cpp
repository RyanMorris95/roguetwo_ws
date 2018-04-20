#include "path_planning_node.h"


PathPlanningNode::PathPlanningNode()
{
	goal.x = 0;
	goal.y = 0;
	delta_time = 0.1;
}

void PathPlanningNode::start_autonomous(std_msgs::Bool start)
{
	goal.x = 0;
	goal.y = 0;
	std::cout << "Starting autonmous mode." << std::endl;
	timer = nh.createTimer(
			ros::Duration(0.1), 
			&PathPlanningNode::generate_dynamic_window_path,
			this,
			false);

}

void PathPlanningNode::update_se2(const nav_msgs::Odometry odometry)
{
	float x = odometry.pose.pose.position.x;
	float y = odometry.pose.pose.position.y;
	tf::Quaternion q(
		odometry.pose.pose.orientation.x,
		odometry.pose.pose.orientation.y,
		odometry.pose.pose.orientation.z,
		odometry.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	se2.x = x;
	se2.y = y;
	se2.yaw = yaw;
}

void PathPlanningNode::update_obstacles(const nav_msgs::OccupancyGrid occupancy_grid)
{
	obstacles.clear();

	for (int w = 0; w < occupancy_grid.info.width; w++)
	{
		for (int h = 0; h < occupancy_grid.info.height; h++)
		{
			if (occupancy_grid.data[h * occupancy_grid.info.width + w] == 100)
			{
				double y = w * occupancy_grid.info.resolution + occupancy_grid.info.resolution / 2.0;
				y += occupancy_grid.info.origin.position.x;
				double x = h * occupancy_grid.info.resolution + occupancy_grid.info.resolution / 2.0;
				x += occupancy_grid.info.origin.position.y;

				Point obstacle;
				obstacle.x = y;
				obstacle.y = x;

				obstacles.push_back(obstacle);
			}
		}
	}

}

void PathPlanningNode::generate_dynamic_window_path(const ros::TimerEvent& event)
{

	trajectory = dynamic_window_planner.plan(
		robot_state, 
		controls, 
		goal, 
		obstacles);

	robot_state = dynamic_window_planner.motion(robot_state, dynamic_window_planner.best_controls_, delta_time);

	robot_state.x = se2.x;
	robot_state.y = se2.y;
	robot_state.yaw = se2.yaw;

	double distance_from_goal = sqrt(pow((robot_state.x - goal.x), 2) +
										pow((robot_state.y - goal.y), 2));

	std::cout << "distance_from_goal: " << distance_from_goal << std::endl;

	roguetwo_navigation::Path path_msg;

	if (distance_from_goal < 0.5)
	{
		std::cout << "Finished!" << std::endl;
		path_msg.x_states.push_back(-100);
		path_msg.y_states.push_back(-100);
		path_msg.yaw_states.push_back(-100);
		timer.stop();
	} 
	else 
	{
		for (int i = 0; i < trajectory.size(); i++)
		{
			RobotState state = trajectory[i];
			path_msg.x_states.push_back(state.x);
			path_msg.y_states.push_back(state.y);
			path_msg.yaw_states.push_back(state.yaw);
		}
	}

	local_path_pub.publish(path_msg);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_planning_node");
	std::cout << "Created path planning node" << std::endl;
	PathPlanningNode path_planning_node = PathPlanningNode();

	path_planning_node.update_se2_sub = path_planning_node.nh.subscribe(
		"/encoder/odometry", 
		1, 
		&PathPlanningNode::update_se2, 
		&path_planning_node);

	// path_planning_node.update_obstacles_sub = path_planning_node.nh.subscribe(
	// 	"/projected_map",
	// 	1,
	// 	&PathPlanningNode::update_obstacles,
	// 	&path_planning_node);

	path_planning_node.start_autonomous_sub = path_planning_node.nh.subscribe(
		"/start_autonomous",
		1,
		&PathPlanningNode::start_autonomous,
		&path_planning_node);

	path_planning_node.local_path_pub = path_planning_node.nh.advertise<roguetwo_navigation::Path>("local_path", 1);

	// ros::Timer timer = path_planning_node.nh.createTimer(
	// 	ros::Duration(0.1), 
	// 	&PathPlanningNode::generate_dynamic_window_path,
	// 	&path_planning_node,
	// 	false);

	ros::spin();
}
