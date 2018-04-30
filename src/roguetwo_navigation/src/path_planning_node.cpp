#include "path_planning_node.h"

PathPlanningNode::PathPlanningNode()
{
	goal.x = 0;
	goal.y = 0;
	delta_time = 0.1;
}

// When autnomous mode is initiated start timer that 
// continuously calls path planning.
//
// @param start: ros bool message
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

// Updates the SE2 state of the robot based on the latest
// odometry reading.
//
// @param odometry: latest ros odometry message
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

// Builds the obstacles vector from an occupancy grid.
//
// @param: latest ros occupancy grid message
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
				y -= occupancy_grid.info.origin.position.y;
				double x = h * occupancy_grid.info.resolution + occupancy_grid.info.resolution / 2.0;
				x -= occupancy_grid.info.origin.position.x;

				Point obstacle;
				obstacle.x = y;
				obstacle.y = x;

				obstacles.push_back(obstacle);
			}
		}
	}

}

// Runs dynamic path planning at every timer event.
//
// @param event: timer event
void PathPlanningNode::generate_dynamic_window_path(const ros::TimerEvent& event)
{
	std::vector<std::vector<RobotState>> all_trajectories;

	trajectory = dynamic_window_planner.plan(
		robot_state, 
		controls, 
		goal, 
		obstacles,
		all_trajectories);

	robot_state = dynamic_window_planner.motion(robot_state, dynamic_window_planner.best_controls_, delta_time);

	robot_state.x = se2.x;
	robot_state.y = se2.y;
	robot_state.yaw = se2.yaw;

	double distance_from_goal = sqrt(pow((robot_state.x - goal.x), 2) +
										pow((robot_state.y - goal.y), 2));

	std::cout << "distance_from_goal: " << distance_from_goal << std::endl;

	roguetwo_navigation::Path path_msg;

	if (distance_from_goal < 2.0)
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

	roguetwo_navigation::Paths paths_msg;
	for (auto traj : all_trajectories)
	{
		roguetwo_navigation::Path path_msg;
		for (int i = 0; i < traj.size(); i++)
		{
			RobotState state = traj[i];
			path_msg.x_states.push_back(state.x);
			path_msg.y_states.push_back(state.y);
			path_msg.yaw_states.push_back(state.yaw);
		}
		paths_msg.paths.push_back(path_msg);
	}

	paths_pub.publish(paths_msg);
	local_path_pub.publish(path_msg);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_planning_node");
	std::cout << "Created path planning node" << std::endl;
	PathPlanningNode path_planning_node = PathPlanningNode();
	path_planning_node.nh = ros::NodeHandle("~");

    double max_speed;
    double min_speed;
    double max_yaw_rate;
    double max_acceleration;
    double max_yaw_acceleration;
    double velocity_resolution;
    double yaw_rate_resolution;
    double delta_time;
    double predict_time;
    double to_goal_cost_gain;
    double speed_cost_gain;
	double obstacle_cost_gain;
    double robot_radius;

	path_planning_node.nh.getParam("max_speed", max_speed);
	path_planning_node.nh.getParam("min_speed", min_speed);
	path_planning_node.nh.getParam("max_yaw_rate", max_yaw_rate);
	path_planning_node.nh.getParam("max_acceleration", max_acceleration);
	path_planning_node.nh.getParam("max_yaw_acceleration", max_yaw_acceleration);
	path_planning_node.nh.getParam("velocity_resolution", velocity_resolution);
	path_planning_node.nh.getParam("yaw_rate_resolution", yaw_rate_resolution);
	path_planning_node.nh.getParam("delta_time", delta_time);
	path_planning_node.nh.getParam("predict_time", predict_time);
	path_planning_node.nh.getParam("to_goal_cost_gain", to_goal_cost_gain);
	path_planning_node.nh.getParam("speed_cost_gain", speed_cost_gain);
	path_planning_node.nh.getParam("obstacle_cost_gain", obstacle_cost_gain);
	path_planning_node.nh.getParam("robot_radius", robot_radius);

	
	DynamicWindowPlanner dynamic_window_planner = DynamicWindowPlanner(max_speed,
																	min_speed,
																	max_yaw_rate,
																	max_acceleration,
																	max_yaw_acceleration,
																	velocity_resolution,
																	yaw_rate_resolution,
																	delta_time,
																	predict_time,
																	to_goal_cost_gain,
																	speed_cost_gain,
																	obstacle_cost_gain,
																	robot_radius);

	path_planning_node.set_planner(dynamic_window_planner);

	path_planning_node.update_se2_sub = path_planning_node.nh.subscribe(
		"/encoder/odometry", 
		1, 
		&PathPlanningNode::update_se2, 
		&path_planning_node);

	path_planning_node.update_obstacles_sub = path_planning_node.nh.subscribe(
		"/occupancy_grid",
		1,
		&PathPlanningNode::update_obstacles,
		&path_planning_node);

	path_planning_node.start_autonomous_sub = path_planning_node.nh.subscribe(
		"/start_autonomous",
		1,
		&PathPlanningNode::start_autonomous,
		&path_planning_node);

	path_planning_node.local_path_pub = path_planning_node.nh.advertise<roguetwo_navigation::Path>("local_path", 1);
	path_planning_node.paths_pub = path_planning_node.nh.advertise<roguetwo_navigation::Paths>("paths", 1);


	// ros::Timer timer = path_planning_node.nh.createTimer(
	// 	ros::Duration(0.1), 
	// 	&PathPlanningNode::generate_dynamic_window_path,
	// 	&path_planning_node,
	// 	false);

	ros::spin();
}
