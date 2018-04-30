//
// Created by ryan on 3/31/18.
//
#include "dynamic_window_planner.h"


DynamicWindowPlanner::DynamicWindowPlanner()
{

}

DynamicWindowPlanner::DynamicWindowPlanner(
    double _max_speed,
    double _min_speed,
    double _max_yaw_rate,
    double _max_acceleration,
    double _max_yaw_acceleration,
    double _velocity_resolution,
    double _yaw_rate_resolution,
    double _delta_time,
    double _predict_time,
    double _to_goal_cost_gain,
    double _speed_cost_gain,
    double _obstacle_cost_gain,
    double _robot_radius
)
  : max_speed(_max_speed),
    min_speed(_min_speed),
    max_yaw_rate(_max_yaw_rate),
    max_acceleration(_max_acceleration),
    max_yaw_acceleration(_max_yaw_acceleration),
    velocity_resolution(_velocity_resolution),
    yaw_rate_resolution(_yaw_rate_resolution),
    delta_time(_delta_time),
    predict_time(_predict_time),
    to_goal_cost_gain(_to_goal_cost_gain),
    speed_cost_gain(_speed_cost_gain),
    obstacle_cost_gain(_obstacle_cost_gain),
    robot_radius(_robot_radius)
{
    max_yaw_rate = max_yaw_rate * M_PI / 180.0;
    max_yaw_acceleration = max_yaw_acceleration * M_PI / 180.0;
    yaw_rate_resolution = yaw_rate_resolution * M_PI / 180.0;
}

// Finds the next state of the robot based on the control inputs.
// Only used for simulations.
//
// @param robot_state: current SE2 state of the robot
// @param controls: speed and steering angle control input
// @param delta_time: how long to apply these controls
// @return robot_state: the new robot state 
RobotState DynamicWindowPlanner::motion(
    RobotState robot_state,
    Controls controls,
    double delta_time)
{
    robot_state.x += (controls.velocity * cos(robot_state.yaw)) * delta_time;
    robot_state.y += (controls.velocity * sin(robot_state.yaw)) * delta_time;
    robot_state.yaw += controls.yaw_rate * delta_time;
    robot_state.velocity = controls.velocity;
    robot_state.yaw_rate = controls.yaw_rate;

    return robot_state;
}


//  Calculates the dynamic window using the robot footprint
//  and current motion states.
//
//  @param robot_state: current motion state of the robot
//  @return: dynamic window
DynamicWindow DynamicWindowPlanner::calculate_dynamic_window(RobotState robot_state)
{
    DynamicWindow dynamic_window_robot;
    dynamic_window_robot.y_min = min_speed;
    dynamic_window_robot.y_max = max_speed;
    dynamic_window_robot.yaw_rate_min = -1*max_yaw_rate;
    dynamic_window_robot.yaw_rate_max = max_yaw_rate;

    DynamicWindow dynamic_window_motion;
    dynamic_window_motion.y_min = robot_state.velocity - max_acceleration * delta_time;
    dynamic_window_motion.y_max = robot_state.velocity + max_acceleration * delta_time;
    dynamic_window_motion.yaw_rate_min = robot_state.yaw_rate - max_yaw_acceleration * delta_time;
    dynamic_window_motion.yaw_rate_max = robot_state.yaw_rate + max_yaw_acceleration * delta_time;

    // the final dynamic window is the minimum of the robot and motion windows
    DynamicWindow dynamic_window_final;
    dynamic_window_final.y_min = std::max(dynamic_window_robot.y_min, dynamic_window_motion.y_min);
    dynamic_window_final.y_max = std::min(dynamic_window_robot.y_max, dynamic_window_motion.y_max);
    dynamic_window_final.yaw_rate_min = std::max(dynamic_window_robot.yaw_rate_min, dynamic_window_motion.yaw_rate_min);
    dynamic_window_final.yaw_rate_max = std::min(dynamic_window_robot.yaw_rate_max, dynamic_window_motion.yaw_rate_max);

    return dynamic_window_final;
}


//  Calculates the trajectory for the robot to follow. 
//
//  @param robot_state: current motion state of the robot
//  @param velocity:  control input velocity
//  @param yaw_rate: control input yaw rate
//  @return: vector of motion states which is the trajectory
std::vector<RobotState> DynamicWindowPlanner::calculate_trajectory(
        RobotState robot_state,
        double velocity,
        double yaw_rate)
{
    double time = 0;
    Controls controls;
    controls.velocity = velocity;
    controls.yaw_rate = yaw_rate;

    std::vector<RobotState> trajectory;
    trajectory.push_back(robot_state);

    while (time <= predict_time)
    {
        robot_state = DynamicWindowPlanner::motion(robot_state, 
                                                    controls, 
                                                    delta_time);
        trajectory.push_back(robot_state);
        time += delta_time;
    }

    return trajectory;
}

// Calculates the best trajectory by finding the trajectory 
// with the lowest cost.
// 
// @param robot_state: current state of robot
// @param controls: current controls of the robot
// @param dynamic_window: the window of yaws and velocities considered
// @param goal: x, y goal 
// @param obstacles: vector of points representing obstacles
// @return best_trajectory: vector of robot states to follow
std::vector<RobotState> DynamicWindowPlanner::calculate_final_input(
        RobotState robot_state,
        Controls controls,
        DynamicWindow dynamic_window,
        Point goal,
        const std::vector<Point>& obstacles,
        std::vector<std::vector<RobotState>>& all_trajectories)
{
    double min_cost = std::numeric_limits<double>::max();
    double best_goal, best_speed;
    double goal_distance, angle_to_goal;
    std::vector<RobotState> best_trajectory;
    double obstacle_cost = 1.0;

    all_trajectories.clear();

    #pragma omp parallel for
    for (double v = dynamic_window.y_min; 
        v <= dynamic_window.y_max; 
        v += velocity_resolution)
    {
        #pragma omp parallel for
        for (double y = dynamic_window.yaw_rate_min; 
            y <= dynamic_window.yaw_rate_max;
            y += yaw_rate_resolution)
        {
            std::vector<RobotState> trajectory = calculate_trajectory(robot_state, 
                                                                        v, 
                                                                        y);
            double to_goal_cost = to_goal_cost_gain * calculate_to_goal_cost(trajectory, 
                                                                            goal, 
                                                                            robot_state,
                                                                            goal_distance,
                                                                            angle_to_goal);

            double speed_cost = speed_cost_gain * (max_speed - trajectory[-1].velocity);

            obstacle_cost = obstacle_cost_gain * calculate_obstacle_cost(trajectory, obstacles);

            double final_cost = to_goal_cost + speed_cost + obstacle_cost;

            all_trajectories.push_back(trajectory);

            // search for minimum trajectory
            if (min_cost >= final_cost)
            {
                min_cost = final_cost;
                best_controls_.velocity = v;
                best_controls_.yaw_rate = y;
                best_goal = to_goal_cost;
                best_speed = speed_cost;
                best_trajectory = trajectory;
            }
        }
    }

    // std::cout << "********************************************" << std::endl;
    // std::cout << "Best To Goal:  " << best_goal << std::endl;
    // std::cout << "Best Speed: " << best_speed << std::endl;
    // std::cout << "Goal Distance: " << goal_distance << std::endl;
    // std::cout << "Angle to goal: " << angle_to_goal << std::endl;
    // std::cout << "********************************************" << std::endl;

    return best_trajectory;
}

// Loop through all the obstacles and make sure that 
// the current trajectory isn't within the robot raidus
// of an obstacle.
//
// @param trajectory: vector of robot states 
// @param obstacles: vector of points representing obstacles
// @return : cost of input trajectory
double DynamicWindowPlanner::calculate_obstacle_cost(
        std::vector<RobotState> trajectory,
        const std::vector<Point>& obstacles)
{
    int skip = 2;
    double min_distance = std::numeric_limits<double>::max();

    if (obstacles.size() > 0)
    {
        #pragma omp parallel for
        for (int i = 0; i < trajectory.size(); i+=skip)
        {
            for (int j = 0; j < obstacles.size(); j++)
            {
                Point obstacle = obstacles[j];
                RobotState current_robot_state = trajectory[i];

                // calculate distance between trajectory point and obstacle
                double dx = current_robot_state.x - obstacle.x;
                double dy = current_robot_state.y - obstacle.y;
                double distance = sqrt(pow(dx, 2) + pow(dy, 2));
                if (distance <= robot_radius)  // collision
                    break;

                if (min_distance >= robot_radius)
                {
                    min_distance = distance;
                }
            }
        }
    }

    return double(1.0 / min_distance);

}

// Find the cost of the trajectory to the goal by
// scaling the eculidan distance to the goal point 
// by the gain.
//
// @param trajectory: vector of robot states
// @param goal: x, y point 
// @param curr_state: current state of the robot
// @return cost: scaled euclidean distance representing the cost
double DynamicWindowPlanner::calculate_to_goal_cost(
        std::vector<RobotState> trajectory,
        Point goal,
        RobotState curr_state,
        double& goal_distance,
        double& angle_to_goal)
{
    double dx = goal.x - trajectory.back().x;
    double dy = goal.y - trajectory.back().y;
    goal_distance = sqrt(pow(dx, 2) + pow(dy, 2));

    angle_to_goal = atan2(goal.y - trajectory.back().y, 
                                 goal.x - trajectory.back().x);

    double cost = angle_to_goal + goal_distance;

    return cost;
}

// Top level function to run dynamic window path planner
//
// @param robot_state: current state of robot
// @param controls: current controls of robot
// @param goal: x, y goal point
// @param obstacles: vector points representing obstacles
// @return trajectory: lowest cost vector of robot states to follow
std::vector<RobotState> DynamicWindowPlanner::plan(
        RobotState robot_state,
        Controls controls,
        Point goal,
        std::vector<Point> obstacles,
        std::vector<std::vector<RobotState>>& all_trajectories)
{
    DynamicWindow dynamic_window = calculate_dynamic_window(robot_state);

    std::vector<RobotState> trajectory = calculate_final_input(
        robot_state,
        controls,
        dynamic_window,
        goal,
        obstacles,
        all_trajectories);

    return trajectory;
}


void DynamicWindowPlanner::print_trajectory(std::vector<RobotState> trajectory)
{
    for (int i = 0; i < trajectory.size(); i++)
    {
        RobotState robot_state = trajectory[i];
        std::cout << "Robot State " << i << " : "
            << robot_state.x << "\t" << robot_state.y << "\t"
            << robot_state.yaw << "\t" << robot_state.velocity 
            << "\t" << robot_state.yaw_rate << std::endl;
    }
}
