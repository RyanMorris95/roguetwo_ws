//
// Created by ryan on 3/31/18.
//
#include "dynamic_window_planner.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

DynamicWindowPlanner::DynamicWindowPlanner()
{
    max_speed = 0.70;
    min_speed = -0.50;
    max_yaw_rate = 45 * M_PI / 180.0;
    max_acceleration = 0.4;
    max_yaw_acceleration = 45 * M_PI / 180.0;
    velocity_resolution = 0.05;
    yaw_rate_resolution = 0.5 * M_PI / 180.0;
    delta_time = 0.1;
    predict_time = 7.0;
    to_goal_cost_gain = 1.25;
    speed_cost_gain = 1.0;
    robot_radius = 0.31;
}


RobotState DynamicWindowPlanner::motion(
    RobotState robot_state,
    Controls controls,
    double delta_time)
{
    robot_state.x += controls.velocity * cos(robot_state.yaw) * delta_time;
    robot_state.y += controls.velocity * sin(robot_state.yaw) * delta_time;
    robot_state.yaw += controls.yaw_rate * delta_time;
    robot_state.velocity = controls.velocity;
    robot_state.yaw_rate = controls.yaw_rate;

    return robot_state;
}

/**
    Calculates the dynamic window using the robot footprint
    and current motion states.

    @param robot_state: current motion state of the robot
    @return: dynamic window
*/
DynamicWindow DynamicWindowPlanner::calculate_dynamic_window(RobotState robot_state)
{
    DynamicWindow dynamic_window_robot;
    dynamic_window_robot.y_min = min_speed;
    dynamic_window_robot.y_max = max_speed;
    dynamic_window_robot.yaw_rate_min = -1*max_yaw_rate;
    dynamic_window_robot.yaw_rate_max = max_yaw_rate;
    // std::cout << "dynamic window robot" << std::endl;
    // std::cout << "Y_min: " << dynamic_window_robot.y_min 
    // << " Y_max: " << dynamic_window_robot.y_max 
    // << " yaw_rate_min: " << dynamic_window_robot.yaw_rate_min
    // << " Y_max: " << dynamic_window_robot.yaw_rate_max << std::endl;

    DynamicWindow dynamic_window_motion;
    dynamic_window_motion.y_min = robot_state.velocity - max_acceleration * delta_time;
    dynamic_window_motion.y_max = robot_state.velocity + max_acceleration * delta_time;
    dynamic_window_motion.yaw_rate_min = robot_state.yaw_rate - max_yaw_acceleration * delta_time;
    dynamic_window_motion.yaw_rate_max = robot_state.yaw_rate + max_yaw_acceleration * delta_time;
    // std::cout << "dynamic window motion" << std::endl;
    // std::cout << "Y_min: " << dynamic_window_motion.y_min 
    // << " Y_max: " << dynamic_window_motion.y_max 
    // << " yaw_rate_min: " << dynamic_window_motion.yaw_rate_min
    // << " Y_max: " << dynamic_window_motion.yaw_rate_max << std::endl;

    // the final dynamic window is the minimum of the robot and motion windows
    DynamicWindow dynamic_window_final;
    dynamic_window_final.y_min = std::max(dynamic_window_robot.y_min, dynamic_window_motion.y_min);
    dynamic_window_final.y_max = std::min(dynamic_window_robot.y_max, dynamic_window_motion.y_max);
    dynamic_window_final.yaw_rate_min = std::max(dynamic_window_robot.yaw_rate_min, dynamic_window_motion.yaw_rate_min);
    dynamic_window_final.yaw_rate_max = std::min(dynamic_window_robot.yaw_rate_max, dynamic_window_motion.yaw_rate_max);

    return dynamic_window_final;
}

/**
    Calculates the trajectory for the robot to follow. 

    @param robot_state: current motion state of the robot
    @param velocity:  control input velocity
    @param yaw_rate: control input yaw rate
    @return: vector of motion states which is the trajectory
*/
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

std::vector<RobotState> DynamicWindowPlanner::calculate_final_input(
        RobotState robot_state,
        Controls controls,
        DynamicWindow dynamic_window,
        Point goal,
        const std::vector<Point>& obstacles)
{
    double min_cost = 10000.0;
    std::vector<RobotState> best_trajectory;
    double obstacle_cost = 1.0;


    for (double v = dynamic_window.y_min; 
        v <= dynamic_window.y_max; 
        v += velocity_resolution)
    {
        for (double y = dynamic_window.yaw_rate_min; 
            y <= dynamic_window.yaw_rate_max;
            y += yaw_rate_resolution)
        {
            std::vector<RobotState> trajectory = calculate_trajectory(robot_state, 
                                                                        v, 
                                                                        y);
            // print_trajectory(trajectory);

            double to_goal_cost = calculate_to_goal_cost(trajectory, goal);

            //std::cout << "Goal Cost: " << to_goal_cost << std::endl;

            double speed_cost = speed_cost_gain * (max_speed - trajectory[-1].velocity);
            //std::cout << "Speed Cost: " << speed_cost << std::endl;

            obstacle_cost = calculate_obstacle_cost(trajectory, obstacles);
            //std::cout << "obstacle Cost: " << obstacle_cost << std::endl;

            double final_cost = to_goal_cost + speed_cost + obstacle_cost;

            //std::cout << "Final Cost: " << final_cost << std::endl;

            // search for minimum trajectory
            if (min_cost >= final_cost)
            {
                min_cost = final_cost;
                best_controls_.velocity = v;
                best_controls_.yaw_rate = y;
                best_trajectory = trajectory;
            }
        }
    }

    return best_trajectory;
}

double DynamicWindowPlanner::calculate_obstacle_cost(
        std::vector<RobotState> trajectory,
        const std::vector<Point>& obstacles)
{
    int skip = 2;
    double min_distance = 1e33;

    if (obstacles.size() > 1)
    {
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

double DynamicWindowPlanner::calculate_to_goal_cost(
        std::vector<RobotState> trajectory,
        Point goal)
{
    // RobotState robot_state = trajectory.back();
    // std::cout << "Robot State " << " : "
    //         << robot_state.x << "\t" << robot_state.y << "\t"
    //         << robot_state.yaw << "\t" << robot_state.velocity 
    //         << "\t" << robot_state.yaw_rate << std::endl;
    //std::cout << trajectory.back().x << "\t" << trajectory.back().y << std::endl;
    double dx = goal.x - trajectory.back().x;
    double dy = goal.y - trajectory.back().y;
    //std::cout << "Dx: " << dx << " Dy: " << dy << std::endl;
    double goal_distance = sqrt(pow(dx, 2) + pow(dy, 2));
    //std::cout << "Goal distance: " << goal_distance << std::endl;
    double cost = to_goal_cost_gain * goal_distance;

    return cost;
}

std::vector<RobotState> DynamicWindowPlanner::plan(
        RobotState robot_state,
        Controls controls,
        Point goal,
        std::vector<Point> obstacles)
{
    DynamicWindow dynamic_window = calculate_dynamic_window(robot_state);

    std::vector<RobotState> trajectory = calculate_final_input(
        robot_state,
        controls,
        dynamic_window,
        goal,
        obstacles);

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