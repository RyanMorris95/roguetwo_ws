//
// Created by ryan on 3/31/18.
//
#ifndef ROGUETWO_WS_DYNAMIC_WINDOW_PLANNER_H
#define ROGUETWO_WS_DYNAMIC_WINDOW_PLANNER_H

#include <iostream>
#include <math.h>
#include <algorithm>
#include <ros/ros.h>
#include <roguetwo_navigation/Path.h>

struct RobotState {
    double x;
    double y;
    double yaw;
    double velocity;
    double yaw_rate;

    RobotState() : x(0), y(0), yaw(0), velocity(0), yaw_rate(0) {}
};

struct Controls {
    double velocity;
    double yaw_rate;

    Controls() : velocity(0), yaw_rate(0) {}
};

struct DynamicWindow {
    double y_min;
    double y_max;
    double yaw_rate_min;
    double yaw_rate_max;

    DynamicWindow() : y_min(0), y_max(0), yaw_rate_min(0), yaw_rate_max(0) {}
};

struct Point {
    double x;
    double y;

    Point() : x(0), y(0) {}
};

class DynamicWindowPlanner
{
public:
    DynamicWindowPlanner();

    RobotState motion(RobotState robot_state,
                        Controls controls,
                        double delta_time);

    DynamicWindow calculate_dynamic_window(RobotState robot_state);

    std::vector<RobotState> calculate_trajectory(RobotState init_robot_state,
                                             double velocity,
                                             double yaw_rate);
    std::vector<RobotState> calculate_final_input(RobotState robot_state,
                                              Controls controls,
                                              DynamicWindow dynamic_window,
                                              Point goal,
                                              const std::vector<Point>& obstacles);

    double calculate_obstacle_cost(std::vector<RobotState> trajectory,
                                   const std::vector<Point>& obstacles);

    double calculate_to_goal_cost(std::vector<RobotState> trajectory,
                                  Point goal);

    std::vector<RobotState> plan(RobotState robot_state,
                                Controls controls,
                                Point goal,
                                std::vector<Point> obstacles);

    void print_trajectory(std::vector<RobotState> trajectory);

    Controls best_controls_;

private:
    // params
    double max_speed, min_speed;
    double max_yaw_rate;
    double max_acceleration;
    double max_yaw_acceleration;
    double velocity_resolution;
    double yaw_rate_resolution;
    double delta_time;
    double predict_time;
    double to_goal_cost_gain;
    double speed_cost_gain;
    double robot_radius;
    
};

#endif //ROGUETWO_WS_DYNAMIC_WINDOW_PLANNER_H
