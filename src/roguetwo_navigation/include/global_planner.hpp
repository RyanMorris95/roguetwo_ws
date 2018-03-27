#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <roguetwo_navigation/Path.h>
#include <roguetwo_perception/SE2.h>
#include <boost/scoped_ptr.hpp>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <limits>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;


class GlobalPlanner
{
public:
	GlobalPlanner(); 

	void plan(const ros::TimerEvent& event);
	void set_se2(const roguetwo_perception::SE2::ConstPtr& se2);

	ros::Publisher path_pub;
	ros::Subscriber se2_sub;

private:
	static bool isStateValid(
		const ob::SpaceInformation *si, 
		const ob::State *state
		);

	std::vector<float> se2_state;
};

# endif