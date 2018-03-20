#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

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

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>

namespace ob = ompl::base;
namespace oc = ompl::control;


class PathPlanning
{
public:
	PathPlanning(); 

	void plan(const ros::TimerEvent& event);
	void set_se2(const roguetwo_perception::SE2::ConstPtr& se2);

	ros::Publisher path_pub;
	ros::Subscriber se2_sub;

private:
	static bool isStateValid(
		const oc::SpaceInformation *si, 
		const ob::State *state
		);

	static void propagate(
		const ob::State *state,
		const oc::Control *control,
		const double duration,
		ob::State *result
		);

	std::vector<float> se2_state;
};

# endif