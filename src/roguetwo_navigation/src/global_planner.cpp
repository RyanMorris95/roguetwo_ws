#include <iostream>
#include "global_planner.hpp"


GlobalPlanner::GlobalPlanner()
{
    // initalize vector
	for (int i=0; i < 3; i++)
	{
		se2_state.push_back(0);
	}
}


bool GlobalPlanner::isStateValid(
	const ob::SpaceInformation *si, 
	const ob::State *state
	)
{
    //    ob::ScopedState<ob::SE2StateSpace>
    // cast the abstract state type to the type we expect
    const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const ob::SO2StateSpace::StateType *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    // check validity of state defined by pos & rot


    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (rot->value < 0.436332 && rot->value > -0.436332);  // limit rotation to +- some degrees
}


void GlobalPlanner::plan(const ros::TimerEvent& event)
{
    // construct the state space we are planning in
    ob::StateSpacePtr space(boost::make_shared<ob::ReedsSheppStateSpace>());
    //ob::StateSpacePtr space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(30);

    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // Use geometric planners
    og::SimpleSetup ss(space);

    ss.setStateValidityChecker(
        [&ss](const ob::State *state) { return isStateValid(ss.getSpaceInformation().get(), state); });

    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(0.0);
    start->setY(0.0);
    start->setYaw(0.0);

    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(25.0);
    goal->setY(0.0);
    goal->setYaw(0.0);

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05);

    ss.setPlanner(ob::PlannerPtr(boost::make_shared<og::InformedRRTstar>(ss.getSpaceInformation())));
    // ss.getSpaceInformation()->setMinMaxControlDuration(1,100);
    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(0.5);


    ss.simplifySolution();
    og::PathGeometric solution_path = ss.getSolutionPath();
    //solution_path.interpolate(100);

    // // convert ompl path to our custom Path message
    roguetwo_navigation::Path path_msg;

    std::vector<ob::State* >& states = solution_path.getStates();

    for (int i = 0; i != states.size(); i++)
    {
        double x = states[i]->as<ob::SE2StateSpace::StateType>()->getX();
        double y = states[i]->as<ob::SE2StateSpace::StateType>()->getY();
        double yaw = states[i]->as<ob::SE2StateSpace::StateType>()->getYaw();

        path_msg.x_states.push_back(x);
        path_msg.y_states.push_back(y);
        path_msg.yaw_states.push_back(yaw);
    }

    path_pub.publish(path_msg);
}


void GlobalPlanner::set_se2(const roguetwo_perception::SE2::ConstPtr& se2)
{
	se2_state[0] = se2->x;
	se2_state[1] = se2->y;
	se2_state[2] = se2->yaw;
}


int main(int argc, char **argv)
{
	std::cout << "OMPL Version: " << OMPL_VERSION << std::endl;
	ros::init(argc, argv, "global_planner");
	ros::NodeHandle nh;

	GlobalPlanner global_planner = GlobalPlanner();

	global_planner.path_pub = nh.advertise<roguetwo_navigation::Path>("global_path", 1);
	global_planner.se2_sub = nh.subscribe(
		"/se2_state",
		1,
		&GlobalPlanner::set_se2,
		&global_planner);

	// // plan at 2 Hz
	ros::Timer timer = nh.createTimer(
		ros::Duration(1),
		&GlobalPlanner::plan,
		&global_planner);

	ros::spin();

	return 0;
}
