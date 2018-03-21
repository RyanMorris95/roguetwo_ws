#include <iostream>
#include "path_planning.hpp"


PathPlanning::PathPlanning()
{
    // initalize vector
	for (int i=0; i < 3; i++)
	{
		se2_state.push_back(0);
	}
}


bool PathPlanning::isStateValid(
	const oc::SpaceInformation *si, 
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
    return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;

}


void PathPlanning::propagate(
	const ob::State *state,
	const oc::Control *control,
	const double duration, 
	ob::State *result
	)
{
	const ob::SE2StateSpace::StateType* se2state = state->as<ob::SE2StateSpace::StateType>();
	const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
	const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
	const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

	result->as<ob::SE2StateSpace::StateType>()->setXY(
		pos[0] + ctrl[0] * duration * cos(rot),
		pos[1] + ctrl[0] * duration * sin(rot));
	result->as<ob::SE2StateSpace::StateType>()->setYaw(
		rot + ctrl[1] * duration);
}


void PathPlanning::plan(const ros::TimerEvent& event)
{
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::SE2StateSpace());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // create a control space
    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);

    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set the state propagation routine
    ss.setStatePropagator(boost::bind(&propagate, _1, _2, _3, _4));

    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), _1));

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(-0.5);
    start->setY(0.0);
    start->setYaw(0.0);

    // create a  goal state; use the hard way to set the elements
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.5;
    (*goal)[1]->as<ob::SO2StateSpace::StateType>()->value = 0.0;


    // set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05);

    // ss.setPlanner(ob::PlannerPtr(new oc::PDST(ss.getSpaceInformation())));
    // ss.getSpaceInformation()->setMinMaxControlDuration(1,100);
    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen

        ss.getSolutionPath().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;

    oc::PathControl solution_path = ss.getSolutionPath();

    // convert ompl path to our custom Path message
    roguetwo_navigation::Path path_msg;

    std::vector<ob::State* >& states = solution_path.getStates();
    std::vector<oc::Control*>& controls = solution_path.getControls();
    std::vector<double>& control_durations = solution_path.getControlDurations();

    for (int i = 0; i != states.size(); i++)
    {
        double x = states[i]->as<ob::SE2StateSpace::StateType>()->getX();
        double y = states[i]->as<ob::SE2StateSpace::StateType>()->getY();
        double yaw = states[i]->as<ob::SE2StateSpace::StateType>()->getYaw();
        double x_control = controls[i]->as<oc::RealVectorControlSpace::ControlType>()->values[0];
        double y_control = controls[i]->as<oc::RealVectorControlSpace::ControlType>()->values[1];
        double yaw_control = controls[i]->as<oc::RealVectorControlSpace::ControlType>()->values[2];

        path_msg.x_states.push_back(x);
        path_msg.y_states.push_back(y);
        path_msg.yaw_states.push_back(yaw);
        path_msg.x_controls.push_back(x_control);
        path_msg.y_controls.push_back(y_control);
        path_msg.yaw_controls.push_back(yaw_control);
        path_msg.control_durations.push_back(control_durations[i]);
    }

    path_pub.publish(path_msg);

}


void PathPlanning::set_se2(const roguetwo_perception::SE2::ConstPtr& se2)
{
	se2_state[0] = se2->x;
	se2_state[1] = se2->y;
	se2_state[2] = se2->yaw;
}


int main(int argc, char **argv)
{
	std::cout << "OMPL Version: " << OMPL_VERSION << std::endl;
	ros::init(argc, argv, "path_planning");
	ros::NodeHandle nh;

	PathPlanning path_planning = PathPlanning();

	path_planning.path_pub = nh.advertise<roguetwo_navigation::Path>("path", 1);
	path_planning.se2_sub = nh.subscribe(
		"/se2_state",
		1,
		&PathPlanning::set_se2,
		&path_planning);

	// plan at 2 Hz
	ros::Timer timer = nh.createTimer(
		ros::Duration(0.5),
		&PathPlanning::plan,
		&path_planning);


	ros::spin();

	return 0;
}
