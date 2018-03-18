#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
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
#include <iostream>

namespace ob = ompl::base;
namespace oc = ompl::control;


bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
	// cast the abstract state type to the type we expect
	const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

	// extract the first component of the state and cast it to what we expect
	const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

	// extract the second component of the state and cast it to what we expect
	const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

	// check validity of state defined by pos & rot

	// return a value that is always true but uses the two variables we define
	return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}


void propagate(
	const ob::State *state,
	const oc::Control *control,
	const double duration, 
	ob::State *result
	)
{
	const auto* se2state = state->as<ob::SE2StateSpace::StateType>();
	const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
	const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
	const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

	result->as<ob::SE2StateSpace::StateType>()->setXY(
		pos[0] + ctrl[0] * duration * cos(rot),
		pos[1] + ctrl[0] * duration * sin(rot));
	result->as<ob::SE2StateSpace::StateType>()->setYaw(
		rot + ctrl[1] * duration);
}


void plan()
{
	// construct the state space we are planning in
	auto space(std::make_shared<ob::SE2StateSpace>());

	// set the bounds for the R^2 part of SE(2)
	ob::RealVectorBounds bounds(2);
	bounds.setLow(-1);
	bounds.setHigh(1);

	space->setBounds(bounds);

	// create a control space
	auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

	// set the bounds for the control space
	ob::RealVectorBounds cbounds(2);
	cbounds.setLow(-0.3);
	cbounds.setHigh(0.3);

	// construct an instance of space information from this control space
	auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

	// set state validitity checking for this space
	si->setStateValidityChecker(
		[&si](const ob::State *state) {return isStateValid(si.get(), state); });

	// set the state propagation routine
	si->setStatePropagator(propagate);

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

	// set the state and goal states
	ss.setStartAndGoalStates(start, goal, 0.05);

	// attempt to solve the problem within one second of planning time
	ob::PlannerStatus solved = ss.solve(10.0);

	if (solved)
	{
		std::cout << "Found Solution: " << std::endl;
		ss.getSolutionPath().printAsMatrix(std::cout);
	}
	else
		std::cout << "No solution found" << std::endl;
}

int main(int argc, char **argv)
{
	std::cout << "OMPL Version: " << OMPL_VERSION << std::endl;

	planWithSimpleSetup();
}