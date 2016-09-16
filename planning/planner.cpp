#include "planner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;



void planWithSimpleSetup(void){
	ob::StateSpacePtr space(new ob::SEStateSpace());
	//

	og::SimpleSetup ss(space);
	ss.setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
	ob::ScopedState<> start(space);
	//set start state	
	start.random();
	
	//add random goal
	ob::ScopedState<> goal(space);
	goal.random();

	//set states as start and goal states
	ss.setStartAndGoalStates(start, goal);
	
	ob::PlannerStatus solved = ss.solve(1.0);

	if(solved){
		std::cout << "found solution: "<< std::endl;
		ss.simplifySolution();
		ss.getSolutionPath().print(std::cout);
	}
}

bool isStateValid(const ob::State *state){
	


}


bool myStateValidityCheckerFunction(const base::State *state)
{
     return ...;
}











