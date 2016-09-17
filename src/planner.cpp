/*

*/

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/StateSpace.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <iostream>


namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State* state){
    return true;
}

/*
void planWithSimpleSetup(void){
    ob::StateSpacePtr space(new ob::SE2StateSpace());
    //is setting bounds required?
    
    og::SimpleSetup ss(space);
    
    //assign state validity checker
    ss.setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
    
    ob::ScopedState<> start(space);
    start.random();
    
}
*/


int main(int argc, char* argv){
    std::cout << "main() beginning" << std::endl;
    ob::RealVectorBounds rlvb(2);
    rlvb.setLow(4);
    rlvb.setHigh(4);
    
    ob::StateSpacePtr space(new ob::SE2StateSpace());
    
    std::cout << "main() ending" << std::endl;
    

    return 0;
}