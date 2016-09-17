/*


*/
//#include <ompl/geometric/SimpleSetup.h>
//#include <ompl/base/StateSpace.h>

//#include <ompl/base/spaces/SE2StateSpace.h>
//#include <ompl/base/spaces/RealVectorBounds.h>

#include <iostream>
#include <ompl/base/spaces/DiscreteStateSpace.h>

//#include <ompl/base/StateSampler.h>

namespace ob = ompl::base;
//namespace og = ompl::geometric;
/*
bool isStateValid(const ob::State* state){
    return true;
}

*/

int main(int argc, char** argv){
    std::cout << "main() beginning" << std::endl;
    
    //create a simple space dim=2, bound=4
    ob::DiscreteStateSpace ds(4,4);        
    std::cout << ds.getLowerBound() << std::endl;


//   ob::RealVectorBounds rlvb(2);
    //rlvb.setLow(4);
    //rlvb.setHigh(4);
    
    //ob::StateSpacePtr space(new ob::SE2StateSpace());
    
    std::cout << "main() ending" << std::endl;
    

    return 0;
}
