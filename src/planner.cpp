/*
Copying from: http://ompl.kavrakilab.org/Point2DPlanning_8cpp_source.html




*/



#include <ompl/base/spaces/SE2StateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


class Plane2DEnvironment{
    public:
    Plane2DEnvironment(const char* ppm_file){
        bool ok=false;
        try {   //opening the file
            ppm_.loadFile(ppm_file)
            ok = true;
        } catch (ompl::Exception &ex) {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
        }
        
        //TODO: fucking change this giant if around        
        if(ok) {
            ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();
            space->addDimension(0.0, ppm_.getWidth());
            space->addDimension(0.0, ppm_.getHeight());
            maxWidth_ = ppm_.getWidth() - 1;
            maxHeight_ = ppm_.getHeight() - 1;
            ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));
            // set state validity checking for this space
            ss_->setStateValidityChecker(std::bind(&Plane2DEnvironment::isStateValid, this, std::placeholders::_1));
            space->setup();
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
            //ss_->setPlanner(ob::PlannerPtr(new og::RRTConnect(ss_->getSpaceInformation())));
        }
    }   //end constructor

    bool plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col){
        if (!ss_){
            return false;
        }
        ob::ScopedState<> start(ss_->getStateSpace());
        start[0] = start_row;
        start[1] = start_col;
        ob::ScopedState<> goal(ss_->getStateSpace());
        goal[0] = goal_row;
        goal[1] = goal_col;
        ss_->setStartAndGoalStates(start, goal);
        // generate a few solutions; all will be added to the goal;
        for (int i = 0 ; i < 10 ; ++i){
            if (ss_->getPlanner()){
                ss_->getPlanner()->clear();
            }
            ss_->solve();
        }
        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (ss_->haveSolutionPath()){
            ss_->simplifySolution();
            og::PathGeometric &p = ss_->getSolutionPath();
            ss_->getPathSimplifier()->simplifyMax(p);
            ss_->getPathSimplifier()->smoothBSpline(p);
            return true;
        } else {
            return false;
        }
    } //end of plan




} //end class







int main(int argc, char* argv){




    return 0;
}


