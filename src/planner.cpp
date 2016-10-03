/*
Copying from: http://ompl.kavrakilab.org/Point2DPlanning_8cpp_source.html
*/




#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/PPM.h>
#include <ompl/config.h>
//#include <../tests/resources/config.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <vector>
#include <boost/filesystem.hpp>
#include <iostream>




namespace ob = ompl::base;
namespace og = ompl::geometric;


class Plane2DEnvironment{
    public:
    Plane2DEnvironment(const char* ppm_file){
        bool ok=false;
        try {   //opening the file
            ppm_.loadFile(ppm_file);
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
        
        if (ss_->getPlanner()){
            ss_->getPlanner()->clear();
        }
        ss_->solve();  
        
        // generate a few solutions; all will be added to the goal;
        /*
        for (int i = 0 ; i < 10 ; ++i){
            if (ss_->getPlanner()){ //IGOR:this gets a random planner. We should choose the best planner. for more details visit http://ompl.kavrakilab.org/classompl_1_1geometric_1_1SimpleSetup.html#a8a94558b2ece27d938a92b062d55df71
                ss_->getPlanner()->clear();
            }
            ss_->solve();
        }
        */
        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (ss_->haveSolutionPath()){
			//Simplifying solution
            og::PathGeometric &p = ss_->getSolutionPath();
            og::PathSimplifierPtr& pathSimplifier = ss_->getPathSimplifier();//->simplifyMax(p);
            pathSimplifier->reduceVertices(p, 0, 0, 1);

			//TODO: add path shortening

            //Printing the solution
			std::cout << "Solution in base form" << std::endl;
            p.print(std::cout);
            std::cout << "Solution in matrix form" << std::endl;
            p.printAsMatrix(std::cout);


            return true;
        } else {
            return false;
        }
    } //end of plan

    void recordSolution(){
        if (!ss_ || !ss_->haveSolutionPath()){
            return;
        }
        og::PathGeometric &p = ss_->getSolutionPath();
        p.interpolate();
        for (std::size_t i = 0 ; i < p.getStateCount() ; ++i){
            const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            const int h = std::min(maxHeight_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            ompl::PPM::Color &c = ppm_.getPixel(h, w);
            c.red = 255;
            c.green = 0;
            c.blue = 0;
        }
    } //end of recordSolution


    void save(const char *filename){
        if (!ss_){
            return;
        }
        ppm_.saveFile(filename);
    } //end of save


    private:
    
    bool isStateValid(const ob::State *state) const {
        const int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
        const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);

        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
        return c.red > 127 && c.green > 127 && c.blue > 127;
    } //end of isStateValid
 
    og::SimpleSetupPtr ss_;
    int maxWidth_;
    int maxHeight_;
    ompl::PPM ppm_;
    
	void getOrders(){
		og::PathGeometric &p = ss_->getSolutionPath();

		//Accessing each waypoint on the path
		std::vector< ob::State * >& waypoints = p.getStates();
		for(int i=0; i<waypoints.size(); i++){
			ob::State* state=waypoints[i];
	        const double X = (double)state->as<ob::RealVectorStateSpace::StateType>()->values[0];
            const double Y = (double)state->as<ob::RealVectorStateSpace::StateType>()->values[1];
			std::cout<<"X="<<X<<" ; Y="<<Y<<std::endl;
		}		
	}	




}; //end of class
    
int main(int, char **){
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    //boost::filesystem::path path(TEST_RESOURCES_DIR);
//    Plane2DEnvironment env((path / "ppm/floor.ppm").string().c_str());
    Plane2DEnvironment env("/home/igor/robot_movement/OlgaIgor_project/gmaps/toConvert.ppm");
    if (env.plan(15, 15, 78, 57)){
		env.getOrders();
        env.recordSolution();
        env.save("reduce_vertices.ppm");
    }
    return 0;
}





