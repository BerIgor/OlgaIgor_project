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
#include <math.h>



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
			ob::SE2StateSpace *space = new ob::SE2StateSpace();

//define bounds
			ob::RealVectorBounds bounds(2);
			bounds.setLow(0, 0);
			bounds.setHigh(0, ppm_.getWidth());

			bounds.setLow(1, 0);
			bounds.setHigh(1, ppm_.getHeight());

			//bounds.setLow(2, 0);
			//bounds.setHigh(2, 360);
			space->setBounds(bounds);			


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
		start[2] = 0;
        ob::ScopedState<> goal(ss_->getStateSpace());
        goal[0] = goal_row;
        goal[1] = goal_col;
		goal[2] = 0;
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
            const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::SE2StateSpace::StateType>()->getX());
            const int h = std::min(maxHeight_, (int)p.getState(i)->as<ob::SE2StateSpace::StateType>()->getY());
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


	void getOrders(){
		og::PathGeometric &p = ss_->getSolutionPath();

		updateYaws();

		//Accessing each waypoint on the path
		std::vector< ob::State * >& waypoints = p.getStates();
		for(int i=0; i<waypoints.size(); i++){
			ob::State* state=waypoints[i];
	        const double X = (double)state->as<ob::SE2StateSpace::StateType>()->getX();
            const double Y = (double)state->as<ob::SE2StateSpace::StateType>()->getY();
			std::cout<<"X="<<X<<" ; Y="<<Y<<std::endl;
		}		
	}


	/*
	 * Written by Igor Berendorf
	 * updateYaws
	 * Function will update the resulting yaws from the solution path
	 * and add them to the solution path as the 3rd dimension
	 */
	void updateYaws(){
		og::PathGeometric &p = ss_->getSolutionPath();
		//TODO: check to see that an actual solution has been found

		//Accessing each waypoint on the path
		std::vector< ob::State * >& waypoints = p.getStates();

		/* 
		 * This solution is not optimal. It uses {(#ofWaypoints-1) * 4} casts.
		 * A better solution would be to use variables to represent the previous state
		 */
		for(int i=0; i<waypoints.size()-1; i++){

			ob::State* current_state=waypoints[i];
			ob::State* next_state=waypoints[i+1];
	        const double X = (double)current_state->as<ob::SE2StateSpace::StateType>()->getX();
            const double Y = (double)current_state->as<ob::SE2StateSpace::StateType>()->getY();
	        const double next_X = (double)next_state->as<ob::SE2StateSpace::StateType>()->getX();
            const double next_Y = (double)next_state->as<ob::SE2StateSpace::StateType>()->getY();

			//unsigned double hypotenuse = sqrt(pow(X_diff, 2), pow(Y_diff, 2));

			//depends on the quadrant
			unsigned int rel_quadrant=getRelativeQuadrant(X, Y, next_X, next_Y);

			double Y_diff = abs(next_Y - Y);
			double X_diff = abs(next_X - X);

			double angle = atan2(Y_diff, X_diff) * 180/M_PI;
				
			double yaw=0;
			switch (rel_quadrant){
				case 0:
					yaw = angle;
					break;
				case 1:
					yaw = 180 - angle;
					break;
				case 2:
					yaw = 180 + angle;
					break;
				case 3:
					yaw = 360 - angle;
					break;								
				default:
					std::cout<<"ERROR"<<std::endl;				
			}//end of switch case

			//Update the state which will have this resulting yaw
			//next_state->as<ob::SE2StateSpace::StateType>()->setYaw(yaw);

		}//end of for
		return;
	}//end of calcYaws


    private:
    
    bool isStateValid(const ob::State *state) const {
        const int w = std::min((int)state->as<ob::SE2StateSpace::StateType>()->getX(), maxWidth_);
        const int h = std::min((int)state->as<ob::SE2StateSpace::StateType>()->getY(), maxHeight_);

        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
        return c.red > 127 && c.green > 127 && c.blue > 127;
    } //end of isStateValid
 
	/*
	* Written by Igor Berendorf
	* getRelativeQuadrant
	* NOTE: This function works with the messed up coordinate system ompl has in place
	* Parameters are X,Y of Origin, followed by X,Y of target coordinate
	* Returns the relative quadrant of Target to Origin (1 to 4)
	*/
	unsigned int getRelativeQuadrant(double x_ref, double y_ref, double x_tar, double y_tar){
		unsigned int quadrants[4]={1, 2, 3, 4};
		if( x_tar > x_ref ){
			quadrants[1] = quadrants[2] = 0;
		} else {
			quadrants[0] = quadrants[3] = 0;
		}
		//NOTE: Due to the Y axis being vertically flipped, the condition here is different
		if( y_tar < y_ref ){
			quadrants[0] = quadrants[1] = 0;
		} else {
			quadrants[2] = quadrants[3] = 0;
		}
		int i;
		for(i=0; i<4; i++){
			if (quadrants[i] == 0)
				continue;
		}
		return quadrants[i];
	}



    og::SimpleSetupPtr ss_;
    int maxWidth_;
    int maxHeight_;
    ompl::PPM ppm_;
    





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





