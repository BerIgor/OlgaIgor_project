/*
Copying from: http://ompl.kavrakilab.org/Point2DPlanning_8cpp_source.html
*/


/*

HOW TO USE:

<.exe> <path to map> <robot radius> <start x> <start y> <end x> <end y>


*/



#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/PPM.h>
#include <ompl/config.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <vector>
#include <boost/filesystem.hpp>
#include <iostream>
#include <math.h>
#include <ompl/base/StateValidityChecker.h>
#include <string>

#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
//added for pixel weight optimization
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/ProblemDefinition.h>
////////
#define MAX_COLOR 254
#define MIN_COLOR 0





namespace ob = ompl::base;
namespace og = ompl::geometric;


class MyStateCostIntegralObjective : public ob::StateCostIntegralObjective {
private:
	ompl::PPM ppm_;

public:
	MyStateCostIntegralObjective(const ob::SpaceInformationPtr& si, ompl::PPM ppm) : ob::StateCostIntegralObjective(si, true) {
		description_="rgb based state weight";
		//TODO see about segment count factor
		ppm_=ppm;
	}

	ob::Cost stateCost(const ob::State* state) const {		
        const int w = (int)state->as<ob::SE2StateSpace::StateType>()->getX();
        const int h = (int)state->as<ob::SE2StateSpace::StateType>()->getY();
        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
		
		//TODO: add check that all colors are equal
		double weight = MAX_COLOR+1 - c.red;
		
		return ob::Cost(weight);
	}
};




class MyStateValidityChecker : public ob::StateValidityChecker {
private:
	ompl::PPM ppm_;
	og::SimpleSetupPtr ss_;
	int robotRadius_;
    int maxWidth_;
    int maxHeight_;

public:
	//c'tor
	MyStateValidityChecker(ompl::PPM ppm, const ob::SpaceInformationPtr &si, int robotRadius) :
			 ob::StateValidityChecker(si) {
		robotRadius_ = robotRadius;
		ppm_=ppm;	
		maxWidth_ = ppm_.getWidth() - 1;
		maxHeight_ = ppm_.getHeight() - 1;
	}

	//For future reference: note that clearance is not implemented in a sensible way by ompl
	virtual bool isValid(const ob::State* state) const {
	    const int w = (int)state->as<ob::SE2StateSpace::StateType>()->getX();
	    const int h = (int)state->as<ob::SE2StateSpace::StateType>()->getY();
		//for all x
		for(int x = w - robotRadius_; x <= w + robotRadius_; x++){
			if(x > maxWidth_ || x<0) {
				continue;
			}
			//for all y
			for(int y = h - robotRadius_; y <= h + robotRadius_; y++){
				if(y > maxHeight_ || y<0) {
					continue;
				}

				//get color
				const ompl::PPM::Color &c = ppm_.getPixel(y, x);
				//check color
				if( c.red > 127 && c.green > 127 && c.blue > 127){
					//if ok continue
					continue;
				} else {
					return false;
				}
			}
		}
		return true;
	}//end of isValid

};




class Plane2DEnvironment{
private:
	ompl::PPM ppm_;
    og::SimpleSetupPtr ss_;
    int maxWidth_;
    int maxHeight_;
	int robotRadius_;
	//TODO: find a solution which doesn't need explicit yaw calculation
	std::vector<double> yawsVector;

public:
    Plane2DEnvironment(const char* ppm_file, int radius){
        bool ok=false;
        try {   //opening the file
            ppm_.loadFile(ppm_file);
            ok = true;
        } catch (ompl::Exception &ex) {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
        }
        
		if(!ok){
			return;
		}

		robotRadius_ = radius;

		ob::SE2StateSpace *space = new ob::SE2StateSpace();

		//define bounds
		ob::RealVectorBounds bounds(2);
		bounds.setLow(0, 0);
		bounds.setHigh(0, ppm_.getWidth());
		bounds.setLow(1, 0);
		bounds.setHigh(1, ppm_.getHeight());
		space->setBounds(bounds);			

        maxWidth_ = ppm_.getWidth() - 1;
        maxHeight_ = ppm_.getHeight() - 1;
        ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));

		//create object for state validity checking
		ss_->setStateValidityChecker(ob::StateValidityCheckerPtr(new MyStateValidityChecker(ppm_, ss_->getSpaceInformation(), robotRadius_)));


        space->setup();

        ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.0001);

		ss_->setPlanner(ob::PlannerPtr(new og::PRMstar(ss_->getSpaceInformation())));

		ob::OptimizationObjectivePtr obj1p(new MyStateCostIntegralObjective(ss_->getSpaceInformation(), ppm_));
		ob::OptimizationObjectivePtr obj2p(new ob::PathLengthOptimizationObjective(ss_->getSpaceInformation()));
		ob::MultiOptimizationObjective* moo = new ob::MultiOptimizationObjective(ss_->getSpaceInformation());
		moo->addObjective(obj1p, 0.001);
		moo->addObjective(obj2p, 0.001);

		ss_->setOptimizationObjective(ob::OptimizationObjectivePtr(moo));
    }//end of constructor

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

        // generate a few solutions; all will be added to the goal;        
        for (int i = 0 ; i < 10 ; ++i){
            if (ss_->getPlanner()){ //IGOR:this gets a random planner. We should choose the best planner. for more details visit http://ompl.kavrakilab.org/classompl_1_1geometric_1_1SimpleSetup.html#a8a94558b2ece27d938a92b062d55df71
                ss_->getPlanner()->clear();
            }
            ss_->solve();
			//TODO: finish
			if (ss_->haveSolutionPath()) {
				og::PathGeometric &p = ss_->getSolutionPath();//->cost(ss_->getOptimizationObjective());
				p->cost(ss_->getOptimizationObjective());
			}
        }
        
        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (ss_->haveSolutionPath()){
			//Simplifying solution
            og::PathGeometric &p = ss_->getSolutionPath();
            og::PathSimplifierPtr& pathSimplifier = ss_->getPathSimplifier();
			pathSimplifier->collapseCloseVertices(p, 100, 0);
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


	//TODO: Write to accomodate varying initial yaw.
	/*
	 * Written by Igor Berendorf
	 * getOrders will print to stdout the orders for the robot
	 */
	void getOrders(){
		og::PathGeometric &p = ss_->getSolutionPath();
		updateYaws();
		//TODO: fix. assuming initial orientation is 0
		double previous_yaw=0;
		double yaw;


		//Accessing each waypoint on the path
		std::vector< ob::State * >& waypoints = p.getStates();
		for(int i=0; i<waypoints.size()-1; i++){
			ob::State* state=waypoints[i];
			ob::State* next_state=waypoints[i+1];
	        const double X = (double)state->as<ob::SE2StateSpace::StateType>()->getX();
            const double Y = (double)state->as<ob::SE2StateSpace::StateType>()->getY();
	        const double next_X = (double)next_state->as<ob::SE2StateSpace::StateType>()->getX();
	        const double next_Y = (double)next_state->as<ob::SE2StateSpace::StateType>()->getY();
			yaw=yawsVector[i];

			//get the turn
			double turn = 360-previous_yaw+yaw;
			if (turn>180) {
				turn = turn -360;
			}

			//get distance
			double distance = sqrt( pow(abs(next_X - X), 2) + pow(abs(next_Y - Y), 2) );
			
			std::cout<<"turn "<< turn <<std::endl;
			std::cout<<"drive "<< distance <<std::endl;
			previous_yaw=yaw;
		}		
	}


	/*
	 * Written by Igor Berendorf
	 * updateYaws
	 * Function will update the resulting yaws from the solution path
	 * and add them to the solution path as the 3rd dimension
	 * NOTE: These yaws are the angle (0-360) at which to travel
	 */
	void updateYaws(){

		if (!ss_->haveSolutionPath()){ //no solution
			return;
		}
		og::PathGeometric &p = ss_->getSolutionPath();

		//Accessing each waypoint on the path
		std::vector< ob::State * >& waypoints = p.getStates();

		//This solution is not optimal. It uses {(#ofWaypoints-1) * 4} casts.
		//A better solution would be to use variables to represent the previous state
		for(int i=0; i<waypoints.size()-1; i++){

			ob::State* current_state=waypoints[i];
			ob::State* next_state=waypoints[i+1];
	        const double X = (double)current_state->as<ob::SE2StateSpace::StateType>()->getX();
            const double Y = (double)current_state->as<ob::SE2StateSpace::StateType>()->getY();
	        const double next_X = (double)next_state->as<ob::SE2StateSpace::StateType>()->getX();
            const double next_Y = (double)next_state->as<ob::SE2StateSpace::StateType>()->getY();

			//depends on the quadrant
			unsigned int rel_quadrant=getRelativeQuadrant(X, Y, next_X, next_Y);

			double Y_diff = abs(next_Y - Y);
			double X_diff = abs(next_X - X);
			double angle = atan2(Y_diff, X_diff) * 180/M_PI;
				
			double yaw=0;
			switch (rel_quadrant){
				case 1:
					yaw = angle;
					break;
				case 2:
					yaw = 180 - angle;
					break;
				case 3:
					yaw = 180 + angle;
					break;
				case 4:
					yaw = 360 - angle;
					break;								
				default:
					std::cout<<"ERROR"<<std::endl;				
			}//end of switch case		
			yawsVector.push_back(yaw); //update the yawVecotr
		}//end of for
		return;
	}//end of calcYaws

	void getClearances(){
		og::PathGeometric &p = ss_->getSolutionPath();
		std::vector< ob::State * >& waypoints = p.getStates();
		for(int i=0; i<waypoints.size()-1; i++){
			ob::State* state=waypoints[i];
			double c = ss_->getStateValidityChecker()->clearance(state);
			std::cout<< c <<std::endl;
		}
		return;
	}

private:
	//TODO: see what happens if target and reference are at the same coordinate
	/*
	* Written by Igor Berendorf
	* getRelativeQuadrant
	* NOTE: This function works with ompl's basic coordinate system (inverted y axis)
	* 		but will return the relative quadrant in regular coordinates
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
		if( y_tar > y_ref ){
			quadrants[0] = quadrants[1] = 0;
		} else {
			quadrants[2] = quadrants[3] = 0;
		}
		int i;
		for(i=0; i<4; i++){	//find the only quadrant not set to 0
			if (quadrants[i] != 0) {
				break;
			}
		}
		return quadrants[i];
	}//end of getRelativeQuadrant





}; //end of class
    
int main(int argc, char **argv){
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	char* filename = argv[1];
	int radius = std::stoi(argv[2]);
	int startX = std::stoi(argv[3]);
	int startY = std::stoi(argv[4]);
	int endX = std::stoi(argv[5]);
	int endY = std::stoi(argv[6]);
	std::cout<< "file is: " << filename << " ; radius is: " << radius << std::endl;
    Plane2DEnvironment env(filename, radius);
	if (env.plan(startX, startY, endX, endY)){
		env.getOrders();
        env.recordSolution();
        env.save("reduce_vertices.ppm");
    }
    return 0;
}





