#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
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
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

//for log output
#include <fstream>

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
	double mod_;
	double startYaw;

public:
	MyStateCostIntegralObjective(const ob::SpaceInformationPtr& si, ompl::PPM ppm) : ob::StateCostIntegralObjective(si, true) {
		description_="rgb based state weight";
		ppm_ = ppm;
	}

	ob::Cost stateCost(const ob::State* state) const {		
        const int w = (int)state->as<ob::SE2StateSpace::StateType>()->getX();
        const int h = (int)state->as<ob::SE2StateSpace::StateType>()->getY();
        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
		
		double weight = MAX_COLOR - c.red;
		
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
				if( c.red > 10 && c.green > 10 && c.blue > 10){
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
	std::vector<double> yawsVector;

	int approxSolutions;
	int exactSolutions;
	double minimalCost;
	double finalCost;

public:

	int getApproxmiateSolutionCount(){
		return approxSolutions;
	}

	int getExactSolutionCount(){
		return exactSolutions;
	}

	double getMinimalCost(){
		return minimalCost;
	}

	double getFinalCost(){
		return finalCost;
	}

    Plane2DEnvironment(const char* ppm_file, int radius, double probabilityModifier, double lengthModifier, char* plannerName){
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
		approxSolutions = 0;
		exactSolutions = 0;
		minimalCost = -1.0;
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
		//This changes nothing
		space->setValidSegmentCountFactor(10);

		//By default DiscreteMotionValidator is used, but let's call it anyway
		ss_->getSpaceInformation()->setMotionValidator(ob::MotionValidatorPtr(new ob::DiscreteMotionValidator(ss_->getSpaceInformation())));
        ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.0001); //Increasing this value results in larger distance between states

		//Choose the planner
		ob::Planner* planner;
		if (!strcmp(plannerName, "prmstar")) {
			planner = new og::PRMstar(ss_->getSpaceInformation());
		} else if (!strcmp(plannerName, "rrtstar")) {
			//og::RRTstar* rrtPlanner = new og::RRTstar(ss_->getSpaceInformation());
			//rrtPlanner->setKNearest(true);
			planner = new og::RRTstar(ss_->getSpaceInformation());
		} else 	if (!strcmp(plannerName, "fmtstar")) {
			planner = new og::FMT(ss_->getSpaceInformation());
		} else 	if (!strcmp(plannerName, "trrt")) {
			planner = new og::TRRT(ss_->getSpaceInformation());
		} else {
			std::cout<< "Illegal planner name" <<std::endl;
			exit(1);
		}

		ss_->setPlanner(ob::PlannerPtr(planner));

		ob::OptimizationObjectivePtr obj1p(new MyStateCostIntegralObjective(ss_->getSpaceInformation(), ppm_));
		ob::OptimizationObjectivePtr obj2p(new ob::PathLengthOptimizationObjective(ss_->getSpaceInformation()));
		ob::MultiOptimizationObjective* moo = new ob::MultiOptimizationObjective(ss_->getSpaceInformation());
		moo->addObjective(obj1p, probabilityModifier);
		moo->addObjective(obj2p, lengthModifier);

		ss_->setOptimizationObjective(ob::OptimizationObjectivePtr(moo));
    }//end of constructor

    bool plan(unsigned int start_row, unsigned int start_col, int start_yaw, unsigned int goal_row, unsigned int goal_col, int iterations, double attempt_duration){
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
        for (int i = 0 ; i < iterations ; ++i){
		if (!ss_->haveExactSolutionPath()){
				//std::cout<< "_____________________"<<std::endl;
				//std::cout << "====Iteration "<<i+1<<"===="<<std::endl;
		    if (ss_->getPlanner()){
		        ss_->getPlanner()->clear();
		    }
		    ss_->solve(attempt_duration);

				if (ss_->haveExactSolutionPath()){
					exactSolutions++;
				}
				if (ss_->haveSolutionPath()){
					approxSolutions++;
				}

				if (ss_->haveSolutionPath()) {
					double currentCost = ss_->getProblemDefinition()->getSolutionPath()->cost(ss_->getProblemDefinition()->getOptimizationObjective()).value();
					std::cout << currentCost << std::endl;
					//update minimal path cost
					if(minimalCost > currentCost || minimalCost == -1.0){
						minimalCost = currentCost;
					}
					//commented out so we have a clean solution path
					//this->recordSolution((i+1)*25);
				}
		}
	}
	    
        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (ss_->haveSolutionPath()){
			//Simplifying solution
            og::PathGeometric &p = ss_->getSolutionPath();
            og::PathSimplifierPtr& pathSimplifier = ss_->getPathSimplifier();
			pathSimplifier->shortcutPath(p);
			pathSimplifier->collapseCloseVertices(p);
			std::cout << ss_->getProblemDefinition()->getSolutionPath()->cost(ss_->getProblemDefinition()->getOptimizationObjective()) << std::endl;
			finalCost = ss_->getProblemDefinition()->getSolutionPath()->cost(ss_->getProblemDefinition()->getOptimizationObjective()).value();
			updateYaws(start_yaw);            
			return true;
        } else {
            return false;
        }
    } //end of plan

    void recordSolution(int color){
        if (!ss_ || !ss_->haveSolutionPath()){
            return;
        }
        og::PathGeometric &p = ss_->getSolutionPath();
        p.interpolate();
        for (std::size_t i = 0 ; i < p.getStateCount() ; ++i){
            const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::SE2StateSpace::StateType>()->getX());
            const int h = std::min(maxHeight_, (int)p.getState(i)->as<ob::SE2StateSpace::StateType>()->getY());
            ompl::PPM::Color &c = ppm_.getPixel(h, w);
            c.red = color;
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
	 * getOrders will print to stdout the orders for the robot
	 */
	void getOrders(double initialYaw, char* instructionsFile){
		og::PathGeometric &p = ss_->getSolutionPath();
		double previous_yaw=initialYaw;
		double yaw;

		std::ofstream outputFile;
		outputFile.open(instructionsFile);

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
			double turn = 360-previous_yaw-yaw;
			if (turn>180) {
				turn = turn -360;
			}

			//get distance
			double distance = sqrt( pow(abs(next_X - X), 2) + pow(abs(next_Y - Y), 2) );
			
			outputFile << "turn " << turn << std::endl;
			outputFile << "drive "<< distance << std::endl;
			
			//std::cout<<"turn "<< turn <<std::endl;
			//std::cout<<"drive "<< distance <<std::endl;
			previous_yaw=yaw;
		}
		outputFile.close();
	}


	/*
	 * Written by Igor Berendorf
	 * updateYaws
	 * Function will update the resulting yaws from the solution path
	 * and add them to the solution path as the 3rd dimension
	 * NOTE: These yaws are the angle (0-360) at which to travel
	 */
	void updateYaws(double initialYaw){

		if (!ss_->haveSolutionPath()){ //no solution
			return;
		}
		og::PathGeometric &p = ss_->getSolutionPath();

		//Accessing each waypoint on the path
		std::vector< ob::State * >& waypoints = p.getStates();

		//This solution is not optimal. It uses {(#ofWaypoints-1) * 4} casts.
		//A better solution would be to use variables to represent the previous state
		double yaw = initialYaw;
		for(int i=0; i<waypoints.size()-1; i++){

			ob::State* current_state = waypoints[i];
			ob::State* next_state = waypoints[i+1];
	        	const double X = (double)current_state->as<ob::SE2StateSpace::StateType>()->getX();
            	const double Y = (double)current_state->as<ob::SE2StateSpace::StateType>()->getY();
	        	const double next_X = (double)next_state->as<ob::SE2StateSpace::StateType>()->getX();
           		const double next_Y = (double)next_state->as<ob::SE2StateSpace::StateType>()->getY();
			bool is_yaw = false;

			//depends on the quadrant
			unsigned int rel_quadrant = getRelativeQuadrant(X, Y, next_X, next_Y);

			double Y_diff = next_Y - Y;
			double X_diff = next_X - X;
			double angle = atan2(abs(Y_diff), abs(X_diff)) * 180/M_PI;

			if (Y_diff==0) {
	  			if (X_diff<0){
	  				yaw = 180;
					is_yaw = true;
	  			} else {
	  				yaw = 0;
					is_yaw = true;
	  			}
  			} else if(X_diff==0){
  				if(Y_diff>0){
  					yaw = 90;
					is_yaw = true;
  				} else {
  					yaw = 270;
					is_yaw = true;
  				}
			}
				
			//double yaw=0;
			
			if (!is_yaw){
				yaw = angle + 90 * (rel_quadrant-1);
			}
			yawsVector.push_back(yaw); //update the yawVecotr
			yaw = 0;
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
	/*
	* getRelativeQuadrant
	* NOTE: This function works with ompl's basic coordinate system (inverted y axis)
	* 		but will return the relative quadrant in regular coordinates
	* Parameters are X,Y of Origin, followed by X,Y of target coordinate
	* Returns the relative quadrant of Target to Origin (1 to 4)
	*/
	unsigned int getRelativeQuadrant(double x_ref, double y_ref, double x_tar, double y_tar){
		if(y_tar>y_ref){//if he higher
  			if(x_tar>x_ref){//if he righter
  				return 1;
  			} else {
  				return 2;
  			}
  		} else {//else
  			if(x_tar>x_ref){//if he righter
  				return 4;
  			} else {
  				return 3;
  			}
  		}
	}//end of getRelativeQuadrant
}; //end of class


//program parameters <map_file> <radius> <start_X> <start_Y> <start_YAW> <goal_X> <goal_Y> <planner> <probability_mod> <length_mod> <iteration_count> <time_per_iteration> <output_log_file> <output_ppm_file> 
//example:
//./a.out /home/igor/robot_movement/OlgaIgor_project/gmaps/map_full.ppm 1 1822 4842 0 328 1136 rrtstar 1 1 10 1 test_log test_sol
//available planners: prmstar, rrtstar, fmtstar, trrt

int main(int argc, char **argv){
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

	char* filename = argv[1];
	int radius = std::stoi(argv[2]);
	int startX = std::stoi(argv[3]);
	int startY = std::stoi(argv[4]);
	int startYaw = std::stoi(argv[5]);

	int endX = std::stoi(argv[6]);
	int endY = std::stoi(argv[7]);

	char* plannerName = argv[8];

	double probabilityModifier = std::stoi(argv[9]);
 	double lengthModifier = std::stoi(argv[10]);

	int iterations = std::stoi(argv[11]);
	double maxIterationDuration = std::stoi(argv[12]);

	char* log = argv[13];
	std::ofstream out;
	out.open(log, std::ios::app);

	char* instructionsFile = argv[14];

	char* outputFile = argv[15];

	const char* type = ".ppm";
	strcat(outputFile, type);

	std::cout<< "Input map: " << filename << std::endl;
	std::cout<<"Output map: "<<outputFile<< std::endl;
	std::cout<<"Instructions: " << instructionsFile << std::endl;
	std::cout<< "Radius: " << radius << std::endl;
    Plane2DEnvironment env(filename, radius, probabilityModifier, lengthModifier, plannerName);
	if (env.plan(startX, startY, startYaw, endX, endY, iterations, maxIterationDuration)){
		env.getOrders(startYaw, instructionsFile);
        env.recordSolution(250);
        env.save(outputFile);
    }

	//NOTE: approximate solutions (according to ompl) include exact solutions
	out<<plannerName<<","<< env.getApproxmiateSolutionCount() - env.getExactSolutionCount() << "," << env.getExactSolutionCount() << "," << env.getFinalCost() <<  "," << iterations << "," << maxIterationDuration << std::endl;	



	out.close();

    return 0;
}





