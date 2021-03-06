/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Luis G. Torres, Jonathan Gammell */

#include <chrono>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/PathSimplifier.h>


// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>

#include <fstream>

#include <sstream>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;
/// @cond IGNORE
using namespace std;
// An enum of supported optimal planners, alphabetical order
enum optimalPlanner
{
    PLANNER_BITSTAR,
    PLANNER_CFOREST,
    PLANNER_FMTSTAR,
    PLANNER_BFMTSTAR,
    PLANNER_INF_RRTSTAR,
    PLANNER_PRMSTAR,
    PLANNER_RRTSTAR,
    PLANNER_RRT
};

// An enum of the supported optimization objectives, alphabetical order
enum planningObjective
{
    OBJECTIVE_PATHCLEARANCE,
    OBJECTIVE_PATHLENGTH,
    OBJECTIVE_THRESHOLDPATHLENGTH,
    OBJECTIVE_WEIGHTEDCOMBO
};

// Parse the command-line arguments
bool argParse(int argc, char** argv, double *runTime, optimalPlanner *plannerPtr, planningObjective *objectivePtr, std::string *outputFilePtr,std::string *bmFilePtr, std::string *sgFilePtr,std::string *resultFile);

// Our "collision checker". For this demo, our robot's state space
// lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
// centered at (0.5,0.5). Any states lying in this circular region are
// considered "in collision".
class ValidityChecker : public ob::StateValidityChecker
{
public:
    vector<vector<double> > binaryMap;
    ValidityChecker(const ob::SpaceInformationPtr& si,vector<vector<double> > bm) :
        ob::StateValidityChecker(si) {
            this->binaryMap=bm;
        }

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
    {
        int ans = this->clearance(state);
        if(ans==0)
            return true;
        else
            return false;
    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ob::RealVectorStateSpace::StateType* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        int x = (int)floor(state2D->values[0]);
        int y = (int)floor(state2D->values[1]);
        //std::cout<<x<<" "<<y<<" "<<binaryMap[x][y]<<std::endl;
        // Distance formula between two points, offset by the circle's
        // radius
        return binaryMap[x][y];

            // Extract the robot's (x,y) position from its state
        /*double x = state2D->values[0];
        double y = state2D->values[1];

        // Distance formula between two points, offset by the circle's
        // radius
        return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25;*/
    }
};

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, optimalPlanner plannerType)
{
    switch (plannerType)
    {
        case PLANNER_BITSTAR:
        {
            return std::make_shared<og::BITstar>(si);
            break;
        }
        case PLANNER_CFOREST:
        {
            return std::make_shared<og::CForest>(si);
            break;
        }
        case PLANNER_FMTSTAR:
        {
            return std::make_shared<og::FMT>(si);
            break;
        }
        case PLANNER_BFMTSTAR:
        {
            return std::make_shared<og::BFMT>(si);
            break;
        }
        case PLANNER_INF_RRTSTAR:
        {
            return std::make_shared<og::InformedRRTstar>(si);
            break;
        }
        case PLANNER_PRMSTAR:
        {
            return std::make_shared<og::PRMstar>(si);
            break;
        }
        case PLANNER_RRTSTAR:
        {
            return std::make_shared<og::RRTstar>(si);
            break;
        }
        case PLANNER_RRT:
        {
            return std::make_shared<og::RRT>(si);
            break;
        }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}

ob::OptimizationObjectivePtr allocateObjective(ob::SpaceInformationPtr si, planningObjective objectiveType)
{
    switch (objectiveType)
    {
        case OBJECTIVE_PATHCLEARANCE:
            return getClearanceObjective(si);
            break;
        case OBJECTIVE_PATHLENGTH:
            return getPathLengthObjective(si);
            break;
        case OBJECTIVE_THRESHOLDPATHLENGTH:
            return getThresholdPathLengthObj(si);
            break;
        case OBJECTIVE_WEIGHTEDCOMBO:
            return getBalancedObjective1(si);
            break;
        default:
            OMPL_ERROR("Optimization-objective enum is not implemented in allocation function.");
            return ob::OptimizationObjectivePtr();
            break;
    }
}

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

vector<vector<double> > getSourceGoal(std::string sourceGoalfile)
{
    vector<vector<double> > sg;
    std::ifstream sgfile(sourceGoalfile);
    cout<<sourceGoalfile<<endl;
    if(!sgfile.is_open())
    {
        cout<<"Could not open Binary Map file...";
        exit(0);
    }

    
    std::string str;
    while(std::getline(sgfile,str))
    {
        vector<std::string> pos=split(str,' ');
        vector<double> position;
        for(int i=0;i<pos.size();i++)
        {
            position.push_back(std::stod(pos.at(i)));
        }
        sg.push_back(position);
    }

    return sg;
}

vector<vector<double> > getBinaryMap(std::string binaryMapfile)
{

    vector<vector<double> > binaryMap;
    std::ifstream bmfile(binaryMapfile);
    if(!bmfile.is_open())
    {
        cout<<"Could not open Binary Map file...";
        exit(0);
    }
    
    
    std::string str;
    std::getline(bmfile,str);
    
    while(std::getline(bmfile,str))
    {
        vector<std::string> binary=split(str,' ');
        vector<double> env_binary;
        for(int i=0;i<binary.size();i++)
        {
            env_binary.push_back(std::stod(binary.at(i)));
        }
        binaryMap.push_back(env_binary);
    }

    return binaryMap;
}


void plan(double runTime, optimalPlanner plannerType, planningObjective objectiveType, std::string outputFile,std::string bmFile,std::string sgFile,std::string resultFile)
{
    //get binaryMap and source source goal location
    vector<vector<double> > binaryMap = getBinaryMap(bmFile);
    vector<vector<double> > sg = getSourceGoal(sgFile);

    // Construct the robot state space in which we're planning. We're
    // planning in [0,1]x[0,1], a subset of R^2.
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

    // Set the bounds of space to be in [0,1].
    space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 100.0);

    // Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si,binaryMap)));

    si->setup();

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = sg[0][0];
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = sg[0][1];

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = sg[1][0];
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = sg[1][1];

    cout<<sg[0][0]<<" "<<sg[0][1]<<" "<<sg[1][0]<<" "<<sg[1][1]<<" "<<endl;

    // Create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Create the optimization objective specified by our command-line argument.
    // This helper function is simply a switch statement.
    pdef->setOptimizationObjective(allocateObjective(si, objectiveType));

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr optimizingPlanner = allocatePlanner(si, plannerType);

    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    // attempt to solve the planning problem in the given runtime
    ob::PlannerStatus solved = optimizingPlanner->solve(runTime);
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

    if (solved)
    {
        
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        path->print(std::cout);
             
        // Output the length of the path found
        std::cout
            << optimizingPlanner->getName()
            << " found a solution of length "
            << pdef->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << "\n Time "<<duration<<std::endl;     
             
              const ob::PathPtr &p = pdef->getSolutionPath();
              og::PathGeometric &path1 = static_cast<og::PathGeometric&>(*p);
              path1.printAsMatrix(std::cout);


             /* //path shortcut applied here....
              std::cout<<"\nPath Shortcut"<<std::endl;
              og::PathSimplifier *pathSimplifier = new ompl::geometric::PathSimplifier(si);
              bool shortened = pathSimplifier->shortcutPath(path1);
              if(shortened == true)
                std::cout<<"\nCongrats! Path is shortened\n";
              */

        // If a filename was specified, output the path as a matrix to
        // that file for visualization
        double minimumLength = path1.length();   
        double pathLengthShortCutPath = path1.length();   
        double pathLengthSimplifyMax = path1.length();
        
        og::PathSimplifier *pathSimplifier = new ompl::geometric::PathSimplifier(si);
        og::PathGeometric &pathforShortCut = static_cast<og::PathGeometric&>(*p);
        t1 = std::chrono::high_resolution_clock::now();
        bool shortened = pathSimplifier->shortcutPath(pathforShortCut);
        t2 = std::chrono::high_resolution_clock::now();
        auto durationShortCutPath = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
        pathLengthShortCutPath = pathforShortCut.length();

        og::PathGeometric &pathforSimplifyMax = static_cast<og::PathGeometric&>(*p);
        t1 = std::chrono::high_resolution_clock::now();
        pathSimplifier->simplifyMax(pathforSimplifyMax);
        t2 = std::chrono::high_resolution_clock::now();
        auto durationSimplyMaxPath = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
        pathLengthSimplifyMax = pathforSimplifyMax.length();
            
        if (!outputFile.empty())
        {
            

            /*
            std::ofstream outFile(outputFile.c_str());
            outFile<<"Cost Original: "<<pdef->getSolutionPath()->length()<<"\n";
            //outFile<<"Optimized Cost "<< path1.length()<<"\n";
            outFile<<"Time Original: "<<duration<<"\n";
            outFile<<"Original Path: "<<"\n";
            std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())-> printAsMatrix(outFile);

            outFile<<"\n-----------------------------\n";
            og::PathSimplifier *pathSimplifier = new ompl::geometric::PathSimplifier(si);
            og::PathGeometric &pathforShortCut = static_cast<og::PathGeometric&>(*p);
            bool shortened = pathSimplifier->shortcutPath(pathforShortCut);  
            if(shortened == true)
            {
                outFile<<"\n ShortCut Path \n :"<<pathforShortCut.length();
                pathforShortCut.printAsMatrix(outFile);
            }
            
            outFile<<"\n-----------------------------\n";
            og::PathGeometric &pathforReducedVertices = static_cast<og::PathGeometric&>(*p);
            shortened = pathSimplifier->reduceVertices(pathforReducedVertices);
            if(shortened == true)
            {
                outFile<<"\n Reduced Vertices Path \n :"<<pathforReducedVertices.length();
                pathforReducedVertices.printAsMatrix(outFile);
            }
            
            outFile<<"\n-----------------------------\n";
            og::PathGeometric &pathforSimplifyMax = static_cast<og::PathGeometric&>(*p);
            pathSimplifier->simplifyMax(pathforSimplifyMax);
            outFile<<"\n Simplify Max Path \n :"<<pathforSimplifyMax.length();
            minimumLength = pathforSimplifyMax.length();
            pathforSimplifyMax.printAsMatrix(outFile);

            //path1.printAsMatrix(outFile);
            
            outFile.close();*/
        }

        if(!resultFile.empty())
        {
            std::ofstream resFile(resultFile.c_str(),std::fstream::app);
            //resFile<<min(pdef->getSolutionPath()->length(),minimumLength)<<" "<<duration<<"\n";
            resFile<< minimumLength << " " << pathLengthShortCutPath << " " << pathLengthSimplifyMax <<" "<<duration<<" "<< durationShortCutPath<< " " << durationSimplyMaxPath<< "\n";
        }
    }
    else
        std::cout << "No solution found." << std::endl;
}


int main(int argc, char** argv)
{
    // The parsed arguments
    double runTime;
    optimalPlanner plannerType;
    planningObjective objectiveType;
    std::string outputFile;
    std::string bmFile;
    std::string sgFile;
    std::string resultFile;


    // Parse the arguments, returns true if successful, false otherwise
    if (argParse(argc, argv, &runTime, &plannerType, &objectiveType, &outputFile,&bmFile,&sgFile,&resultFile))
    {
        

        // Plan
        plan(runTime, plannerType, objectiveType, outputFile,bmFile,sgFile,resultFile);

        // Return with success
        return 0;
    }
    else
    {
        // Return with error
        return -1;
    }
}

/** Returns a structure representing the optimization objective to use
    for optimal motion planning. This method returns an objective
    which attempts to minimize the length in configuration space of
    computed paths. */
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

/** Returns an optimization objective which attempts to minimize path
    length that is satisfied when a path of length shorter than 1.51
    is found. */
ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(100000));
    return obj;
}

/** Defines an optimization objective which attempts to steer the
    robot away from obstacles. To formulate this objective as a
    minimization of path cost, we can define the cost of a path as a
    summation of the costs of each of the states along the path, where
    each state cost is a function of that state's clearance from
    obstacles.

    The class StateCostIntegralObjective represents objectives as
    summations of state costs, just like we require. All we need to do
    then is inherit from that base class and define our specific state
    cost function by overriding the stateCost() method.
 */
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }

    // Our requirement is to maximize path clearance from obstacles,
    // but we want to represent the objective as a path cost
    // minimization. Therefore, we set each state's cost to be the
    // reciprocal of its clearance, so that as state clearance
    // increases, the state cost decreases.
    ob::Cost stateCost(const ob::State* s) const
    {
        return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
    }
};

/** Return an optimization objective which attempts to steer the robot
    away from obstacles. */
ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ClearanceObjective(si));
}

/** Create an optimization objective which attempts to optimize both
    path length and clearance. We do this by defining our individual
    objectives, then adding them to a MultiOptimizationObjective
    object. This results in an optimization objective where path cost
    is equivalent to adding up each of the individual objectives' path
    costs.

    When adding objectives, we can also optionally specify each
    objective's weighting factor to signify how important it is in
    optimal planning. If no weight is specified, the weight defaults to
    1.0.
*/
ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));

    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(lengthObj, 10.0);
    opt->addObjective(clearObj, 1.0);

    return ob::OptimizationObjectivePtr(opt);
}

/** Create an optimization objective equivalent to the one returned by
    getBalancedObjective1(), but use an alternate syntax.
 */
ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));

    return 10.0*lengthObj + clearObj;
}

/** Create an optimization objective for minimizing path length, and
    specify a cost-to-go heuristic suitable for this optimal planning
    problem. */
ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
    return obj;
}

/** Parse the command line arguments into a string for an output file and the planner/optimization types */
bool argParse(int argc, char** argv, double* runTimePtr, optimalPlanner *plannerPtr, planningObjective *objectivePtr, std::string *outputFilePtr, std::string *bmFilePtr, std::string *sgFilePtr,std::string *resultFile)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("runtime,t", bpo::value<double>()->default_value(10.0), "(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.")
        ("planner,p", bpo::value<std::string>()->default_value("RRTstar"), "(Optional) Specify the optimal planner to use, defaults to RRTstar if not given. Valid options are BITstar, CForest, FMTstar, BFMTstar, InformedRRTstar, PRMstar, and RRTstar.") //Alphabetical order
        ("objective,o", bpo::value<std::string>()->default_value("PathLength"), "(Optional) Specify the optimization objective, defaults to PathLength if not given. Valid options are PathClearance, PathLength, ThresholdPathLength, and WeightedLengthAndClearanceCombo.") //Alphabetical order
        ("file,f", bpo::value<std::string>()->default_value(""), "(Optional) Specify an output path for the found solution path.")
        ("info,i", bpo::value<unsigned int>()->default_value(0u), "(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG. Defaults to WARN.")
        ("binaryMap,b", bpo::value<std::string>()->default_value(""),"(Required) Set the binary map file path.")
        ("resultFile,r",bpo::value<std::string>()->default_value(""), "(Optional) This file store final time and final cost.")
        ("sourceGoal,s",bpo::value<std::string>()->default_value(""),"(Required) Set the source goal file path.");
    
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // Check if the help flag has been given:
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return false;
    }

    // Set the log-level
    unsigned int logLevel = vm["info"].as<unsigned int>();

    // Switch to setting the log level:
    if (logLevel == 0u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
    }
    else if (logLevel == 1u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
    }
    else if (logLevel == 2u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);
    }
    else
    {
        std::cout << "Invalid log-level integer." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the runtime as a double
    *runTimePtr = vm["runtime"].as<double>();

    // Sanity check
    if (*runTimePtr <= 0.0)
    {
        std::cout << "Invalid runtime." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the specified planner as a string
    std::string plannerStr = vm["planner"].as<std::string>();

    // Map the string to the enum
    if (boost::iequals("BITstar", plannerStr))
    {
        *plannerPtr = PLANNER_BITSTAR;
    }
    else if (boost::iequals("CForest", plannerStr))
    {
        *plannerPtr = PLANNER_CFOREST;
    }
    else if (boost::iequals("FMTstar", plannerStr))
    {
        *plannerPtr = PLANNER_FMTSTAR;
    }
    else if (boost::iequals("BFMTstar", plannerStr))
    {
        *plannerPtr = PLANNER_BFMTSTAR;
    }
    else if (boost::iequals("InformedRRTstar", plannerStr))
    {
        *plannerPtr = PLANNER_INF_RRTSTAR;
    }
    else if (boost::iequals("PRMstar", plannerStr))
    {
        *plannerPtr = PLANNER_PRMSTAR;
    }
    else if (boost::iequals("RRTstar", plannerStr))
    {
        *plannerPtr = PLANNER_RRTSTAR;
    }
    else if(boost::iequals("RRT",plannerStr))
    {
        *plannerPtr = PLANNER_RRT;
    }
    else
    {
        std::cout << "Invalid planner string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the specified objective as a string
    std::string objectiveStr = vm["objective"].as<std::string>();

    // Map the string to the enum
    if (boost::iequals("PathClearance", objectiveStr))
    {
        *objectivePtr = OBJECTIVE_PATHCLEARANCE;
    }
    else if (boost::iequals("PathLength", objectiveStr))
    {
        *objectivePtr = OBJECTIVE_PATHLENGTH;
    }
    else if (boost::iequals("ThresholdPathLength", objectiveStr))
    {
        *objectivePtr = OBJECTIVE_THRESHOLDPATHLENGTH;
    }
    else if (boost::iequals("WeightedLengthAndClearanceCombo", objectiveStr))
    {
        *objectivePtr = OBJECTIVE_WEIGHTEDCOMBO;
    }
    else
    {
        std::cout << "Invalid objective string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the output file string and store it in the return pointer
    *outputFilePtr = vm["file"].as<std::string>();
    
    *bmFilePtr = vm["binaryMap"].as<std::string>();
    
    *sgFilePtr = vm["sourceGoal"].as<std::string>();

    *resultFile = vm["resultFile"].as<std::string>();







    // Looks like we parsed the arguments successfully
    return true;
}
/// @endcond
