#ifndef PLANNING_OMPL
#define PLANNING_OMPL

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
// #include <ompl/geometric/planners/informedtrees/AITstar.h>
// #include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>


// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <boost/filesystem.hpp>
#include <memory>

#include <fstream>

#include "utils.hpp"



namespace ob = ompl::base;
namespace og = ompl::geometric;

// An enum of supported optimal planners, alphabetical order
enum optimalPlanner
{
    PLANNER_INF_RRTSTAR,
    PLANNER_RRTSTAR,
};

// An enum of the supported optimization objectives, alphabetical order
enum planningObjective
{
    OBJECTIVE_PATHCLEARANCE,
    OBJECTIVE_PATHLENGTH,
    OBJECTIVE_THRESHOLDPATHLENGTH,
    OBJECTIVE_WEIGHTEDCOMBO
};

bool argParse(int argc, char** argv, double *runTimePtr, optimalPlanner *plannerPtr, planningObjective *objectivePtr, std::string *outputFilePtr);

//class ValidityChecker : public ob::StateValidityChecker;
//class ClearanceObjective : public ob::StateCostIntegralObjective;

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, optimalPlanner plannerType);

ob::OptimizationObjectivePtr allocateObjective(const ob::SpaceInformationPtr& si, planningObjective objectiveType);

void plan(double runTime, optimalPlanner plannerType, planningObjective objectiveType, const std::string& outputFile, const Polygon borders,
        const float x, const float y, const float gatex, const float gatey, const Polygon circle, const std::vector<float> radius);
#endif
