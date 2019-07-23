/// \file PathPlanningInterface.h
/// \brief Contains the abstract class interface for path planning.

#ifndef PATH_PLANNING_INTERFACE_H
#define PATH_PLANNING_INTERFACE_H

#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <iostream>

using namespace std;

/// This is an interface class for path planning.
class PathPlanningInterface
{
  public:
    PathPlanningInterface();
    virtual bool planPath(Eigen::MatrixXd positions) = 0;
    virtual Eigen::MatrixXd getPath() = 0;
    virtual double getPathLength() = 0;
    virtual bool configureFromFile(string config_filename) = 0;
};

#endif // PATH_PLANNING_INTERFACE_H