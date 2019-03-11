/// \file PathPlanningInterface.h
/// \brief Contains the abstract class interface for path planning.

#ifndef PATH_PLANNING_INTERFACE_H
#define PATH_PLANNING_INTERFACE_H

#include "MotionPlanningDatatypes.h"
#include <iostream>

using namespace std;

/// This is an interface class for path planning.
class PathPlanningInterface
{
  public:
    PathPlanningInterface();
    virtual bool planPath(Eigen::MatrixXd positions) = 0;
    virtual Eigen::MatrixXd getPath() = 0;
};

#endif // PATH_PLANNING_INTERFACE_H