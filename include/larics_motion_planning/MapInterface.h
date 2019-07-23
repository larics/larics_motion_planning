/// \file MapInterface.h
/// \brief Contains the abstract class interface for map representation.

#ifndef MAP_INTERFACE_H
#define MAP_INTERFACE_H

#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <iostream>

using namespace std;

/// This is an interface class for trajectories of all types.
class MapInterface
{
  public:
    MapInterface();
    virtual bool isStateValid(Eigen::VectorXd state) = 0;
    virtual bool configureFromFile(string config_file) = 0;
};

#endif // MAP_INTERFACE_H
