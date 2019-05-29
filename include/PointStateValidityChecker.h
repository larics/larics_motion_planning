/// \file PointStateValidityChecker.h
/// \brief This calss considers the robot to be a point and checks if that
///   is in obstacle or not.

#ifndef POINT_STATE_VALIDITY_CHECKER_H
#define POINT_STATE_VALIDITY_CHECKER_H

// #include "MotionPlanningDatatypes.h"
#include <StateValidityCheckerInterface.h>
#include <iostream>
#include <string>
#include <memory>
#include <eigen3/Eigen/Eigen>
#include <MapInterface.h>

using namespace std;

/// This is an interface class for path planning.
class PointStateValidityChecker : public StateValidityCheckerInterface
{
  public:
    PointStateValidityChecker(string config_filename, 
      shared_ptr<MapInterface> map);
    bool isStateValid(Eigen::VectorXd state);
};

#endif // POINT_STATE_VALIDITY_CHECKER_H