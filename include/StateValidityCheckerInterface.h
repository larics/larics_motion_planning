/// \file StateValidityCheckerInterface.h
/// \brief Contains the abstract class interface for checking robot state
///   validity.

#ifndef STATE_VALIDITY_CHECKER_INTERFACE_H
#define STATE_VALIDITY_CHECKER_INTERFACE_H

// #include "MotionPlanningDatatypes.h"
#include <iostream>
#include <eigen3/Eigen/Eigen>

using namespace std;

/// This is an interface class for path planning.
class StateValidityCheckerInterface
{
  public:
    StateValidityCheckerInterface();
    virtual bool isStateValid(Eigen::MatrixXd state) = 0;
};

#endif // STATE_VALIDITY_CHECKER_INTERFACE_H