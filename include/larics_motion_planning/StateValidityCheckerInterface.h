/// \file StateValidityCheckerInterface.h
/// \brief Contains the abstract class interface for checking robot state
///   validity.

#ifndef STATE_VALIDITY_CHECKER_INTERFACE_H
#define STATE_VALIDITY_CHECKER_INTERFACE_H

// #include "MotionPlanningDatatypes.h"
#include <iostream>
#include <eigen3/Eigen/Eigen>

using namespace std;

/// This is an interface class for state validity checking.
class StateValidityCheckerInterface
{
  public:
    StateValidityCheckerInterface();
    virtual bool isStateValid(Eigen::VectorXd state) = 0;
    Eigen::MatrixXd getStatePoints() {return points_;}
    virtual Eigen::MatrixXd generateValidityPoints(Eigen::VectorXd state) {return points_;}
    virtual bool configureFromFile(string config_filename) {return true;}

  protected:
    Eigen::MatrixXd points_;
};

#endif // STATE_VALIDITY_CHECKER_INTERFACE_H