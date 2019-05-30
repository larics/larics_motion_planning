/// \file UavWpManipulatorStateValidityChecker.h
/// \brief This file provides with a method to check if state of a UAV with 
///   the wp manipulator is valid.

#ifndef UAV_WP_MANIPULATOR_STATE_VALIDITY_CHECKER
#define UAV_WP_MANIPULATOR_STATE_VALIDITY_CHECKER

// #include "MotionPlanningDatatypes.h"
#include <StateValidityCheckerInterface.h>
#include <MapInterface.h>

// Include for direct and inverse kinematics.
#include <aerial_manipulators_control/ManipulatorControl.h>

#include <iostream>
#include <string>
#include <memory>
#include <eigen3/Eigen/Eigen>

using namespace std;

/// Uses direct and inverse kinematics of a UAV with wp manipulator to check
/// if robot state is valid. It uses direct kinematics to get positions of 
/// all links and joints and approximates the manipulator with rectangular 
/// shapes.
class UavWpManipulatorStateValidityChecker : public StateValidityCheckerInterface
{
  public:
    /// \brief Constructor.
    /// \param config_filename Configuration filename for validity checker.
    /// \param map Map interface that is used for checking validity of the
    ///   system
    UavWpManipulatorStateValidityChecker(string config_filename, 
      shared_ptr<MapInterface> map);

    /// \brief Validity of a robot state.
    /// \param state Vector of robot state.
    /// \return True if robot state does not interfere with obstacles. False
    ///   otherwise.
    bool isStateValid(Eigen::VectorXd state);

  private:
    shared_ptr<MapInterface> map_;
};

#endif // UAV_WP_MANIPULATOR_STATE_VALIDITY_CHECKER