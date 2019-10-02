/// \file PointStateValidityChecker.h
/// \brief This calss considers the robot to be a point and checks if that
///   is in obstacle or not.

#ifndef POINT_STATE_VALIDITY_CHECKER_H
#define POINT_STATE_VALIDITY_CHECKER_H

// #include "MotionPlanningDatatypes.h"
#include <larics_motion_planning/StateValidityCheckerInterface.h>
#include <larics_motion_planning/MapInterface.h>

#include <iostream>
#include <string>
#include <memory>
#include <eigen3/Eigen/Eigen>

using namespace std;

/// This class check if a single point is in obstacle or not.
class PointStateValidityChecker : public StateValidityCheckerInterface
{
  public:
    /// \brief Constructor.
    /// \param map Map interface that is used for checking validity of a point.
    PointStateValidityChecker(shared_ptr<MapInterface> map);

    /// \brief Validity of a robot state.
    /// \param state Vector of robot state.
    /// \return True if robot state does not interfere with obstacles. False
    ///   otherwise.
    bool isStateValid(Eigen::VectorXd state);

    /// \brief Generates a single point to be checked.
    /// \param state Robot state, only first three DOF are taken into account.
    /// \return Matrix of points for collision checking.
    Eigen::MatrixXd generateValidityPoints(Eigen::VectorXd state);

  private:
    shared_ptr<MapInterface> map_;
};

#endif // POINT_STATE_VALIDITY_CHECKER_H