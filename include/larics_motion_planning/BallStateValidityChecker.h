/// \file BallStateValidityChecker.h
/// \brief This calss considers the robot to be a ball and checks if that
///   is in obstacle or not.

#ifndef BALL_STATE_VALIDITY_CHECKER_H
#define BALL_STATE_VALIDITY_CHECKER_H

// #include "MotionPlanningDatatypes.h"
#include <larics_motion_planning/StateValidityCheckerInterface.h>
#include <larics_motion_planning/MapInterface.h>

#include <iostream>
#include <string>
#include <memory>
#include <cmath>
#include <Eigen/Eigen>

#include "yaml-cpp/yaml.h"

using namespace std;

/// This class check if a single point is in obstacle or not.
class BallStateValidityChecker : public StateValidityCheckerInterface
{
  public:
    /// \brief Constructor.
    /// \param config_filename Configuration filename for validity checker.
    /// \param map Map interface that is used for checking validity of a point.
    BallStateValidityChecker(string config_filename, 
      shared_ptr<MapInterface> map, string type);

    /// \brief Validity of a robot state.
    /// \param state Vector of robot state.
    /// \return True if robot state does not interfere with obstacles. False
    ///   otherwise.
    bool isStateValid(Eigen::VectorXd state);

    /// \brief Configures state validity checker from yaml file
    /// \param config_filename Path to configuration filename.
    /// \return False if configuration is unsuccessful, true otherwise.
    bool configureFromFile(string config_filename);

    /// \brief Generates a ball of points
    /// \param state Robot state, only first three DOF are taken into account.
    /// \return Matrix of points for collision checking.
    Eigen::MatrixXd generateValidityPoints(Eigen::VectorXd state);

    /// \brief Generates ball of points.
    /// \return Matrix of points for the ball.
    Eigen::MatrixXd generateBall();

    /// \brief Generates sphere of points.
    /// \return Matrix of points for the sphere.
    Eigen::MatrixXd generateSphere();

  private:
    shared_ptr<MapInterface> map_;
    string checker_type_;
    double ball_radius_, ball_resolution_;
    double sphere_radius_, sphere_resolution_;
};

#endif // BALL_STATE_VALIDITY_CHECKER_H