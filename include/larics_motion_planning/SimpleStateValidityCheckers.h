/// \file SimpleStateValidityCheckers.h
/// \brief This class contains several simple geometric shapes as state 
///   validity checkers. It features point, sphere and ball states.

#ifndef SIMPLE_STATE_VALIDITY_CHECKERS_H
#define SIMPLE_STATE_VALIDITY_CHECKERS_H

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
class SimpleStateValidityCheckers : public StateValidityCheckerInterface
{
  public:
    /// \brief Constructor.
    /// \param config_filename Configuration filename for validity checker.
    /// \param map Map interface that is used for checking validity of a point.
    SimpleStateValidityCheckers(string config_filename, 
      shared_ptr<MapInterface> map, string type);

    /// \brief Constructor that does not need file config.
    /// \param map Map interface that is used for checking validity of a point.
    /// \param type Type of state validity checker.
    /// \param configuration Configuration vector for state validity checker.
    SimpleStateValidityCheckers(shared_ptr<MapInterface> map, string type, 
      Eigen::VectorXd configuration);

    /// \brief Validity of a robot state.
    /// \param state Vector of robot state.
    /// \return True if robot state does not interfere with obstacles. False
    ///   otherwise.
    bool isStateValid(Eigen::VectorXd state);

    /// \brief Configures state validity checker from yaml file
    /// \param config_filename Path to configuration filename.
    /// \return False if configuration is unsuccessful, true otherwise.
    bool configureFromFile(string config_filename);

    /// \brief Since points are generated in constructor, this translates 
    ///   them to robot current position.
    /// \param state Robot state, only first three DOF are taken into account.
    /// \return Matrix of points for collision checking.
    Eigen::MatrixXd generateValidityPoints(Eigen::VectorXd state);

    /// \brief Generates ball of points.
    /// \return Matrix of points for the ball.
    Eigen::MatrixXd generateBall();

    /// \brief Generates sphere of points.
    /// \return Matrix of points for the sphere.
    Eigen::MatrixXd generateSphere();

    /// \brief Generates a single point.
    /// \return Matrix that contains only that point.
    Eigen::MatrixXd generatePoint();

    /// \brief Generates a circle.
    /// \return Matrix that contains circle of points.
    Eigen::MatrixXd generateCircle(double z=0.0);

    /// \brief Generates a cylinder.
    /// \return Matrix that contains cylinder of points.
    Eigen::MatrixXd generateCylinder();

    /// \brief Generates a rectangle.
    /// \return Matrix that contains rectangle of points.
    Eigen::MatrixXd generateRectangle(double z=0.0);

    /// \brief Generates a prism.
    /// \return Matrix that contains prism of points.
    Eigen::MatrixXd generatePrism();


  private:
    shared_ptr<MapInterface> map_;
    string checker_type_;
    double ball_radius_, ball_resolution_;
    double sphere_radius_, sphere_resolution_;
    double circle_radius_, circle_resolution_;
    double cylinder_radius_, cylinder_resolution_, cylinder_height_;
    double rectangle_x_, rectangle_y_, rectangle_resolution_;
    double prism_x_, prism_y_, prism_z_, prism_resolution_;
};

#endif // SIMPLE_STATE_VALIDITY_CHECKERS_H