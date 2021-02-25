/// \file UavWpManipulatorStateValidityChecker.h
/// \brief This file provides with a method to check if state of a UAV with 
///   the wp manipulator is valid.

#ifndef UAV_WP_MANIPULATOR_STATE_VALIDITY_CHECKER
#define UAV_WP_MANIPULATOR_STATE_VALIDITY_CHECKER

// #include "MotionPlanningDatatypes.h"
#include <larics_motion_planning/StateValidityCheckerInterface.h>
#include <larics_motion_planning/MapInterface.h>
#include <larics_motion_planning/KinematicsInterface.h>

#include <iostream>
#include <string>
#include <memory>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "yaml-cpp/yaml.h"

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
      shared_ptr<MapInterface> map, shared_ptr<KinematicsInterface> kinematics);

    /// \brief Configures state validity checker from yaml file
    /// \param config_filename Path to configuration filename.
    /// \return False if configuration is unsuccessful, true otherwise.
    bool configureFromFile(string config_filename);

    /// \brief Validity of a robot state.
    /// \param state Vector of robot state.
    /// \return True if robot state does not interfere with obstacles. False
    ///   otherwise.
    bool isStateValid(Eigen::VectorXd state);

    //bool isStateValid(Eigen::VectorXd state, double roll=0.0, double pitch=0.0);

    /// \brief Generates points that are to be checked based on the state of
    ///   the aerial manipulator.
    /// \param state Robot state with (x, y, z, yaw, q(5))
    /// \param roll UAV roll angle. Default is zero.
    /// \param pitch UAV pitch angle. Default iz zero.
    /// \return Matrix of points for collision checking.
    Eigen::MatrixXd generateValidityPoints(Eigen::VectorXd state);

    void testDirectKinematics();

  private:
    shared_ptr<MapInterface> map_;
    shared_ptr<KinematicsInterface> kinematics_;
    Eigen::MatrixXd link_dimensions_;
    std::vector<string> link_directions_;
    Eigen::Vector3d uav_dimensions_;
    double uav_sampling_resolution_, manipulator_sampling_resolution_;
    int num_joints_;

    Eigen::Affine3d t_uav_manipulator_;

    bool use_tool_;
    Eigen::Vector3d tool_dimensions_;
    string tool_direction_;

    Eigen::MatrixXd generatePrism(double x, double y, double z, 
      double resolution, string direction);
};

#endif // UAV_WP_MANIPULATOR_STATE_VALIDITY_CHECKER