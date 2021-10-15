/// \file MultipleManipulatorsStateValidityChecker.h
/// \brief This file provides with a method to check if state of multiple
///   manipulators is valid.

#ifndef MULTIPLE_MANIPULATORS_STATE_VALIDITY_CHECKER
#define MULTIPLE_MANIPULATORS_STATE_VALIDITY_CHECKER

// #include "MotionPlanningDatatypes.h"
#include <larics_motion_planning/StateValidityCheckerInterface.h>
#include <larics_motion_planning/MapInterface.h>
#include <larics_motion_planning/KinematicsInterface.h>
#include <larics_motion_planning/MultipleManipulatorsKinematics.h>

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
class MultipleManipulatorsStateValidityChecker : public StateValidityCheckerInterface
{
  public:
    /// \brief Constructor.
    /// \param config_filename Configuration filename for validity checker.
    /// \param map Map interface that is used for checking validity of the
    ///   system
    MultipleManipulatorsStateValidityChecker(string config_filename, 
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
    ///   the manipulator with its base.
    /// \param state Single manipulator state that has (x,y,z,roll,pitch,yaw)
    ///   of the base and n DoF manipulator.
    /// \param id Index of the manipulator for which the validity points are
    ///   being generated.
    /// \return Mx3 matrix of points to be checked for validity.
    Eigen::MatrixXd generateSingleManipulatorValidityPoints(
      Eigen::VectorXd state, int id);

    /// \brief Generates validity points for all manipulators.
    /// \param state Full state of the robot. This will be segmented through
    ///   indexes parameter to get appropriate state.
    /// \return Validity points of all robots.
    Eigen::MatrixXd generateValidityPoints(Eigen::VectorXd state);

    void testDirectKinematics();

  private:
    // Map and kinematics
    shared_ptr<MapInterface> map_;
    shared_ptr<MultipleManipulatorsKinematics> kinematics_;

    // Since there are multiple manipulators, everything will be stored in
    // vectors
    std::vector<Eigen::MatrixXd> link_dimensions_vector_;
    Eigen::MatrixXd link_dimensions_;
    std::vector< std::vector<string> > link_directions_vector_;
    std::vector<string> link_directions_;

    std::vector<Eigen::Vector3d> base_dimensions_;
    Eigen::Vector3d uav_dimensions_;

    std::vector<double> base_sampling_resolutions_, 
      manipulator_sampling_resolutions_;
    double uav_sampling_resolution_, manipulator_sampling_resolution_;
    int num_joints_, n_manipulators_;

    std::vector<Eigen::Affine3d> t_uav_manipulator_vector_;
    Eigen::Affine3d t_uav_manipulator_;

    std::vector<bool> use_tool_vector_;
    std::vector<Eigen::Vector3d> tool_dimensions_vector_;
    std::vector<string> tool_direction_vector_;
    bool use_tool_;
    Eigen::Vector3d tool_dimensions_;
    string tool_direction_;

    std::vector<int> start_indexes_;
    std::vector<int> end_indexes_;
    std::vector<int> base_dof_;

    Eigen::MatrixXd generatePrism(double x, double y, double z, 
      double resolution, string direction);
};

#endif // MULTIPLE_MANIPULATORS_STATE_VALIDITY_CHECKER