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

/// Uses direct and inverse kinematics of multiple manipulators to check
/// if system state is valid. It uses direct kinematics to get positions of 
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

    /// \brief Generates validity points for all manipulators and the 
    ///   manipulated object
    /// \param state 6-DoF state of the object. The full state can be obtained
    ///   through kinematics.
    /// \return Validity points of all robots and the object.
    Eigen::MatrixXd generateObjectValidityPoints(Eigen::VectorXd state);

  private:
    // Map and kinematics
    shared_ptr<MapInterface> map_;
    shared_ptr<MultipleManipulatorsKinematics> kinematics_;

    // Since there are multiple manipulators, everything will be stored in
    // vectors. Number of manipulators is naturally integer.
    int n_manipulators_;

    // Link dimensions are needed to generate validity points for each link,
    // and directions are required to know in which direction of its root
    // joint is the link extending.
    std::vector<Eigen::MatrixXd> link_dimensions_vector_;
    std::vector< std::vector<string> > link_directions_vector_;

    // Dimensions of the manipulator base. This is required even in the case
    // the base is fixed.
    std::vector<Eigen::Vector3d> base_dimensions_;

    // The base and links are discretized as prisms of some dimensions. The
    // resolution can be set differently.
    std::vector<double> base_sampling_resolutions_, 
      manipulator_sampling_resolutions_;

    // Fixed transform between base and first joint of the manipulator.
    std::vector<Eigen::Affine3d> t_base_manipulator_vector_;

    // Tool configuration. Flag if tool is used, dimensions of the tool and
    // direction the tool is set in. The tool will be considered as prism.
    std::vector<bool> use_tool_vector_;
    std::vector<Eigen::Vector3d> tool_dimensions_vector_;
    std::vector<string> tool_direction_vector_;

    // The configuration vector will contain all manipulators' DoF. Start and
    // end indexes define part of that vector that belongs to a single
    // manipulator.
    std::vector<int> start_indexes_, end_indexes_;
    
    // Number of degrees of freedom of the base.
    std::vector<int> base_dof_;

    // Object planner related variables.
    bool object_planner_is_used_;
    double object_planner_resolution_;
    Eigen::Vector3d object_dimensions_;

    Eigen::MatrixXd generatePrism(double x, double y, double z, 
      double resolution, string direction);

    Eigen::MatrixXd generateObject(Eigen::Vector3d dimensions,
      double resolution);
};

#endif // MULTIPLE_MANIPULATORS_STATE_VALIDITY_CHECKER