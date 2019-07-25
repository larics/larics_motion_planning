/// \file WpManipulatorKinematics.h
/// \brief Provides implementation of West Point manipulator kinematics

#ifndef WP_MANIPULATOR_KINEMATICS_H
#define WP_MANIPULATOR_KINEMATICS_H

#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <larics_motion_planning/KinematicsInterface.h>

// Include for direct and inverse kinematics.
#include <aerial_manipulators_control/ManipulatorControl.h>

#include <string>
#include <Eigen/Eigen>

#include "yaml-cpp/yaml.h"


using namespace std;

/// \brief Provides direct and inverse kinematics for WP manipulator by 
/// instantiating ROS class that computes IK through moveit.
class WpManipulatorKinematics : public KinematicsInterface
{
  public:
    /// \brief Constructor
    /// \param config_filename Path to config file.
    WpManipulatorKinematics(string config_filename);

    /// \brief Configures everything from file
    /// \param config_filename Path to config file
    /// \return true if configuration was successful, false otherwise
    bool configureFromFile(string config_filename);

    /// \brief Function that gets all joint positions in manipulator base frame.
    ///   In other words, direct kinematics to each joint and end-effector.
    /// \param q Manipulator state vector.
    /// \return List of transformations to all joints.
    std::vector<Eigen::Affine3d> getJointPositions(Eigen::VectorXd q);

    /// \brief Direct kinematics to end effector.
    /// \param q Manipulator joint state vector.
    /// \return Transform of the end effector in manipulator base frame.
    Eigen::Affine3d getEndEffectorTransform(Eigen::VectorXd q);

    /// \brief Inverse kinematics from end effector pose.
    /// \param transform Eigen::Affine3d transform of the manipulator
    ///   end-effector
    /// \param found_ik Flag determining if inverse solution was found.
    /// return Joint states for provided transform.
    Eigen::VectorXd calculateInverseKinematics(
      Eigen::Affine3d transform, bool &found_ik);

  private:
    ManipulatorControl manipulator_;
};

#endif // WP_MANIPULATOR_KINEMATICS_H