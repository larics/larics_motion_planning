/// \file MultipleManipulatorsKinematics.h
/// \brief Provides kinematics for multiple manipulators

#ifndef MULTIPLE_MANIPULATORS_KINEMATICS_H
#define MULTIPLE_MANIPULATORS_KINEMATICS_H

#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <larics_motion_planning/KinematicsInterface.h>
#include <larics_motion_planning/WpManipulatorKinematics.h>

// Include for direct and inverse kinematics.
#include <aerial_manipulators_control/ManipulatorControl.h>

#include <string>
#include <Eigen/Eigen>

#include "yaml-cpp/yaml.h"


using namespace std;

/// \brief Provides kinematics for multiple manipulators by instantiating them
///   as objects.
class MultipleManipulatorsKinematics : public KinematicsInterface
{
  public:
    /// \brief Constructor
    /// \param config_filename Path to config file.
    MultipleManipulatorsKinematics(string config_filename);

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
    //ManipulatorControl manipulator_;
    // Container of multiple manipulators
    std::vector<shared_ptr<KinematicsInterface> > manipulators_;

    // Number of manipulators
    int n_manipulators_;
};

#endif // MULTIPLE_MANIPULATORS_KINEMATICS_H