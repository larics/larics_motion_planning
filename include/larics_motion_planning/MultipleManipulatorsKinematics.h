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

    /// \brief Function that returns all joint transforms of all manipulators.
    /// \param q Full system state vector.
    /// \return List of transformations to all joints of manipulators.
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

    /// \brief Direct kinematics for multiple manipulators.
    /// \param q 
    /// \return Eigen::Affine3d transform of the manipulator
    ///   end-effector. This is further transformed by grasp_transform of
    ///   each manipulator to obtain each end-effector configuration.
    std::vector<Eigen::Affine3d> getMultipleEndEffectorTransforms(
      Eigen::VectorXd q);

  private:
    //ManipulatorControl manipulator_;
    // Container of multiple manipulators
    std::vector<shared_ptr<KinematicsInterface> > manipulators_;

    // Number of manipulators
    int n_manipulators_;

    // Degrees of freedom and indexes of these dofs
    std::vector<int> n_dofs_;
    std::vector< std::vector<int> > dofs_indexes_;

    // Grasp transforms
    std::vector<Eigen::Affine3d> grasp_transforms_;
};

#endif // MULTIPLE_MANIPULATORS_KINEMATICS_H