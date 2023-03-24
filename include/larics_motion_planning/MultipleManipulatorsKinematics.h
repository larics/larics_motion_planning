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
#include <math.h>

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

    /// \brief Function that returns all joint transforms of a single manipulator.
    /// \param q Single manipulator q.
    /// \param id Manipulator index.
    /// \return List of transformations to all joints of manipulators.
    std::vector<Eigen::Affine3d> getSingleManipulatorJointPositions(
      Eigen::VectorXd q, int id);

    /// \brief Direct kinematics to end effector.
    /// \param q Manipulator joint state vector.
    /// \return Transform of the end effector in manipulator base frame.
    Eigen::Affine3d getEndEffectorTransform(Eigen::VectorXd q);

    /// \brief Inverse kinematics from end effector pose.
    /// \param transform Eigen::Affine3d transform of the manipulator
    ///   end-effector
    /// \param found_ik Flag determining if inverse solution was found.
    /// \return Joint states for provided transform.
    Eigen::VectorXd calculateInverseKinematics(
      Eigen::Affine3d transform, bool &found_ik);

    /// \brief Direct kinematics for multiple manipulators.
    /// \param q Joint values of the manipulator.
    /// \return Eigen::Affine3d transform of the manipulator
    ///   end-effector. This is further transformed by grasp_transform of
    ///   each manipulator to obtain each end-effector configuration.
    std::vector<Eigen::Affine3d> getMultipleEndEffectorTransforms(
      Eigen::VectorXd q);

    /// \brief Direct kinematics for single manipulator.
    /// \param q Joint values of the manipulator.
    /// \param id Manipulator index.
    /// \return Eigen::Affine3d transform of the manipulator
    ///   end-effector. This is further transformed by grasp_transform of
    ///   each manipulator to obtain each end-effector configuration.
    Eigen::Affine3d getSingleManipulatorEndEffectorTransform(
      Eigen::VectorXd q, int id);

    /// \brief Inverse kinematics for a single manipulator
    /// \param transform Eigen::Affine3d transform of the manipulator
    ///   end-effector. This is for now multiplied by grasp transform
    /// \param id Manipulator index.
    /// \param found_ik Flag determining if inverse solution was found.
    /// \return Joint states to satisfy provided transform.
    Eigen::VectorXd calculateSingleManipulatorInverseKinematics(
      Eigen::Affine3d transform, int id, bool &found_ik);

    /// \brief Get jacobian matrix for single manipulator
    /// \param q Joint positions.
    /// \param id Manipulator index.
    /// \return Jacobian of manipulator id.
    Eigen::MatrixXd getSingleManipulatorJacobian(Eigen::VectorXd q, int id);

    /// \brief Sets single manipulator joint positions.
    /// \param joint_positions Manipulator configuration to be set.
    /// \param id Manipulator index.
    void setSingleManipulatorJointPositions(Eigen::VectorXd q, int id);

    /// \brief Based on the object state, get full aerial manipulator state
    ///   under some assumptions. First is that the kinematics module of each
    ///   manipulator can provide best fitting q_m for the given object state.
    ///   Second is hovering assumption, or in case of static manipulator,
    ///   that the base is in horizontal position.
    /// \param object_q Object state that is 6-DoF.
    /// \param id Manipulator index.
    /// \return Full state of a single manipulator with base and joints.
    Eigen::VectorXd getSingleManipulatorStateFromObjectState(
      Eigen::VectorXd object_q, int id);

    /// \brief Based on the object state, get full system state. Will call
    ///   getSingleManipulatorStateFromObjectState() for each manipulator and
    ///   create the full state.
    /// \param object_q Object state that is 6-DoF.
    /// \return Full state of the multiple manipulator system.
    Eigen::VectorXd getFullSystemStateFromObjectState(
      Eigen::VectorXd object_q);

    /// \brief Based on the object state, get full system state for multiple
    ///   waypoints.
    /// \param object_q Matrix of object state that is 6-DoF.
    /// \return Matrix of full state of the multiple manipulator system.
    Eigen::MatrixXd getFullSystemStateFromObjectState(
      Eigen::MatrixXd object_q);

  private:
    //ManipulatorControl manipulator_;
    // Container of multiple manipulators
    std::vector<shared_ptr<KinematicsInterface> > manipulators_;

    // Number of manipulators
    int n_manipulators_;

    // Degrees of freedom and indexes of these dofs
    std::vector<int> n_dofs_;
    std::vector< std::vector<int> > dofs_indexes_;

    // Transform from base to manipulator
    std::vector<int> base_n_dofs_;
    std::vector<Eigen::Affine3d> t_base_manipulator_vector_;
    std::vector<double> base_relative_yaw_vector_;

    // Grasp transforms. The idea of this variable is to provide one
    // end-effector transform and get all manipulators transforms by multiplying
    // them with appropriate grasp transform.
    std::vector<Eigen::Affine3d> grasp_transforms_;
};

double wrapToPi(double x);

#endif // MULTIPLE_MANIPULATORS_KINEMATICS_H