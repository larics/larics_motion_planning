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
#include <math.h>

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

    /// \brief Constructor
    /// \param robot_model_name Robot model name for moveit configuration.
    /// \param joint_group_name Joint group name for moveit configuration.
    /// \param dh_parameters_file DH parameters file location required by
    ///   ManipulatorControl.
    WpManipulatorKinematics(string robot_model_name, 
      string joint_group_name, string dh_parameters_file);

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
    /// \return Joint states for provided transform.
    Eigen::VectorXd calculateInverseKinematics(
      Eigen::Affine3d transform, bool &found_ik);

    /// \brief Sets joint positions so inverse kinematics optimization can
    ///   start from that point.
    /// \param joint_positions Manipulator configuration.
    void setJointPositions(Eigen::VectorXd joint_positions);

    /// \brief Returns the jacobian matrix.
    /// \param q Manipulator joint values.
    /// \return Jacobian matrix.
    Eigen::MatrixXd getJacobian(Eigen::VectorXd q);

    /// \brief This is intended to be used in the multiple manipulators setting
    ///   where the idea is to plan from the perspective of the manipulated
    ///   object. Based on object world transform and fixed grasp transform
    ///   calculate the joint positions required to achieve that, with the
    ///   internal assumption of multirotor hover.
    /// \param T_w_t Transform of the tool in the world frame. This is required
    ///   to obtain the roll and pitch of the end-effector.
    /// \param object_state 6-DoF object state with appropriate angles.
    /// \return Joint states for the required configuration.
    Eigen::VectorXd calculateOptimalSingleManipulatorState(
      Eigen::Affine3d grasp_transform, Eigen::VectorXd object_state);

  private:
    ManipulatorControl manipulator_;
    string robot_model_name_;
};

double signum(double val);

#endif // WP_MANIPULATOR_KINEMATICS_H