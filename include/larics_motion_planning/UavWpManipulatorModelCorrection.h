/// \file UavWpManipulatorModelCorrection.h
/// \brief This file provides model corrections for UavAndWpManipulator type.

#ifndef UAV_WP_MANIPULATOR_MODEL_CORRECTION
#define UAV_WP_MANIPULATOR_MODEL_CORRECTION

// #include "MotionPlanningDatatypes.h"
#include <larics_motion_planning/KinematicsInterface.h>
#include <larics_motion_planning/MultipleManipulatorsKinematics.h>
#include <larics_motion_planning/ModelCorrectionInterface.h>

#include <iostream>
#include <string>
#include <memory>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "yaml-cpp/yaml.h"

using namespace std;

/// After some trajectory has been executed in simulation environment, it is
/// recorded. Then, the end-effector configuration can be corrected due to
/// unforseen/unmodelled parameters. This file provides model corrections
/// for UavWpManipulator type, where the manipulator can have various number of
/// degrees of freedom.
/// Also, this has multiple constructors since it can be used as a standalone
/// model correction or as a part of multiple manipulators model correction.
class UavWpManipulatorModelCorrection : public ModelCorrectionInterface
{
  public:
    /// \brief Standalone constructor.
    /// \param config_filename Configuration filename for validity checker.
    /// \param kinematics Direct and inverse kinematics for multiple
    ///   manipulators.
    UavWpManipulatorModelCorrection(string config_filename,
      shared_ptr<KinematicsInterface> kinematics);

    /// \brief Multiple manipulators constructor.
    /// \param config Yaml node configuration passed by multiple manipulators.
    /// \param kinematics Direct and inverse kinematics for multiple
    ///   manipulators.
    /// \param id Manipulator index.
    UavWpManipulatorModelCorrection(YAML::Node config,
      shared_ptr<MultipleManipulatorsKinematics> kinematics, int id);

    /// \brief Configures state validity checker from yaml file
    /// \param config_filename Path to configuration filename.
    /// \return False if configuration is unsuccessful, true otherwise.
    bool configureFromFile(string config_filename);

    /// \brief Corrects each manipulator end-effector separately nad returns
    ///   the full trajectory.
    /// \param planned_trajectory Initially planned trajectory.
    /// \param executed_trajectory Trajectory executed through model.
    /// \return Trajectory with corrected end-effector configuration which
    //    is achieved by recalculating some or all degrees of freedom.
    Trajectory modelCorrectedTrajectory(Trajectory planned_trajectory,
      Trajectory executed_trajectory);

  private:
    // Kinematics interface.
    shared_ptr<KinematicsInterface> kinematics_;
    // If multiple manipulators are used, kinematics will be cast into multiple
    // manipulator kinematics because that implementation offers direct and
    // inverse kinematics for multiple manipulators.
    shared_ptr<MultipleManipulatorsKinematics> mm_kinematics_;

    // If multiple manipulators kinematics are used, then the id will determine
    // which manipulator are we accessing from here. The id will be set to -1
    // if standalone kinematics are used.
    int id_;

    // Transformation between body and manipulator.
    Eigen::Affine3d t_b_l0_;

    // Manipulator and uav degrees of freedom
    int manipulator_dof_, uav_dof_;

    // End-effector q is calculated with some weighing factor alpha
    double alpha_;

    // Gains for roll and pitch corrections
    double roll_correction_gain_, pitch_correction_gain_;
};

#endif // UAV_WP_MANIPULATOR_MODEL_CORRECTION