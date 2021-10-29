/// \file MultipleManipulatorsModelCorrection.h
/// \brief This file provides with model corrections for multiple manipulators.

#ifndef MULTIPLE_MANIPULATORS_MODEL_CORRECTION
#define MULTIPLE_MANIPULATORS_MODEL_CORRECTION

// #include "MotionPlanningDatatypes.h"
#include <larics_motion_planning/KinematicsInterface.h>
#include <larics_motion_planning/MultipleManipulatorsKinematics.h>
#include <larics_motion_planning/ModelCorrectionInterface.h>
#include <larics_motion_planning/UavWpManipulatorModelCorrection.h>

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
/// unforseen/unmodelled parameters. Here, multiple manipulators that are
/// manipulating same object are taken into account. Each manipuator is
// corrected independently.
class MultipleManipulatorsModelCorrection : public ModelCorrectionInterface
{
  public:
    /// \brief Constructor.
    /// \param config_filename Configuration filename for validity checker.
    /// \param kinematics Direct and inverse kinematics for multiple
    ///   manipulators.
    MultipleManipulatorsModelCorrection(string config_filename,
      shared_ptr<KinematicsInterface> kinematics);

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
    // Kinematics will be cast into multiple manipulator kinematics because
    // that implementation offers direct and inverse kinematics for multiple
    // manipulators.
    shared_ptr<MultipleManipulatorsKinematics> kinematics_;

    // Since there are multiple manipulators, everything will be stored in
    // vectors. Number of manipulators is naturally integer.
    int n_manipulators_;

    // Container for multiple manipulators model corrections
    // Container of multiple manipulators
    std::vector<shared_ptr<ModelCorrectionInterface> > manipulators_;

    // The configuration vector will contain all manipulators' DoF. Start and
    // end indexes define part of that vector that belongs to a single
    // manipulator.
    std::vector<int> start_indexes_, end_indexes_;
};

#endif // MULTIPLE_MANIPULATORS_MODEL_CORRECTION