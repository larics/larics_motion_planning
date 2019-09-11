/// \file Local.h
/// \brief Provides implementation for local planner that executes trajectory
///   and locally searches for obstacles and compensates manipulator pose


#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <larics_motion_planning/MapInterface.h>
#include <larics_motion_planning/OctomapMap.h>
#include <larics_motion_planning/KinematicsInterface.h>
#include <larics_motion_planning/WpManipulatorKinematics.h>
#include <larics_motion_planning/PathPlanningInterface.h>
#include <larics_motion_planning/RrtPathPlanner.h>
#include <larics_motion_planning/TrajectoryInterface.h>
#include <larics_motion_planning/ToppraTrajectory.h>
#include <larics_motion_planning/StateValidityCheckerInterface.h>
#include <larics_motion_planning/PointStateValidityChecker.h>
#include <larics_motion_planning/UavWpManipulatorStateValidityChecker.h>

#include <eigen3/Eigen/Eigen>

#include "yaml-cpp/yaml.h"

#include <iostream>
using namespace std;

/// \brief This class implements local planner strategies and ideas.
class LocalPlanner
{
  public:
    /// \brief Constructor initializes local planner.
    /// \param config_filename Path to configuration filename.
    LocalPlanner(string config_filename);

    /// \brief Configures local planner from file.
    /// \param config_filename Path to configuration file.
    /// \return True if configuration was successful, false otherwise.
    bool configureFromFile(string config_filename);
};


#endif // LOCAL_PLANNER_H