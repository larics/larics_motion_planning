/// \file GlobalPlanner.h
/// \brief Provides implementation for global path and trajectory plan.

#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <larics_motion_planning/MapInterface.h>
#include <larics_motion_planning/OctomapMap.h>
#include <larics_motion_planning/KinematicsInterface.h>
#include <larics_motion_planning/WpManipulatorKinematics.h>
#include <larics_motion_planning/MultipleManipulatorsKinematics.h>
#include <larics_motion_planning/PathPlanningInterface.h>
#include <larics_motion_planning/RrtPathPlanner.h>
#include <larics_motion_planning/TrajectoryInterface.h>
#include <larics_motion_planning/ToppraTrajectory.h>
#include <larics_motion_planning/StateValidityCheckerInterface.h>
#include <larics_motion_planning/PointStateValidityChecker.h>
#include <larics_motion_planning/SimpleStateValidityCheckers.h>
#include <larics_motion_planning/UavWpManipulatorStateValidityChecker.h>
#include <larics_motion_planning/MultipleManipulatorsStateValidityChecker.h>

#include <larics_motion_planning/ModelCorrectionInterface.h>
#include <larics_motion_planning/MultipleManipulatorsModelCorrection.h>

#include <eigen3/Eigen/Eigen>

#include "yaml-cpp/yaml.h"

#include <iostream>

using namespace std;


/// \brief This class contains implementation of global trajectory planner.
///   The intended output of this class is collision free path and/or 
///   trajectory.
class GlobalPlanner
{
  public:
    /// \brief Constructor initializes the global planner
    /// \param config_filename Path to configuration filename
    GlobalPlanner(string config_filename);

    /// \brief Configures global plinner from file.
    /// \param config_filename Path to configuration filename.
    /// \return False if configuration is unsuccessful, true otherwise.
    bool configureFromFile(string config_filename);

    /// \brief Checks path for collisions using map interface.
    /// \param path Matrix of points that need to be collision checked.
    /// \return True if path is collision free, false otherwise.
    bool collisionCheck(Eigen::MatrixXd path);
    std::tuple<bool, std::vector<int>> collisionCheckWithIndices(Eigen::MatrixXd path);

    /// \brief Plans collision free path through two or more waypoints.
    /// \param waypoints Multiple n-dimensional waypoints that are subject to
    ///   path planning. The resulting path will be concatenation of paths
    ///   between each two adjacent waypoints.
    /// \return Success of path planning algorithm.
    bool planPath(Eigen::MatrixXd waypoints);

    /// \brief Plans collision free trajectory through provided waypoints. This
    ///   function can be used as standalone trajectory planner by providing
    ///   desired waypoints.
    /// \param waypoints Multiple n-dimensional waypoints as input to
    ///   trajectory interpolation.
    /// \return Success of trajectory generation.
    bool planTrajectory(Eigen::MatrixXd waypoints);

    /// \brief Plans collision free path through provided waypoints. The path
    ///   is then interpolated with trajectory and checked for collisions.
    /// \param waypoints Two or more waypoints for path planning.
    /// \return Success of collision free trajectory generation. Returns false
    ///   if path is not collision free.
    bool planPathAndTrajectory(Eigen::MatrixXd waypoints);

    /// \brief Based on the planned and executed trajectory, the output is the
    ///   model corrected trajectory so the end-effector follows planned
    ///   trajectory.
    /// \param planned_trajectory Initially planned trajectory.
    /// \param executed_trajectory Trajectory executed through model.
    /// \return Trajectory with corrected end-effector configuration which
    //    is achieved by recalculating some or all degrees of freedom.
    Trajectory modelCorrectedTrajectory(
      Trajectory planned_trajectory, Trajectory executed_trajectory);

    /// \brief Plans a path for the object manipulated by multiple aerial
    ///   manipulators.
    /// \param waypoints Two or more 6-DoF waypoints of the manipulated object.
    /// \return Success if a collision free path is found.
    bool planObjectPath(Eigen::MatrixXd waypoints);

    /// \brief Returns planned path.
    /// \return Planned path as Eigen matrix.
    Eigen::MatrixXd getPath();

    /// \brief Returns length of the planned path.
    /// \return Path length
    double getPathLength();

    /// \brief Returns planned trajectory.
    /// \return Sampled trajectory
    Trajectory getTrajectory();

    /// \brief Returns robot state points
    /// \return State Points
    Eigen::MatrixXd getRobotStatePoints(Eigen::VectorXd state);

    /// \brief Returns kinematics interface used in planner.
    /// \return Shared pointer to kinematics interface.
    shared_ptr<KinematicsInterface> getKinematicsInterface();

    /// \brief Returns map interface used in planner
    /// \return Shared pointer to map interface.
    shared_ptr<MapInterface> getMapInterface();

    /// \brief Returns trajectory interface used in planner.
    /// \return Shared pointer to trajectory interface.
    shared_ptr<TrajectoryInterface> getTrajectoryInterface();
    
  protected:
    shared_ptr<MapInterface> map_interface_;
    shared_ptr<TrajectoryInterface> trajectory_interface_;
    shared_ptr<StateValidityCheckerInterface> state_validity_checker_interface_;
    shared_ptr<PathPlanningInterface> path_planner_interface_;
    shared_ptr<KinematicsInterface> kinematics_interface_;
    shared_ptr<ModelCorrectionInterface> model_correction_interface_;

    string state_validity_checker_type_, config_filename_;

    // Local copy of path
    Eigen::MatrixXd path_;
    int num_trajectory_restarts_, num_path_and_trajectory_restarts_;
    int num_path_restarts_, num_collision_check_restarts_;
    bool plan_path_collision_check_;

    bool planPathThroughTwoWaypoints(Eigen::MatrixXd waypoints);
};

#endif // GLOBAL_PLANNER_H