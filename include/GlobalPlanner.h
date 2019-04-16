/// \file GlobalPlanner.h
/// \brief Provides implementation for global path and trajectory plan.

#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <MotionPlanningDatatypes.h>
#include <MapInterface.h>
#include <OctomapMap.h>
#include <PathPlanningInterface.h>
#include <RrtPathPlanner.h>
#include <TrajectoryInterface.h>
#include <ToppraTrajectory.h>

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

    /// \brief Returns planned path.
    /// \return Planned path as Eigen matrix.
    Eigen::MatrixXd getPath();

    /// \brief Returns planned trajectory.
    /// \return Sampled trajectory
    Trajectory getTrajectory();

  private:
    shared_ptr<MapInterface> map_interface_;
    shared_ptr<TrajectoryInterface> trajectory_interface_;
    shared_ptr<PathPlanningInterface> path_planner_interface_;

    // Local copy of path
    Eigen::MatrixXd path_;
    int num_trajectory_restarts_, num_path_and_trajectory_restarts_;
    int num_path_restarts_, num_collision_check_restarts_;
    bool plan_path_collision_check_;

    bool planPathThroughTwoWaypoints(Eigen::MatrixXd waypoints);
};

#endif // GLOBAL_PLANNER_H