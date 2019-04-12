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
    /// \return Success of collision free trajectory generation.
    bool planTrajectory(Eigen::MatrixXd waypoints);

  private:
    shared_ptr<MapInterface> map_;
    shared_ptr<TrajectoryInterface> trajectory_;
    shared_ptr<PathPlanningInterface> path_planner_;
};

#endif // GLOBAL_PLANNER_H