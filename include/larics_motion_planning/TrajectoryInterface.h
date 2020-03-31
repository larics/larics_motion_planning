/// \file TrajectoryInterface.h
/// \brief Contains the abstract class interface for trajectory.

#ifndef TRAJECTORY_INTERFACE_H
#define TRAJECTORY_INTERFACE_H

#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <iostream>

using namespace std;

/// This is an interface class for trajectories of all types.
class TrajectoryInterface
{
  public:
    TrajectoryInterface();
    virtual bool generateTrajectory(Eigen::MatrixXd positions) = 0;
    virtual Trajectory getTrajectory() = 0;
    virtual bool configureFromFile(string config_filename) = 0;
    virtual bool setDynamicConstraints(Eigen::MatrixXd dynamic_constraints) {}
};

#endif // TRAJECTORY_INTERFACE_H