/// \file TrajectoryInterface.h
/// \brief Contains the abstract class interface for trajectory.

#ifndef TRAJECTORY_INTERFACE_H
#define TRAJECTORY_INTERFACE_H

//#define _GLIBCXX_USE_CXX11_ABI 0

#include <iostream>
#include <eigen3/Eigen/Eigen>

/// This is an interface class for trajectories of all types.
class TrajectoryInterface
{
  public:
    TrajectoryInterface();
    ~TrajectoryInterface();

    //virtual void generateTrajectory() = 0;
    //virtual void sampleTrajectory() = 0;
    //virtual void setConfigFromFile() = 0;

  protected:
    Eigen::MatrixXd sampled_trajectory_;
    Eigen::MatrixXd dynamic_constraints_;
    int n_dofs_;
};

#endif // TRAJECTORY_INTERFACE_H