/// \file SplineInterpolator.h

#ifndef SPLINE_INTERPOLATOR_H
#define SPLINE_INTERPOLATOR_H

#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <larics_motion_planning/TrajectoryInterface.h>
#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "yaml-cpp/yaml.h"

using namespace std;

/// \brief Contains implementation of different spline orders with various
///        initial conditions.

class SplineInterpolator
{
  public:
    /// \brief Constructor
    SplineInterpolator();

    /// \brief Generates 5th order spline based on specified initial+final
    ///   conditions and dynamic constraints. At least one dynamic constraint
    ///   must be specified.
    /// \param conditions Initial and final position, velocity and acceleration.
    /// \param constraints Maximum allowed velocity, acceleration etc.
    /// \param sample_time Sampling time of the trajectory.
    /// \return True if trajectory was successfully generated.
    bool generateSplineOrder5(Eigen::VectorXd conditions, 
      Eigen::VectorXd constraints, double sample_time);

    /// \brief Generates 5th order spline with fixed duration. No constraints
    ///   are needed.
    /// \param conditions Initial and final position, velocity and acceleration.
    /// \param duration Spline duration.
    /// \param sample_time Sampling time of the trajectory.
    /// \return True if trajectory was successfully generated.
    bool generateSplineOrder5FixedTime(Eigen::VectorXd conditions, 
      double duration, double sample_time);

    /// \brief Generates a trajectory of n axes.
    /// \param conditions Initial and final position, velocity and acceleration.
    /// \param constraints Maximum allowed velocity, acceleration etc.
    /// \param sample_time Sampling time of the trajectory.
    /// \return True if trajectory was successfully generated.
    bool generateTrajectory(Eigen::MatrixXd conditions, 
      Eigen::MatrixXd constraints, double sample_time);

    /// \brief Generates a trajectory of n axes. Each axis respects it's own
    ///     constraints, finally all axes are the same length(derivatives are
    ///     set to 0 after spline's duration and position is repeated for 
    ///     the remainder of spline). Use this only when generating trajectory
    ///     that stops at the end point.
    /// \param conditions Initial and final position, velocity and acceleration.
    /// \param constraints Maximum allowed velocity, acceleration etc.
    /// \param sample_time Sampling time of the trajectory.
    /// \return True if trajectory was successfully generated.
    bool generateTrajectoryNoSync(Eigen::MatrixXd conditions, 
      Eigen::MatrixXd constraints, double sample_time);

    /// \brief Returns planned trajectory.
    /// \return Generated trajectory.
    Trajectory getTrajectory() {return trajectory_;}

    /// \brief Returns duration of the trajectory.
    /// \return Trajectory duration.
    double getTrajectoryDuration() {return spline_duration_;}

  private:
    double spline_duration_;
    Eigen::VectorXd coefficients_;
    Trajectory trajectory_, spline_;

    inline Eigen::VectorXd getSplineOrder5Coefficients(
      Eigen::VectorXd conditions, Eigen::VectorXd t);

    inline Eigen::VectorXd calculatePolynomialValueOrder5(
      Eigen::VectorXd coefficients, double time);

    Trajectory sampleTrajectory(Eigen::VectorXd coefficients, 
      double duration, double sample_time);

    double calculateTimeScalingFactor(Eigen::VectorXd coefficients, 
      Eigen::VectorXd constraints, double duration, double sample_time);
};

#endif // SPLINE_INTERPOLATOR_H