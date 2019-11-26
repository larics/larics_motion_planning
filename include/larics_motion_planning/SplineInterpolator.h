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
    bool generateSplineOrder5(Eigen::VectorXd conditions, 
      Eigen::VectorXd constraints, double sample_time);

  private:
    double t_spline_;
    Eigen::VectorXd coefficients_;

    inline Eigen::VectorXd getSplineOrder5Coefficients(
      Eigen::VectorXd conditions, Eigen::VectorXd t);
};

#endif // SPLINE_INTERPOLATOR_H