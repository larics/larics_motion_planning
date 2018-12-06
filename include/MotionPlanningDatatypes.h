/// \file MotionPlanningDatatypes.h
/// \brief This file contains datatypes used within larics motion planning
/// package.

#ifndef MOTION_PLANNING_DATATYPES_H
#define MOTION_PLANNING_DATATYPES_H

#include <eigen3/Eigen/Eigen>

typedef struct{
  Eigen::MatrixXd position;
  Eigen::MatrixXd velocity;
  Eigen::MatrixXd acceleration;
  Eigen::MatrixXd jerk;
  Eigen::MatrixXd split;
  Eigen::VectorXd time;
} Trajectory;


#endif // MOTION_PLANNING_DATATYPES_H