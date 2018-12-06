/// \file ToppraTrajectory.h
/// \brief Contains the implementation for time optimal path parametrization
/// with reachability analysis trajectory generation algorithm

#ifndef TOPPRA_TRAJECTORY_H
#define TOPPRA_TRAJECTORY_H

#include "MotionPlanningDatatypes.h"
#include "TrajectoryInterface.h"
#include <iostream>
#include <string>
#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <topp_ros/GenerateTrajectory.h>

using namespace std;

/// This class handles the implementation of TOPP-RA trajectory. Since this
/// algorithm is implemented in Python, this class will communicate with the 
/// ROS node that generates trajectory through service. Therefore, this class
/// must be used within ROS node with valid ros::init statement before creating
/// this object.
class ToppraTrajectory : public TrajectoryInterface
{
  public:
    ToppraTrajectory(string config_filename);
    ToppraTrajectory(Eigen::MatrixXd config_matrix);
    ~ToppraTrajectory();
    bool generateTrajectory(Eigen::MatrixXd positions);
    Trajectory getTrajectory();

  private:
    ros::NodeHandle nh_;
    ros::ServiceClient generate_trajectory_client_;
    Trajectory sampled_trajectory_;
    Eigen::MatrixXd dynamic_constraints_;
    int n_dofs_;

    void sampleTrajectory(
      trajectory_msgs::JointTrajectory joint_trajectory);
};

#endif // TOPPRA_TRAJECTORY_H