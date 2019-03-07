/// \file ToppraTrajectory.h

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

/// \brief Contains the implementation for time optimal path parametrization 
///        with reachability analysis trajectory generation algorithm.

/// This class handles the implementation of TOPP-RA trajectory. Since this
/// algorithm is implemented in Python, this class will communicate with the 
/// ROS node that generates trajectory through service. Therefore, this class
/// must be used within ROS node with valid ros::init statement before creating
/// this object.
class ToppraTrajectory : public TrajectoryInterface
{
  public:
    /// \brief Constructor with filename.
    /// \param config_filename This constructor takes .yaml file configuration for constraints and
    ///   sampling frequency.
    ToppraTrajectory(string config_filename);

    /// \brief Constructor with config_matrix
    /// \param config_matrix This constructor takes configuration matrix for velocity and acceleration
    ///   contstraints. First row of matrix corresponds to velocity constraints, 
    ///   second row to acceleration constraints. Both rows must have the same
    ///   number of elements.
    ToppraTrajectory(Eigen::MatrixXd config_matrix);

    /// \brief Destructor.
    ~ToppraTrajectory();

    /// \brief Generates TOPP-RA trajectory through specified waypoints taking
    ///   constraints into consideration.
    /// \param positions The waypoints are passed through a matrix. Each row of matrix is one
    ///   one waypoint. Number of columns represents the degrees of freedom.
    /// \return Success of trajectory generation.
    bool generateTrajectory(Eigen::MatrixXd positions);

    /// \brief Returns the generated trajectory.
    ///
    /// \return Sampled trajectory.
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