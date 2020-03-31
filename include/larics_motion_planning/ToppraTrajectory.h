/// \file ToppraTrajectory.h

#ifndef TOPPRA_TRAJECTORY_H
#define TOPPRA_TRAJECTORY_H

#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <larics_motion_planning/TrajectoryInterface.h>
#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <topp_ros/GenerateTrajectory.h>
#include "yaml-cpp/yaml.h"

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
    /// \param sampling_frequency Sampling frequency for the trajectory.
    ToppraTrajectory(Eigen::MatrixXd config_matrix, double sampling_frequency);

    /// \brief Destructor.
    ~ToppraTrajectory();

    /// \brief Configures trajectory planner from file.
    /// \param config_filename Path to the .yaml file for configuration.
    /// \return True if configuration provided is valid.
    bool configureFromFile(string config_filename);

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

    /// \brief Set dynamic constraints.
    /// \param dynamic_constraints New velocity and acceleration constraints.
    /// \return True if dimensions are proper and constraints are set.
    bool setDynamicConstraints(Eigen::MatrixXd dynamic_constraints);

  private:
    ros::NodeHandle nh_;
    ros::ServiceClient generate_trajectory_client_;
    Trajectory sampled_trajectory_;
    Eigen::MatrixXd dynamic_constraints_;
    std::vector<int> is_angular_;
    int n_dofs_;
    double sampling_frequency_;

    void sampleTrajectory(
      trajectory_msgs::JointTrajectory joint_trajectory);
};

#endif // TOPPRA_TRAJECTORY_H