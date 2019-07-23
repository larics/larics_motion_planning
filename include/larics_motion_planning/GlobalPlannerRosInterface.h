/// \file GlobalPlannerRosInterface.h
/// \brief ROS interface with services and topics for global planner.

#ifndef GLOBAL_PLANNER_ROS_INTERFACE_H
#define GLOBAL_PLANNER_ROS_INTERFACE_H

#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <larics_motion_planning/MapInterface.h>
#include <larics_motion_planning/OctomapMap.h>
#include <larics_motion_planning/PathPlanningInterface.h>
#include <larics_motion_planning/RrtPathPlanner.h>
#include <larics_motion_planning/TrajectoryInterface.h>
#include <larics_motion_planning/ToppraTrajectory.h>
#include <larics_motion_planning/GlobalPlanner.h>
#include <larics_motion_planning/Visualization.h>

#include <eigen3/Eigen/Eigen>

#include "yaml-cpp/yaml.h"

#include <ros/ros.h>

// Messages
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
// Services
#include <std_srvs/Empty.h>
#include <larics_motion_planning/CartesianTrajectory.h>
#include <larics_motion_planning/MultiDofTrajectory.h>
#include <larics_motion_planning/VisualizeState.h>

#include <iostream>
using namespace std;

/// \brief Interfaces global planner with ros through services and topics.
class GlobalPlannerRosInterface
{
  public:
    /// \brief Constructor initializes planner, publishers, subscribers and
    ///   services.
    GlobalPlannerRosInterface();

    /// \brief Starts the ros loop with rate defined ros params.
    void run();

  private:
    // Instance of global planner
    shared_ptr<GlobalPlanner> global_planner_;
    // Instance of visualization and flag to track if visualization has changed
    Visualization visualization_;
    bool visualization_changed_;
    // Loop rate
    int rate_;

    // ROS stuff. First node handle for accessing topics and services.
    ros::NodeHandle nh_;

    // Publishers
    ros::Publisher multi_dof_trajectory_pub_, cartesian_path_pub_;

    // Empty service
    ros::ServiceServer empty_service_server_;
    bool emptyCallback(std_srvs::Empty::Request &req, 
      std_srvs::Empty::Response &res);

    // Cartesian trajectory service
    ros::ServiceServer cartesian_trajectory_server_;
    bool cartesianTrajectoryCallback(
      larics_motion_planning::CartesianTrajectory::Request &req, 
      larics_motion_planning::CartesianTrajectory::Response &res);

    // Joint trajectory service with n dimensions
    ros::ServiceServer multi_dof_trajectory_server_;
    bool multiDofTrajectoryCallback(
      larics_motion_planning::MultiDofTrajectory::Request &req, 
      larics_motion_planning::MultiDofTrajectory::Response &res);

    // Visualize state service
    ros::ServiceServer visualize_state_server_;
    bool visualizeStateCallback(
      larics_motion_planning::VisualizeState::Request &req, 
      larics_motion_planning::VisualizeState::Response &res);


    // Conversions between GlobalPlanner and ROS messages.
    Eigen::MatrixXd navMsgsPathToEigenPath(nav_msgs::Path nav_path);
    nav_msgs::Path eigenPathToNavMsgsPath(Eigen::MatrixXd eigen_path);
    trajectory_msgs::MultiDOFJointTrajectory trajectoryToMultiDofTrajectory(
      Trajectory eigen_trajectory);
    Eigen::MatrixXd jointTrajectoryToEigenWaypoints(
      trajectory_msgs::JointTrajectory joint_trajectory);
    trajectory_msgs::JointTrajectory eigenPathToJointTrajectory(
      Eigen::MatrixXd path);
    trajectory_msgs::JointTrajectory trajectoryToJointTrajectory(
      Trajectory eigen_trajectory);  
};

#endif // GLOBAL_PLANNER_ROS_INTERFACE_H