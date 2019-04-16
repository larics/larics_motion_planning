/// \file GlobalPlannerRosInterface.h
/// \brief ROS interface with services and topics for global planner.

#ifndef GLOBAL_PLANNER_ROS_INTERFACE_H
#define GLOBAL_PLANNER_ROS_INTERFACE_H

#include <MotionPlanningDatatypes.h>
#include <MapInterface.h>
#include <OctomapMap.h>
#include <PathPlanningInterface.h>
#include <RrtPathPlanner.h>
#include <TrajectoryInterface.h>
#include <ToppraTrajectory.h>
#include <GlobalPlanner.h>

#include <eigen3/Eigen/Eigen>

#include "yaml-cpp/yaml.h"

#include <ros/ros.h>

// Messages
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
// Services
#include <std_srvs/Empty.h>
#include <larics_motion_planning/CartesianTrajectory.h>

#include <iostream>
using namespace std;

class GlobalPlannerRosInterface
{
  public:
    GlobalPlannerRosInterface(string s);

    void run();

  private:
    ros::NodeHandle nh_;
    shared_ptr<GlobalPlanner> global_planner_;

    ros::Publisher multi_dof_trajectory_pub_, cartesian_path_pub_;

    ros::ServiceServer empty_service_server_;
    bool emptyCallback(std_srvs::Empty::Request &req, 
      std_srvs::Empty::Response &res);

    ros::ServiceServer cartesian_trajectory_server_;
    bool cartesianTrajectoryCallback(
      larics_motion_planning::CartesianTrajectory::Request &req, 
      larics_motion_planning::CartesianTrajectory::Response &res);

    Eigen::MatrixXd navMsgsPathToEigenMatrixXd(nav_msgs::Path nav_path);
    nav_msgs::Path eigenMatrixXdToNavMsgsPath(Eigen::MatrixXd eigen_path);
    trajectory_msgs::MultiDOFJointTrajectory trajectoryToMultiDofTrajectory(
      Trajectory eigen_trajectory);
};

#endif // GLOBAL_PLANNER_ROS_INTERFACE_H