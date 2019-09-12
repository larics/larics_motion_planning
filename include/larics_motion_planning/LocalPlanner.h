/// \file Local.h
/// \brief Provides implementation for local planner that executes trajectory
///   and locally searches for obstacles and compensates manipulator pose


#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

// TODO: remove ros from local planner? Consider this, use it now though.
#include <ros/ros.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <larics_motion_planning/MapInterface.h>
#include <larics_motion_planning/OctomapMap.h>
#include <larics_motion_planning/KinematicsInterface.h>
#include <larics_motion_planning/WpManipulatorKinematics.h>
#include <larics_motion_planning/PathPlanningInterface.h>
#include <larics_motion_planning/RrtPathPlanner.h>
#include <larics_motion_planning/TrajectoryInterface.h>
#include <larics_motion_planning/ToppraTrajectory.h>
#include <larics_motion_planning/StateValidityCheckerInterface.h>
#include <larics_motion_planning/PointStateValidityChecker.h>
#include <larics_motion_planning/UavWpManipulatorStateValidityChecker.h>

#include <eigen3/Eigen/Eigen>
#include <cmath>

#include "yaml-cpp/yaml.h"

#include <iostream>
using namespace std;

/// \brief This class implements local planner strategies and ideas.
class LocalPlanner
{
  public:
    /// \brief Constructor initializes local planner.
    /// \param config_filename Path to configuration filename.
    LocalPlanner(string config_filename);

    /// \brief Configures local planner from file.
    /// \param config_filename Path to configuration file.
    /// \return True if configuration was successful, false otherwise.
    bool configureFromFile(string config_filename);

    void run();

  private:
    shared_ptr<MapInterface> map_interface_;
    shared_ptr<StateValidityCheckerInterface> state_validity_checker_interface_;
    shared_ptr<PathPlanningInterface> path_planner_interface_;
    shared_ptr<KinematicsInterface> kinematics_interface_;

    string state_validity_checker_type_;

    // Node handle
    ros::NodeHandle nh_;
    ros::Publisher multi_dof_trajectory_pub_, manipulator_joint_trajectory_pub_, 
      multi_dof_trajectory_point_pub_;
    void jointTrajectoryCallback(const trajectory_msgs::JointTrajectory &msg);
    void odometryCallback(const nav_msgs::Odometry &msg);

    geometry_msgs::Pose uav_current_pose_;

};


#endif // LOCAL_PLANNER_H