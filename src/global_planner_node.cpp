#include <GlobalPlannerRosInterface.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <topp_ros/GenerateTrajectory.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_planner_node");

  GlobalPlannerRosInterface gp("/home/antun/catkin_ws/src/larics_motion_planning/config/one_file_config_example.yaml");
  gp.run();

  return 0;
}