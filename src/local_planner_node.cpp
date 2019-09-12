#include <larics_motion_planning/LocalPlanner.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_planner_node");

  LocalPlanner lp("catkin_ws/src/larics_motion_planning/config/uav_and_wp_manipulator_3r_config.yaml");
  lp.run();

  return 0;
}