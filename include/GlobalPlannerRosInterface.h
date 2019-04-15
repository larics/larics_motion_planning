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

#include <iostream>

using namespace std;

class GlobalPlannerRosInterface
{
  public:
    GlobalPlannerRosInterface();

    void run();

  private:
    ros::NodeHandle nh_;
};

#endif // GLOBAL_PLANNER_ROS_INTERFACE_H