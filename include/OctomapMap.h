/// \file OctomapMap.h
/// \brief Provides implementation of octomap search and transform functions.

#ifndef OCTOMAP_MAP_H
#define OCTOMAP_MAP_H

#include "MotionPlanningDatatypes.h"
#include "MapInterface.h"

#include <iostream>
#include <memory>
#include <cstdlib>
#include <string>

#include <eigen3/Eigen/Eigen>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/math/Vector3.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

using namespace std;

/// \brief Provides implementation of octomap search and transform functions.
/// This class uses Octomap for searching valid points in space. It supports
/// loading the map from .binvox.bt file and changing the map through ROS topic
class OctomapMap : public MapInterface
{
  public:
    OctomapMap(double resolution);
    OctomapMap(string octomap_file, int depth);
    ~OctomapMap();
    bool isStateValid(Eigen::VectorXd state);
    bool isStateValid(Eigen::VectorXd state, int depth);
    bool configureFromFile(string config_file);
    void setDepth(int depth);
    void setOctomapFromRosMessage(const octomap_msgs::Octomap::ConstPtr& ros_octomap);

  private:
    unique_ptr<octomap::OcTree> map_;
    int search_depth_;
};

#endif // OCTOMAP_MAP_H