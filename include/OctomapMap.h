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

using namespace std;

/// This class handles 
class OctomapMap : public MapInterface
{
  public:
    OctomapMap(string config_filename);
    ~OctomapMap();
    bool isStateValid(Eigen::VectorXd state);

  private:
    unique_ptr<octomap::OcTree> map_;
};

#endif // OCTOMAP_MAP_H