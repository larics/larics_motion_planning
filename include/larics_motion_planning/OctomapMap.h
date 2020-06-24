/// \file OctomapMap.h
/// \brief Provides implementation of octomap search and transform functions.

#ifndef OCTOMAP_MAP_H
#define OCTOMAP_MAP_H

#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <larics_motion_planning/MapInterface.h>

#include <iostream>
#include <memory>
#include <cstdlib>
#include <string>

#include <eigen3/Eigen/Eigen>

#include "yaml-cpp/yaml.h"

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
    /// \brief Constructs the empty map.
    /// \param resolution Resolution of the octomap. This will probably be
    ///   overwritten by loading the octomap. Suitable for using the map
    ///   through topic.
    OctomapMap(double resolution);

    /// \brief Configures the planner from .yaml file.
    /// \param octomap_file Path to .yaml file where configuration is stored.
    OctomapMap(string octomap_config_file);

    /// \brief Destructor
    ~OctomapMap();

    /// \brief Checks if point in octomap is occupied or free.
    /// \param state This is vector with (x,y,z) coordinates of query point. 
    /// \return True if point is free, false if point is occupied.
    bool isStateValid(Eigen::VectorXd state);

    /// \brief Checks if point in octomap is occupied or free.
    /// \param state This is vector with (x,y,z) coordinates of query point. 
    /// \param depth Search depth of the octomap. This will only be used with
    ///   a single query.
    /// \return True if point is free, false if point is occupied.
    bool isStateValid(Eigen::VectorXd state, int depth);

    /// \brief Loads octomap from file and uses it for querying validity.
    /// \param octomap_file Path to .binvox.bt file containing octomap.
    /// \return True if map loading was successful.
    bool configureFromFile(string config_filename);

    /// \brief Sets search depth for octomap.
    /// \param depth Depth for search, must be in interval [1,16]
    void setDepth(int depth);

    /// \brief Converts ROS message Octomap to octomap::OcTree.
    ///
    /// This function can be used as a callback to octomap topic or as a
    /// converter.
    /// \param ros_octomap octomap_msgs::Octomap message.
    void setOctomapFromRosMessage(const octomap_msgs::Octomap::ConstPtr& ros_octomap);

    /// \brief Saves octomap binary to file.
    /// \param path Path to file.
    /// \return True if save was successful, false if nullptr.
    bool saveOctomap(string path);

  private:
    unique_ptr<octomap::OcTree> map_;
    int search_depth_;
};

#endif // OCTOMAP_MAP_H