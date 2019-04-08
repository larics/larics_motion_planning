/// \file OctomapMap.cpp

#include "OctomapMap.h"

OctomapMap::OctomapMap(double resolution) :
map_(make_unique<octomap::OcTree>(resolution))
{
  //_octomap = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  //map_ = make_unique(<octomap::OcTree(0.1)>);
  cout << map_->getResolution() << endl;
}

OctomapMap::OctomapMap(string octomap_config_file)
{
  cout << "Loading map from file " << octomap_config_file << endl;
  configureFromFile(octomap_config_file);
}

OctomapMap::~OctomapMap()
{

}

bool OctomapMap::isStateValid(Eigen::VectorXd state)
{
  octomap::point3d query(state(0), state(1), state(2));
  octomap::OcTreeNode* result = map_->search(query, search_depth_);
  if(result == nullptr) return true;
  else return !map_->isNodeOccupied(result);
}

bool OctomapMap::isStateValid(Eigen::VectorXd state, int depth)
{
  octomap::point3d query(state(0), state(1), state(2));
  octomap::OcTreeNode* result = map_->search(query, depth);

  return !map_->isNodeOccupied(result);
}

bool OctomapMap::configureFromFile(string config_filename)
{
  cout << "Loading octomap from file: " << config_filename << endl;
  YAML::Node config = YAML::LoadFile(config_filename);
  map_ = make_unique<octomap::OcTree>(
    config["octomap"]["path_to_file"].as<string>());
  search_depth_ = config["octomap"]["search_depth"].as<int>();
  return true;
}

void OctomapMap::setDepth(int depth)
{
  search_depth_ = depth;
}

void OctomapMap::setOctomapFromRosMessage(const 
  octomap_msgs::Octomap::ConstPtr& ros_octomap)
{
  cout << "Map received" << endl;
  map_.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(
    *ros_octomap)));
}