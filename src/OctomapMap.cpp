/// \file OctomapMap.cpp

#include "OctomapMap.h"

OctomapMap::OctomapMap(double resolution) :
map_(make_unique<octomap::OcTree>(resolution))
{
  //_octomap = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  //map_ = make_unique(<octomap::OcTree(0.1)>);
  cout << map_->getResolution() << endl;
}

OctomapMap::OctomapMap(string octomap_file, int depth) :
  map_(make_unique<octomap::OcTree>(octomap_file)),
  search_depth_(depth)
{
  cout << "Loading map from file " << octomap_file << endl;
}

OctomapMap::~OctomapMap()
{

}

bool OctomapMap::isStateValid(Eigen::VectorXd state)
{
  octomap::point3d query(state(0), state(1), state(2));
  octomap::OcTreeNode* result = map_->search(query, search_depth_);

  return !map_->isNodeOccupied(result);
}

bool OctomapMap::isStateValid(Eigen::VectorXd state, int depth)
{
  octomap::point3d query(state(0), state(1), state(2));
  octomap::OcTreeNode* result = map_->search(query, depth);

  return !map_->isNodeOccupied(result);
}

bool OctomapMap::configureFromFile(string octomap_file)
{
  cout << "Loading octomap from file: " << octomap_file << endl;
  map_ = make_unique<octomap::OcTree>(octomap_file);
  return true;
}

void OctomapMap::setDepth(int depth)
{
  search_depth_ = depth;
}

void OctomapMap::setOctomapFromRosMessage(const octomap_msgs::Octomap::ConstPtr& ros_octomap)
{
  //cout << "Map received" << endl;
  map_.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*ros_octomap)));
}