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

bool OctomapMap::configureFromFile(string config_file)
{
  cout << "Loading octomap from file " << config_file << endl;
  map_ = make_unique<octomap::OcTree>(config_file);
}

void OctomapMap::setDepth(int depth)
{
  search_depth_ = depth;
}

void OctomapMap::setOctomapFromRosMessage(const octomap_msgs::Octomap::ConstPtr& ros_octomap)
{
  //auto temp = (unique_ptr<octomap::AbstractOcTree>)octomap_msgs::binaryMsgToMap(*ros_octomap);
  //auto temp2 = make_unique<octomap::OcTree>(temp);
  map_.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*ros_octomap)));
  //octomap::OcTree* temp = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*ros_octomap);
  //auto temp2 = unique_ptr<octomap::OcTree>(temp);
  //cout << typeid(ros_octomap).name() << endl;
  //cout << typeid(temp).name() << endl;
  //cout << typeid(map_).name() << endl;
  //cout << typeid(temp2).name() << endl;
  //map_ = make_unique<octomap::OcTree>(octomap_msgs::binaryMsgToMap(ros_octomap));
}