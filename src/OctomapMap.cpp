/// \file OctomapMap.cpp

#include <larics_motion_planning/OctomapMap.h>
#include <larics_motion_planning/MotionPlanningUtil.h>

OctomapMap::OctomapMap(double resolution) :
map_(make_unique<octomap::OcTree>(resolution))
{
  //_octomap = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  //map_ = make_unique(<octomap::OcTree(0.1)>);
  cout << map_->getResolution() << endl;
}

OctomapMap::OctomapMap(string octomap_config_file)
{
  // Configures the map from yaml file.
  configureFromFile(motion_util::getUserPrefix() + octomap_config_file);
}

OctomapMap::~OctomapMap()
{

}

bool OctomapMap::isStateValid(Eigen::VectorXd state)
{
  // Create octomap point from vector.
  octomap::point3d query(state(0), state(1), state(2));
  // Search the map
  octomap::OcTreeNode* result = map_->search(query, search_depth_);

  // If result was null pointer the point is not in the map. We consider such
  // points to be free. If the point is in map, search it and return result.
  if(result == nullptr) return true;
  else return !map_->isNodeOccupied(result);
}

bool OctomapMap::isStateValid(Eigen::VectorXd state, int depth)
{
  // Create octomap point from vector.
  octomap::point3d query(state(0), state(1), state(2));
  // Operates with provided search depth.
  octomap::OcTreeNode* result = map_->search(query, depth);

  return !map_->isNodeOccupied(result);
}

bool OctomapMap::configureFromFile(string config_filename)
{
  cout << "Loading octomap from file: " << endl;
  cout << "  " << config_filename << endl;
  YAML::Node config = YAML::LoadFile(config_filename);

  // Load map from path provided in file.
  map_ = make_unique<octomap::OcTree>(
    motion_util::getUserPrefix() +
    motion_util::loadPathOrThrow(
      [&](){ return config["octomap"]["path_to_file"].as<string>(); },
      "OCTOMAP_FILE",
      "octomap/path_to_file"
      )
    );
  // Set search depth from file.
  search_depth_ = config["octomap"]["search_depth"].as<int>();
  return true;
}

void OctomapMap::setDepth(int depth)
{
  // Simply setting the search depth.
  search_depth_ = depth;
}

void OctomapMap::setOctomapFromRosMessage(const
  octomap_msgs::Octomap::ConstPtr& ros_octomap)
{
  cout << "Octomap received" << endl;
  map_.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(
    *ros_octomap)));
  cout << "  Memory usage: " << map_->memoryUsage()/1000000.0 << " MB" << endl;
  double x, y, z;
  map_->getMetricSize(x, y, z);
  cout << "  Map bounds: x=" << x << " y=" << y << " z=" << z << endl;
  cout << "  Map resolution: " << map_->getResolution() << endl;
}

bool OctomapMap::saveOctomap(string path)
{
  if (map_ == nullptr){
    return false;
  }

  map_->writeBinary(path);

  return true;
}