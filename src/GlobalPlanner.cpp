#include <GlobalPlanner.h>

GlobalPlanner::GlobalPlanner(string config_filename)
{
  cout << "Initializing global planner from file: " << endl;
  cout << "  " << config_filename << endl;

  configureFromFile(config_filename);
}

bool GlobalPlanner::configureFromFile(string config_filename)
{
  // Set up map, trajectory and path planner.
  map_ = make_shared<OctomapMap>(
    "/home/antun/catkin_ws/src/larics_motion_planning/config/octomap_config_example.yaml");
  trajectory_ = make_shared<ToppraTrajectory>(
    "/home/antun/catkin_ws/src/larics_motion_planning/config/toppra_config_example.yaml");
  path_planner_ = make_shared<RrtPathPlanner>(
    "/home/antun/catkin_ws/src/larics_motion_planning/config/path_planner_config_example.yaml", 
    map_);

  return true;
}

bool GlobalPlanner::collisionCheck(Eigen::MatrixXd path)
{

}

bool GlobalPlanner::planPath(Eigen::MatrixXd waypoints)
{

}

bool GlobalPlanner::planTrajectory(Eigen::MatrixXd waypoints)
{
  
}