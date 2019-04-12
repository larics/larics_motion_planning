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
  map_interface_ = make_shared<OctomapMap>(
    "/home/antun/catkin_ws/src/larics_motion_planning/config/octomap_config_example.yaml");
  trajectory_interface_ = make_shared<ToppraTrajectory>(
    "/home/antun/catkin_ws/src/larics_motion_planning/config/toppra_config_example.yaml");
  path_planner_interface_ = make_shared<RrtPathPlanner>(
    "/home/antun/catkin_ws/src/larics_motion_planning/config/path_planner_config_example.yaml", 
    map_interface_);

  return true;
}

bool GlobalPlanner::collisionCheck(Eigen::MatrixXd path)
{
  return true;
}

bool GlobalPlanner::planPath(Eigen::MatrixXd waypoints)
{
  return this->planPathThroughTwoWaypoints(waypoints);
}

bool GlobalPlanner::planTrajectory(Eigen::MatrixXd waypoints)
{
  bool success = false;
  int num_trajectory_restarts = 5;
  for (int i=0; i<num_trajectory_restarts && success==false; i++){
    success = trajectory_interface_->generateTrajectory(waypoints);
  }
  return success;
}

bool GlobalPlanner::planPathAndTrajectory(Eigen::MatrixXd waypoints)
{
  bool success = false;

  int num_path_and_trajectory_restarts = 5;

  for (int i=0; i<num_path_and_trajectory_restarts && success==false; i++){
    // First obtain collision free path.
    success = this->planPath(waypoints);
    // Next plan trajectory based on the path. 
    // TODO: swap getPath with path_ when planPath is fully implemented.
    success &= this->planTrajectory(path_planner_interface_->getPath());
    success &= this->collisionCheck(trajectory_interface_->getTrajectory().position);
  }

  return success;
}

Eigen::MatrixXd GlobalPlanner::getPath()
{
  // TODO: swap getPath() with path_ when implemented.
  return path_planner_interface_->getPath();
}

Trajectory GlobalPlanner::getTrajectory()
{
  return trajectory_interface_->getTrajectory();
}

bool GlobalPlanner::planPathThroughTwoWaypoints(Eigen::MatrixXd waypoints)
{
  if (waypoints.rows()<2){
    cout << "planPathThroughTwoWaypoints: " << endl;
    cout << "  Less than two waypoints provided. Unable to plan path." << endl;
    return false;
  }
  
  bool success = false, plan_path_collision_check = true;
  // Planner might not start properly so give it a few attampts. This does not
  // happen a lot in practice, but just in case.
  int num_restarts = 5;
  // If collision check fails how many times should planner restart.
  int num_collision_check_restarts = 5;

  // Outer loop checks for collision and restarts if path is not collision
  // free. Inner loop restarts if planner configuration was unsuccessful
  // or something unexpected happend.
  for (int i=0; i<num_collision_check_restarts && success==false && 
    (plan_path_collision_check==true || i==0); i++){
    for (int j=0; j<num_restarts && success==false; j++){
      success = path_planner_interface_->planPath(waypoints);
    }
    if (plan_path_collision_check==true){
      success &= this->collisionCheck(path_planner_interface_->getPath());
    }
  }
  
  return success;
}