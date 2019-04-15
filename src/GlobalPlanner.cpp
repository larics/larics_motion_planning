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
  bool success = true;
  path_.resize(0,0);

  for (int i=1; i<waypoints.rows(); i++){
    // Plan path through current two waypoints. Waypoints are taken out as a 
    // block of size (2, num_columns) which takes two waypoints from the list.
    success &= this->planPathThroughTwoWaypoints(
      waypoints.block(i-1, 0, 2, waypoints.cols()));

    // Remeber rows before resizing the path.
    int rows = path_.rows();
    // Resize the path so it can hold currently planned one.
    path_.conservativeResize(
      path_.rows()+path_planner_interface_->getPath().rows(), waypoints.cols());
    // Insert currently planned path into path_. It is used as a block of size 
    // of the new path and starts at rows where previous path has ended.
    path_.block(rows, 0, path_planner_interface_->getPath().rows(), 
      path_planner_interface_->getPath().cols()) << 
      path_planner_interface_->getPath();

    // Finally, cut the last row because if we have multiple waypoints the last
    // point on current path will be the first point on next path. That's why 
    // we simply remove that point, unless we are at the last waypoint.
    if (i != waypoints.rows()-1){
      path_.conservativeResize(path_.rows()-1, path_.cols());
    }
  }

  return success;
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
    success &= this->planTrajectory(path_);
    success &= this->collisionCheck(trajectory_interface_->getTrajectory().position);
  }

  return success;
}

Eigen::MatrixXd GlobalPlanner::getPath()
{
  return path_;
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
      success &= this->collisionCheck(path_);
    }
  }
  
  return success;
}