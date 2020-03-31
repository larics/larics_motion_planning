#include <larics_motion_planning/GlobalPlanner.h>

GlobalPlanner::GlobalPlanner(string config_filename)
{
  string username = "/home/";
  username = username + getenv("USER") + "/";
  size_t found = config_filename.find(username);
  if (found != string::npos) configureFromFile(config_filename);
  else configureFromFile(username + config_filename);
}

bool GlobalPlanner::configureFromFile(string config_filename)
{
  cout << "Configuring global planner from file: " << endl;
  cout << "  " << config_filename << endl;
  // Open yaml file with configuration
  YAML::Node config = YAML::LoadFile(config_filename);
  config_filename_ = config_filename;

  // Set up map interface
  map_interface_ = make_shared<OctomapMap>(
    config["global_planner"]["map_config_file"].as<string>());
  // Set up trajectory planning interface
  trajectory_interface_ = make_shared<ToppraTrajectory>(
    config["global_planner"]["trajectory_config_file"].as<string>());
  // Set up state validity interface
  string username = "/home/";
  username = username + getenv("USER") + "/";
  YAML::Node state_config = YAML::LoadFile(username + config["global_planner"]["state_validity_checker_config_file"].as<string>());
  state_validity_checker_type_ = state_config["state_validity_checker"]["type"].as<string>();
  //if (state_validity_checker_type_ == "point"){
  //  state_validity_checker_interface_ = make_shared<PointStateValidityChecker>(
  //    map_interface_);
  //  cout << "State validity checker type is: point" << endl;
  //}
  if (state_validity_checker_type_ == "ball" ||
    state_validity_checker_type_ == "sphere" ||
    state_validity_checker_type_ == "point" ||
    state_validity_checker_type_ == "circle" ||
    state_validity_checker_type_ == "cylinder" ||
    state_validity_checker_type_ == "rectangle" ||
    state_validity_checker_type_ == "prism"){
    state_validity_checker_interface_ = make_shared<SimpleStateValidityCheckers>(
      config["global_planner"]["state_validity_checker_config_file"].as<string>(),
      map_interface_, state_validity_checker_type_);
    cout << "State validity checker type is: " << state_validity_checker_type_ << endl;
  }
  else if (state_validity_checker_type_ == "uav_and_wp_manipulator"){
    // First set up kinematics for wp manipulator.
    kinematics_interface_ = make_shared<WpManipulatorKinematics>(
      config["global_planner"]["kinematics_config_file"].as<string>());
    // Set up validity checker for uav and wp manipulator
    state_validity_checker_interface_ = make_shared<UavWpManipulatorStateValidityChecker>(
      config["global_planner"]["state_validity_checker_config_file"].as<string>(),
      map_interface_, kinematics_interface_);
    cout << "State validity checker type is: uav_and_wp_manipulator" << endl;
  }
  else{
    cout << "State validity checker type is: " << state_validity_checker_type_ << endl;
    cout << "  This type is not supported!" << endl;
    exit(0);
  }
  // Set up path planning interface
  path_planner_interface_ = make_shared<RrtPathPlanner>(
    config["global_planner"]["path_planner_config_file"].as<string>(),
    state_validity_checker_interface_);
  num_trajectory_restarts_ =
    config["global_planner"]["trajectory"]["restarts"].as<int>();
  num_path_and_trajectory_restarts_ =
    config["global_planner"]["path_and_trajectory"]["restarts"].as<int>();

  // Planner might not start properly so give it a few attampts. This does not
  // happen a lot in practice, but just in case.
  num_path_restarts_ = config["global_planner"]["path"]["restarts"].as<int>();
  // If collision check fails how many times should planner restart.
  num_collision_check_restarts_ =
    config["global_planner"]["path"]["collision_check_restarts"].as<int>();

  plan_path_collision_check_ =
    config["global_planner"]["path"]["collision_check"].as<bool>();

  return true;
}

bool GlobalPlanner::collisionCheck(Eigen::MatrixXd path)
{
  bool success = true;

  // Go through all points along path and check is state valid on map. This
  // also works for trajectory.positions
  for (int i=0; i<path.rows(); i++){
    success &= state_validity_checker_interface_->isStateValid((path.row(i)).transpose());
  }

  return success;
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

  for (int i=0; i<num_trajectory_restarts_ && success==false; i++){
    success = trajectory_interface_->generateTrajectory(waypoints);
  }
  return success;
}

bool GlobalPlanner::planPathAndTrajectory(Eigen::MatrixXd waypoints)
{
  bool success = false;

  for (int i=0; i<num_path_and_trajectory_restarts_ && success==false; i++){
    // First obtain collision free path.
    success = this->planPath(waypoints);
    if (success == false) cout << "Planning waypoints failed!" << endl;
    // Next plan trajectory based on the path.
    success &= this->planTrajectory(path_);
    if (success == false) cout << "Planning trajectory failed!" << endl;
    // Depending on the state validity checker type, check the collisions.
    if (state_validity_checker_type_ == "point"){
      success &= this->collisionCheck(trajectory_interface_->getTrajectory().position);
    }
    else if (state_validity_checker_type_ == "uav_and_wp_manipulator"){
      Trajectory trajectory = trajectory_interface_->getTrajectory();
      for (int j=0; j<trajectory.position.rows(); j++){
        // We don't plan for roll and pitch. However we can estimate them from
        // accelerations. Add roll and pitch to the trajectory and check if it
        // is valid when the uav exerts angles.
        trajectory.position(j, 3) = -trajectory.acceleration(j, 1)/9.81;
        trajectory.position(j, 4) = trajectory.acceleration(j, 0)/9.81;
      }
      // TODO: Collision check the trajectory.
      //success &= this->collisionCheck(trajectory.position);
      if (success == false) cout << "Trajectory collision check failed!" << endl;
    }
  }

  return success;
}

Eigen::MatrixXd GlobalPlanner::getPath()
{
  return path_;
}

double GlobalPlanner::getPathLength()
{
  return path_planner_interface_->getPathLength();
}

Trajectory GlobalPlanner::getTrajectory()
{
  return trajectory_interface_->getTrajectory();
}

Eigen::MatrixXd GlobalPlanner::getRobotStatePoints(Eigen::VectorXd state)
{
  return state_validity_checker_interface_->generateValidityPoints(state);
}

shared_ptr<KinematicsInterface> GlobalPlanner::getKinematicsInterface()
{
  return kinematics_interface_;
}

shared_ptr<MapInterface> GlobalPlanner::getMapInterface()
{
  return map_interface_;
}

shared_ptr<TrajectoryInterface> GlobalPlanner::getTrajectoryInterface()
{
  return trajectory_interface_;
}

bool GlobalPlanner::planPathThroughTwoWaypoints(Eigen::MatrixXd waypoints)
{
  if (waypoints.rows()<2){
    cout << "planPathThroughTwoWaypoints: " << endl;
    cout << "  Less than two waypoints provided. Unable to plan path." << endl;
    return false;
  }

  bool success = false;

  // Outer loop checks for collision and restarts if path is not collision
  // free. Inner loop restarts if planner configuration was unsuccessful
  // or something unexpected happend.
  for (int i=0; i<num_collision_check_restarts_ && success==false &&
    (plan_path_collision_check_==true || i==0); i++){
    for (int j=0; j<num_path_restarts_ && success==false; j++){
      success = path_planner_interface_->planPath(waypoints);
    }
    if (plan_path_collision_check_==true){
      success &= this->collisionCheck(path_);
    }
  }

  return success;
}
