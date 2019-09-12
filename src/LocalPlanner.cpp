#include <larics_motion_planning/LocalPlanner.h>

LocalPlanner::LocalPlanner(string config_filename)
{
  string username = "/home/";
  username = username + getenv("USERNAME") + "/";
  configureFromFile(username + config_filename);

  nh_.subscribe("joint_trajectory", 1, &LocalPlanner::jointTrajectoryCallback, this);
}

bool LocalPlanner::configureFromFile(string config_filename)
{
  cout << "Configuring local planner from file: " << endl;
  cout << "  " << config_filename << endl;
  // Open yaml file with configuration
  YAML::Node config = YAML::LoadFile(config_filename);

  // Set up map interface
  map_interface_ = make_shared<OctomapMap>(
    config["local_planner"]["map_config_file"].as<string>());
  // Set up state validity interface
  string username = "/home/";
  username = username + getenv("USERNAME") + "/";
  YAML::Node state_config = YAML::LoadFile(username + config["local_planner"]["state_validity_checker_config_file"].as<string>());
  state_validity_checker_type_ = state_config["state_validity_checker"]["type"].as<string>();
  if (state_validity_checker_type_ == "point"){
    state_validity_checker_interface_ = make_shared<PointStateValidityChecker>(
      map_interface_);
    cout << "State validity checker type is: point" << endl;
  }
  else if (state_validity_checker_type_ == "uav_and_wp_manipulator"){
    // First set up kinematics for wp manipulator.
    kinematics_interface_ = make_shared<WpManipulatorKinematics>(
      config["local_planner"]["kinematics_config_file"].as<string>());
    // Set up validity checker for uav and wp manipulator
    state_validity_checker_interface_ = make_shared<UavWpManipulatorStateValidityChecker>(
      config["local_planner"]["state_validity_checker_config_file"].as<string>(), 
      map_interface_, kinematics_interface_);
    cout << "State validity checker type is: uav_and_wp_manipulator" << endl;
  }
  else{
    cout << "State validity checker type is: " << state_validity_checker_type_ << endl;
    cout << "  This type is not supported!" << endl;
    exit(0);
  }

  cout << "Local planner configured!" << endl;

  return true;
}

void LocalPlanner::run()
{
  ros::spin();
}

void LocalPlanner::jointTrajectoryCallback(
  const trajectory_msgs::JointTrajectory &msg) 
{
  // Dequeue trajectory and compensate for manipulator movement
}