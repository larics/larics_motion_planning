#include <larics_motion_planning/WpManipulatorKinematics.h>

WpManipulatorKinematics::WpManipulatorKinematics(string config_filename)
{
  string username = "/home/";
  username = username + getenv("USERNAME") + "/";
  configureFromFile(username + config_filename);
}

bool WpManipulatorKinematics::configureFromFile(string config_filename)
{
  cout << "Configuring WP manipulator kinematics from file: " << endl;
  cout << "  " << config_filename << endl;

  YAML::Node config = YAML::LoadFile(config_filename);

  // Load manipulator configuration
  string robot_model_name, joint_group_name, dh_parameters_file;
  robot_model_name = config["wp_manipulator_kinematics"]["robot_model_name"].as<string>();
  joint_group_name = config["wp_manipulator_kinematics"]["joint_group_name"].as<string>();
  dh_parameters_file = config["wp_manipulator_kinematics"]["dh_parameters_file"].as<string>();
  // Configure manipulator
  manipulator_.setManipulatorName(robot_model_name, joint_group_name);
  manipulator_.LoadParameters(dh_parameters_file);
  manipulator_.init();
}

std::vector<Eigen::Affine3d> WpManipulatorKinematics::getJointPositions(
  Eigen::VectorXd q)
{
  return manipulator_.getLinkPositions(q); 
}

Eigen::Affine3d WpManipulatorKinematics::getEndEffectorTransform(
  Eigen::VectorXd q)
{
  return manipulator_.getEndEffectorTransform(q);
}

Eigen::VectorXd WpManipulatorKinematics::calculateInverseKinematics(
  Eigen::Affine3d transform, bool &found_ik)
{
  return manipulator_.calculateJointSetpoints(transform, found_ik);
}