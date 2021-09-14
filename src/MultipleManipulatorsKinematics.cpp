#include <larics_motion_planning/MultipleManipulatorsKinematics.h>

MultipleManipulatorsKinematics::MultipleManipulatorsKinematics(string config_filename)
{
  string username = "/home/";
  username = username + getenv("USER") + "/";
  configureFromFile(username + config_filename);
}

bool MultipleManipulatorsKinematics::configureFromFile(string config_filename)
{
  cout << "Configuring multiple manipulators kinematics from file: " << endl;
  cout << "  " << config_filename << endl;

  YAML::Node config = YAML::LoadFile(config_filename);
  // TODO: Napraviti ovo tako da se zadaje jedan transform za inverznu i u
  // config fileu imaju transformacije od te tocke do prihvata za svaki alat.
  // To ce olaksati racunanje inverzne kinematike kada netko posalje tocku
  // izvana
  
  // Load manipulator configuration
  /*string robot_model_name, joint_group_name, dh_parameters_file;
  robot_model_name = config["kinematics"]["wp_manipulator_kinematics"]["robot_model_name"].as<string>();
  joint_group_name = config["kinematics"]["wp_manipulator_kinematics"]["joint_group_name"].as<string>();
  string username = "/home/";
  username = username + getenv("USER") + "/";
  dh_parameters_file = username + config["kinematics"]["wp_manipulator_kinematics"]["dh_parameters_file"].as<string>();
  // Configure manipulator
  manipulator_.setManipulatorName(robot_model_name, joint_group_name);
  manipulator_.LoadParameters(dh_parameters_file);
  manipulator_.init();
  */
}

std::vector<Eigen::Affine3d> MultipleManipulatorsKinematics::getJointPositions(
  Eigen::VectorXd q)
{
  return manipulators_[0].getJointPositions(q);
}

Eigen::Affine3d MultipleManipulatorsKinematics::getEndEffectorTransform(
  Eigen::VectorXd q)
{
  return manipulators_[0].getEndEffectorTransform(q);
}

Eigen::VectorXd MultipleManipulatorsKinematics::calculateInverseKinematics(
  Eigen::Affine3d transform, bool &found_ik)
{
  return manipulators_[0].calculateInverseKinematics(transform, found_ik);
}
