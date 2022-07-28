#include <larics_motion_planning/WpManipulatorKinematics.h>
#include <larics_motion_planning/MotionPlanningUtil.h>

WpManipulatorKinematics::WpManipulatorKinematics(string config_filename)
{
  if (getenv("ABSOLUTE_CONFIG")){
    configureFromFile(config_filename);
  }
  else{
    string username = "/home/";
    username = username + getenv("USER") + "/";
    configureFromFile(username + config_filename);
  }
}

WpManipulatorKinematics::WpManipulatorKinematics(string robot_model_name, 
  string joint_group_name, string dh_parameters_file)
{
  string username = "/home/";
  username = username + getenv("USER") + "/";
  string params_file = username + dh_parameters_file;

  // Configure manipulator
  manipulator_.setManipulatorName(robot_model_name, joint_group_name);
  manipulator_.LoadParameters(params_file);
  manipulator_.init();
}

bool WpManipulatorKinematics::configureFromFile(string config_filename)
{
  cout << "Configuring WP manipulator kinematics from file: " << endl;
  cout << "  " << config_filename << endl;

  YAML::Node config = YAML::LoadFile(config_filename);

  // Load manipulator configuration
  string robot_model_name, joint_group_name, dh_parameters_file;
  robot_model_name = config["kinematics"]["wp_manipulator_kinematics"]["robot_model_name"].as<string>();
  joint_group_name = config["kinematics"]["wp_manipulator_kinematics"]["joint_group_name"].as<string>();
  string username = "/home/";
  username = username + getenv("USER") + "/";
  //dh_parameters_file = username + config["kinematics"]["wp_manipulator_kinematics"]["dh_parameters_file"].as<string>();
  dh_parameters_file = motion_util::loadPathOrThrow(
      [&](){ return config["kinematics"]["wp_manipulator_kinematics"]["dh_parameters_file"].as<string>(); }, 
      "DH_PARAMETERS_FILE",
      "wp_manipulator_kinematics/kinematics_config_file");
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

void WpManipulatorKinematics::setJointPositions(
  Eigen::VectorXd joint_positions)
{
  manipulator_.setJointPositions(joint_positions);
}

Eigen::MatrixXd WpManipulatorKinematics::getJacobian(Eigen::VectorXd q)
{
  return manipulator_.getJacobian(q);
}