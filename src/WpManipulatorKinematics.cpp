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
  robot_model_name_ = robot_model_name;
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
  robot_model_name_ = robot_model_name;
  joint_group_name = config["kinematics"]["wp_manipulator_kinematics"]["joint_group_name"].as<string>();
  if (getenv("ABSOLUTE_CONFIG")){
    dh_parameters_file = motion_util::loadPathOrThrow(
      [&](){ return config["kinematics"]["wp_manipulator_kinematics"]["dh_parameters_file"].as<string>(); }, 
      "DH_PARAMETERS_FILE",
      "wp_manipulator_kinematics/kinematics_config_file");
  }
  else{
    string username = "/home/";
    username = username + getenv("USER") + "/";
    dh_parameters_file = username + config["kinematics"]["wp_manipulator_kinematics"]["dh_parameters_file"].as<string>();
  }

  cout << "WpManipulatorKinematics.cpp" << endl;
  cout << "  DH params file: " << dh_parameters_file << endl;
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

Eigen::VectorXd WpManipulatorKinematics::calculateOptimalManipulatorState(
  Eigen::Affine3d T_w_t)
{
  Eigen::Vector3d rpy = T_w_t.rotation().eulerAngles(2, 1, 0);

  Eigen::VectorXd q;
  // These ifs are non configurable for now.
  if (robot_model_name_ == "wp_manipulator"){
    q = Eigen::VectorXd::Zero(5);
    q << M_PI/4 + rpy(1)/2, M_PI/4 + rpy(1)/2, 0.557, -0.861, 0.304;
  }
  else{
    cout << "[WpManipulatorKinematics] No such manipulator robot model." << endl;
    cout << "  Your provided robot model is: " << robot_model_name_ << endl;
  }

  // If wrong manipulator robot model is provided, return empty q. The caller should
  // check the size of q.
  return q;
}