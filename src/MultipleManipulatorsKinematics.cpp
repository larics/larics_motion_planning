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

  n_manipulators_ = config["kinematics"]["multiple_manipulators"].size();
  for (int i=0; i<n_manipulators_; i++){
    if (config["kinematics"]["multiple_manipulators"][i]["type"].as<string>() == "wp_manipulator"){
      string robot_model_name, joint_group_name, dh_parameters_file;
      robot_model_name = config["kinematics"]["multiple_manipulators"][i]["robot_model_name"].as<string>();
      joint_group_name = config["kinematics"]["multiple_manipulators"][i]["joint_group_name"].as<string>();
      dh_parameters_file = config["kinematics"]["multiple_manipulators"][i]["dh_parameters_file"].as<string>();
      // TODO: This is built by cpp, but... I am unsure if it will cause
      // problems because a shared pointer is created locally. It might
      // work but if some weird issues come, using shared pointers in array
      // and pushing them with locally created objects might be the culprit.
      // Definitely check this!
      shared_ptr<KinematicsInterface> man;
      man = make_shared<WpManipulatorKinematics>(robot_model_name,
        joint_group_name, dh_parameters_file);
      manipulators_.push_back(man);
    }
    else {
      cout << "MultipleManipulatorsKinematics" << endl;
      cout << "  No such kinematics interface: " << 
        config["kinematics"]["multiple_manipulators"][i]["type"].as<string>() << endl;
      exit(0);
    }
  }
  exit(0);
  
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
  return manipulators_[0]->getJointPositions(q);
}

Eigen::Affine3d MultipleManipulatorsKinematics::getEndEffectorTransform(
  Eigen::VectorXd q)
{
  return manipulators_[0]->getEndEffectorTransform(q);
}

Eigen::VectorXd MultipleManipulatorsKinematics::calculateInverseKinematics(
  Eigen::Affine3d transform, bool &found_ik)
{
  return manipulators_[0]->calculateInverseKinematics(transform, found_ik);
}
