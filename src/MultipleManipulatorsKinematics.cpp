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

      // Load degrees of freedom and indexes.
      int temp_dof = config["kinematics"]["multiple_manipulators"][i]["degrees_of_freedom"].as<int>();
      std::vector<int> temp_indexes = config["kinematics"]["multiple_manipulators"][i]["indexes"].as< std::vector<int> >();
      if (temp_dof != temp_indexes.size()){
        cout << "MultipleManipulatorsKinematics" << endl;
        cout << "Manipulator config " << i << endl;
        cout << "  Number of dofs is different from number of indexes." << endl;
        exit(0);
      }

      // Load grasp transforms.
      double x = config["kinematics"]["multiple_manipulators"][i]["grasp_transform"]["translation"][0].as<double>();
      double y = config["kinematics"]["multiple_manipulators"][i]["grasp_transform"]["translation"][1].as<double>();
      double z = config["kinematics"]["multiple_manipulators"][i]["grasp_transform"]["translation"][2].as<double>();
      double roll = config["kinematics"]["multiple_manipulators"][i]["grasp_transform"]["rotation"][0].as<double>();
      double pitch = config["kinematics"]["multiple_manipulators"][i]["grasp_transform"]["rotation"][1].as<double>();
      double yaw = config["kinematics"]["multiple_manipulators"][i]["grasp_transform"]["rotation"][2].as<double>();
      Eigen::Affine3d temp_transform;
      temp_transform = Eigen::Affine3d::Identity();
      // Create rotation matrix in order ZYX
      Eigen::Matrix3d temp_rotation;
      temp_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch,  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
      // Translate and rotate the transform
      temp_transform.translate(Eigen::Vector3d(x,y,z));
      //temp_transform.rotate(temp_rotation);
      // Append to transform list
      grasp_transforms_.push_back(temp_transform);
    }
    else {
      cout << "MultipleManipulatorsKinematics" << endl;
      cout << "  No such kinematics interface: " << 
        config["kinematics"]["multiple_manipulators"][i]["type"].as<string>() << endl;
      exit(0);
    }
  }

  Eigen::VectorXd q(5);
  q << 0.7, 0.7, 0.2, 0.5, -0.7;
  std::vector<Eigen::Affine3d> hm;
  hm = manipulators_[1]->getJointPositions(q);
  cout << hm.size() << endl;
  for (int i=0; i<hm.size(); i++){
    cout << "Link " << i << endl;
    cout << hm[i].translation() << endl;
    cout << hm[i].rotation() << endl << endl;
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
