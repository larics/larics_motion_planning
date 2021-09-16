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
      n_dofs_.push_back(temp_dof);
      std::vector<int> temp_indexes = config["kinematics"]["multiple_manipulators"][i]["indexes"].as< std::vector<int> >();
      if (temp_dof != temp_indexes.size()){
        cout << "MultipleManipulatorsKinematics" << endl;
        cout << "Manipulator config " << i << endl;
        cout << "  Number of dofs is different from number of indexes." << endl;
        exit(0);
      }
      else {
        dofs_indexes_.push_back(temp_indexes);
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
  Eigen::VectorXd q(22);
  //q << 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22;
  q << 1,2,3,4,5,6,1.38491,-0.945959,1.57,-2,-0.23384,12,13,14,15,16,17,0.787,0.787,0.787,-1.57,0.787;
  std::vector<Eigen::Affine3d> hm;
  hm = this->getMultipleEndEffectorTransforms(q);
  for (int i=0; i<hm.size(); i++){
    cout << hm[i].translation() << endl << hm[i].rotation() << endl << endl;
  }
  exit(0);

  Eigen::Affine3d hm2;
  hm2 = Eigen::Affine3d::Identity();
  hm2.translate(Eigen::Vector3d(-0.373199, 0.085557, -0.00252645));
  Eigen::Matrix3d rotm;
  rotm << 0.999987, 0.00399997, 0.00320367, -0.00320364, 1.28146e-05, -0.999995, -0.00399999, -0.999992, 4.89664e-12;
  cout << rotm << endl;
  hm2.rotate(rotm);
  bool flag;
  cout << this->calculateInverseKinematics(hm2, flag) << endl;
}

std::vector<Eigen::Affine3d> MultipleManipulatorsKinematics::getJointPositions(
  Eigen::VectorXd q)
{
  std::vector<Eigen::Affine3d> transforms;
  // Go through all manipulators in the list.
  for (int i=0; i<n_manipulators_; i++){
    // Set up length of the current manipulator q.
    Eigen::VectorXd current_manipulator_q(n_dofs_[i]);
    // Get joint values from the full q as a blok from start index row, column 0,
    // and size of n_dofs_[i], 1.
    current_manipulator_q = q.block(dofs_indexes_[i][0],0,n_dofs_[i],1);
    //for (int j=0; j<n_dofs_[i]; j++){
    //  current_manipulator_q(j) = q(dofs_indexes_[i][j]);
    //}

    // Now that we have q, get transforms from each manipulator.
    std::vector<Eigen::Affine3d> current_transforms = 
      manipulators_[i]->getJointPositions(current_manipulator_q);
    transforms.insert(transforms.end(), current_transforms.begin(),
      current_transforms.end());
  }
  return transforms;
}

Eigen::Affine3d MultipleManipulatorsKinematics::getEndEffectorTransform(
  Eigen::VectorXd q)
{
  return manipulators_[0]->getEndEffectorTransform(q);
}

std::vector<Eigen::Affine3d> MultipleManipulatorsKinematics::getMultipleEndEffectorTransforms(
  Eigen::VectorXd q)
{
  std::vector<Eigen::Affine3d> transforms;

  for (int i=0; i<n_manipulators_; i++){
    Eigen::VectorXd current_manipulator_q(n_dofs_[i]);
    current_manipulator_q = q.block(dofs_indexes_[i][0],0,n_dofs_[i],1);
    transforms.push_back(manipulators_[i]->getEndEffectorTransform(current_manipulator_q));
  }

  return transforms;
}

Eigen::VectorXd MultipleManipulatorsKinematics::calculateInverseKinematics(
  Eigen::Affine3d transform, bool &found_ik)
{
  Eigen::VectorXd joint_states;
  cout << transform.translation() << endl << transform.rotation() << endl;
  cout << joint_states.rows() << endl;
  bool multiple_found_ik = true;
  int start_index = 0;
  for (int i=0; i<n_manipulators_; i++){
    Eigen::VectorXd current_manipulator_state;
    current_manipulator_state = 
      manipulators_[i]->calculateInverseKinematics(transform, multiple_found_ik);
    cout << "cms " << endl << current_manipulator_state << endl << endl;
    found_ik &= multiple_found_ik;
    joint_states.conservativeResize(joint_states.rows() + 
      current_manipulator_state.rows(), 1);
    joint_states.block(start_index, 0, current_manipulator_state.rows(), 1) = 
      current_manipulator_state;
    start_index += current_manipulator_state.rows();
    cout << "Joint states" << endl << joint_states << endl << endl;
  }

  return joint_states;
}
