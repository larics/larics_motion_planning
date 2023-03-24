#include <larics_motion_planning/MultipleManipulatorsKinematics.h>
#include <larics_motion_planning/MotionPlanningUtil.h>

MultipleManipulatorsKinematics::MultipleManipulatorsKinematics(string config_filename)
{
  configureFromFile(motion_util::getUserPrefix() + config_filename);
}

bool MultipleManipulatorsKinematics::configureFromFile(string config_filename)
{
  cout << "Configuring multiple manipulators kinematics from file: " << endl;
  cout << "  " << config_filename << endl;

  YAML::Node config = YAML::LoadFile(config_filename);
  // TODO: Za sada postoje ti grasp transforms koji mijenjaju prihvatnu
  // transformaciju svakog manipulatora. E sad, posto se za svaki manipulator
  // inverzna racuna iz baze, to vrlo vjerojatno nece raditi. Ono sto ce vrlo
  // vjerojatno trebati napraviti je da se prima lista transformacija kao
  // std::vector<Eigen::Affine3d>, ciji su clanovi prihvatne transformacije
  // svakog manipulatora u njegovoj bazi. Time ce se vjerojatno moci izbaciti
  // grasp transform iz ovog dijela koda, ali morat ce biti negdje drugdje ta
  // konfiguracija.

  n_manipulators_ = config["kinematics"]["multiple_manipulators"].size();
  for (int i=0; i<n_manipulators_; i++){
    if (config["kinematics"]["multiple_manipulators"][i]["type"].as<string>() == "wp_manipulator"){
      string robot_model_name, joint_group_name, dh_parameters_file;
      robot_model_name = config["kinematics"]["multiple_manipulators"][i]["robot_model_name"].as<string>();
      joint_group_name = config["kinematics"]["multiple_manipulators"][i]["joint_group_name"].as<string>();
      dh_parameters_file = config["kinematics"]["multiple_manipulators"][i]["dh_parameters_file"].as<string>();

      // Load each wp manipulator as kinematics interface shared pointer and
      // add it to the list of manipulators interfaces.
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
      if (config["kinematics"]["multiple_manipulators"][i]["grasp_transform"]["rotation_matrix"]){
        std::vector<double> r = config["kinematics"]["multiple_manipulators"][i]["grasp_transform"]["rotation_matrix"].as< std::vector<double> >();
        temp_rotation << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
      }
      else{
        temp_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch,  Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        //temp_rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        //* Eigen::AngleAxisd(pitch,  Eigen::Vector3d::UnitY())
        //* Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
      }
      // Translate and rotate the transform
      temp_transform.translate(Eigen::Vector3d(x,y,z));
      temp_transform.rotate(temp_rotation);
      // Append to transform list
      grasp_transforms_.push_back(temp_transform);
      cout << temp_transform.matrix() << endl;

      try{
        // Base degrees of freedom
        temp_dof = config["kinematics"]["multiple_manipulators"][i]["base_dof"].as<int>();
        base_n_dofs_.push_back(temp_dof);

        // Set up a fixed transform between UAV and manipulator
        std::vector<double> temp_transf;
        temp_transf = config["kinematics"]["multiple_manipulators"][i]["base_manipulator_transform"].as< std::vector<double> >();
        Eigen::Affine3d t_base_manipulator;
        t_base_manipulator = Eigen::Affine3d::Identity();
        Eigen::Matrix3d rot_uav_manipulator;
        rot_uav_manipulator = Eigen::AngleAxisd(temp_transf[5], Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(temp_transf[4],  Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(temp_transf[3], Eigen::Vector3d::UnitX());
        t_base_manipulator.translate(Eigen::Vector3d(temp_transf[0], temp_transf[1], temp_transf[2]));
        t_base_manipulator.rotate(rot_uav_manipulator);
        t_base_manipulator_vector_.push_back(t_base_manipulator);

        // Get base relative yaw
        double base_relative_yaw;
        base_relative_yaw = config["kinematics"]["multiple_manipulators"][i]["base_relative_yaw"].as<double>();
        base_relative_yaw_vector_.push_back(base_relative_yaw);
        cout << "Base relative yaw: " << base_relative_yaw << endl;
      }
      catch(...){
        string ys = "\033[0;33m";
        string ye = "\033[0m";
        cout << ys << "[MultipleManipulatorsKinematics] Missing fields." << ye << endl;
        cout << ys << "  Something wrong or missing in the kinematics." << ye << endl;
        cout << ys << "  Possible fields missing:" << ye << endl;
        cout << ys << "    base_dof" << ye << endl;
        cout << ys << "    base_manipulator_transform" << ye << endl;
        cout << ys << "    base_relative_yaw" << ye << endl;
        cout << ys << "  Setting identity transform and 6-DoF." << ye << endl;
        cout << ys << "  Ignore this message if you are not using multiple manipulators." << ye << endl;
        // Default number of degrees of freedom is 6, with identity transform.
        base_n_dofs_.push_back(6);
        Eigen::Affine3d t_base_manipulator;
        t_base_manipulator = Eigen::Affine3d::Identity();
        t_base_manipulator_vector_.push_back(t_base_manipulator);
        base_relative_yaw_vector_.push_back(0.0);
      }
    }
    else {
      cout << "MultipleManipulatorsKinematics" << endl;
      cout << "  No such kinematics interface: " << 
        config["kinematics"]["multiple_manipulators"][i]["type"].as<string>() << endl;
      exit(0);
    }
  }
  /*Eigen::VectorXd q(22);
  //q << 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22;
  q << 1,2,3,4,5,6,1.38491,-0.945959,1.57,-2,-0.23384,12,13,14,15,16,17,0.787,0.787,0.787,-1.57,0.787;
  std::vector<Eigen::Affine3d> hm;
  hm = this->getMultipleEndEffectorTransforms(q);
  for (int i=0; i<hm.size(); i++){
    cout << hm[i].translation() << endl << hm[i].rotation() << endl << endl;
  }
  

  Eigen::Affine3d hm2;
  hm2 = Eigen::Affine3d::Identity();
  hm2.translate(Eigen::Vector3d(-0.373199, 0.085557, -0.00252645));
  Eigen::Matrix3d rotm;
  rotm << 0.999987, 0.00399997, 0.00320367, -0.00320364, 1.28146e-05, -0.999995, -0.00399999, -0.999992, 4.89664e-12;
  cout << rotm << endl;
  hm2.rotate(rotm);
  bool flag;
  cout << this->calculateInverseKinematics(hm2, flag) << endl;
  */
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

    // Now that we have q, get transforms from each manipulator.
    std::vector<Eigen::Affine3d> current_transforms = 
      manipulators_[i]->getJointPositions(current_manipulator_q);
    transforms.insert(transforms.end(), current_transforms.begin(),
      current_transforms.end());
  }
  return transforms;
}

std::vector<Eigen::Affine3d> MultipleManipulatorsKinematics::getSingleManipulatorJointPositions(
  Eigen::VectorXd q, int id)
{
  std::vector<Eigen::Affine3d> transforms;

  // Set up length of the current manipulator q.
  Eigen::VectorXd current_manipulator_q(n_dofs_[id]);
  current_manipulator_q = q;

  // Now that we have q, get transforms from each manipulator.
  std::vector<Eigen::Affine3d> current_transforms = 
    manipulators_[id]->getJointPositions(current_manipulator_q);
  transforms.insert(transforms.end(), current_transforms.begin(),
    current_transforms.end());

  return transforms;
}

Eigen::Affine3d MultipleManipulatorsKinematics::getEndEffectorTransform(
  Eigen::VectorXd q)
{
  // This is required by the interface. It's not pretty, but it must be
  // implemented. It will probably be unused.
  return manipulators_[0]->getEndEffectorTransform(q);
}

std::vector<Eigen::Affine3d> MultipleManipulatorsKinematics::getMultipleEndEffectorTransforms(
  Eigen::VectorXd q)
{
  std::vector<Eigen::Affine3d> transforms;

  // Go through all manipulators
  for (int i=0; i<n_manipulators_; i++){
    // Create q for each manipulator and extract it from the full system state q.
    Eigen::VectorXd current_manipulator_q(n_dofs_[i]);
    current_manipulator_q = q.block(dofs_indexes_[i][0],0,n_dofs_[i],1);
    // Add each transform to the return vector.
    transforms.push_back(manipulators_[i]->getEndEffectorTransform(
      current_manipulator_q));
  }

  return transforms;
}

Eigen::Affine3d MultipleManipulatorsKinematics::getSingleManipulatorEndEffectorTransform(
  Eigen::VectorXd q, int id)
{
  return manipulators_[id]->getEndEffectorTransform(q);
}

Eigen::VectorXd MultipleManipulatorsKinematics::calculateInverseKinematics(
  Eigen::Affine3d transform, bool &found_ik)
{
  Eigen::VectorXd joint_states;
  
  // Final ik flag will be "and" of all manipulators' ik flags.
  found_ik = true;
  bool multiple_found_ik = true;
  // Start index of each manipulator q.
  int start_index = 0;

  // Go through all manipulators.
  for (int i=0; i<n_manipulators_; i++){
    Eigen::VectorXd current_manipulator_state;
    // State of each manipulator is multiplied by grasp transform. This might
    // have to be changed.
    current_manipulator_state = manipulators_[i]->calculateInverseKinematics(
      transform, multiple_found_ik);
    found_ik &= multiple_found_ik;
    // Append current manipulator state to the full merged state.
    joint_states.conservativeResize(joint_states.rows() + 
      current_manipulator_state.rows(), 1);
    joint_states.block(start_index, 0, current_manipulator_state.rows(), 1) = 
      current_manipulator_state;
    // increase start index for the next manipulator.
    start_index += current_manipulator_state.rows();
  }

  return joint_states;
}


Eigen::VectorXd MultipleManipulatorsKinematics::calculateSingleManipulatorInverseKinematics(
  Eigen::Affine3d transform, int id, bool &found_ik)
{
  return manipulators_[id]->calculateInverseKinematics(
    transform/*grasp_transforms_[id]*/, found_ik);
}

Eigen::MatrixXd MultipleManipulatorsKinematics::getSingleManipulatorJacobian(
  Eigen::VectorXd q, int id)
{
  return manipulators_[id]->getJacobian(q);
}

void MultipleManipulatorsKinematics::setSingleManipulatorJointPositions(
  Eigen::VectorXd q, int id)
{
  manipulators_[id]->setJointPositions(q);
}

Eigen::VectorXd MultipleManipulatorsKinematics::getSingleManipulatorStateFromObjectState(
  Eigen::VectorXd object_q, int id)
{
  // Initialize base and manipulator configuration vectors
  Eigen::VectorXd q_m;
  Eigen::VectorXd q_b;
  q_m = Eigen::VectorXd::Zero(n_dofs_[id]);
  q_b = Eigen::VectorXd::Zero(base_n_dofs_[id]);
  // Get the object transform
  Eigen::Affine3d t_w_o(Eigen::Affine3d::Identity());
  t_w_o.translate(Eigen::Vector3d(object_q(0), object_q(1), object_q(2)));
  Eigen::Matrix3d rot_object;
  rot_object = Eigen::AngleAxisd(object_q(5), Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(object_q(4),  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(object_q(3), Eigen::Vector3d::UnitX());
  t_w_o.rotate(rot_object);

  // Based on the grasp transform and the object transform, get the transform
  // of the end-effector in the world frame.
  Eigen::Affine3d t_w_ee;
  t_w_ee = t_w_o * grasp_transforms_[id];
  //cout << "t_w_o" << endl << t_w_o.matrix() << endl;
  //cout << "t_w_ee" << endl << t_w_ee.matrix() << endl;
  
  // Get the manipulator configuration vector
  shared_ptr<WpManipulatorKinematics> man;
  man = static_pointer_cast<WpManipulatorKinematics>(manipulators_[id]);
  q_m = man->calculateOptimalSingleManipulatorState(grasp_transforms_[id],
    object_q);
  
  // Get the base configuration vector
  // Transform between base and manipulator is given in config.
  Eigen::Affine3d t_b_l0 = t_base_manipulator_vector_[id];
  //cout << "t_b_l0" << endl << t_b_l0.matrix() << endl;
  // Transform between L0 and end-effector is obtainable through direct
  // kinematics
  Eigen::Affine3d t_l0_ee;
  t_l0_ee = manipulators_[id]->getEndEffectorTransform(q_m);
  //cout << "t_l0_ee" << endl << t_l0_ee.matrix() << endl;
  // transform between world and body is chained through known ones
  Eigen::Affine3d t_w_b;
  t_w_b = t_w_ee*t_l0_ee.inverse()*t_b_l0.inverse();
  // Pack in q_b
  Eigen::Vector3d xyz = t_w_b.translation();
  // Getting the roll, pitch and yaw from the transform should be the right
  // way of doing things. However, some peculiar results arose. For instance,
  // the identity rotation matrix has been interpretd as roll=pi, pitch=pi
  // and yaw=pi. Now, this is equivalent rotation to (0,0,0), but for some
  // reason I get this one. The body is considered to only have yaw rotation
  // and unfortunately there was no time to do this properly. It works, but if
  // it doesen't this might be the reason.
  Eigen::Vector3d rpy; //= t_w_b.rotation().eulerAngles(0, 1, 2);
  double yaw = object_q(5) + base_relative_yaw_vector_[id];
  rpy << 0, 0, wrapToPi(yaw);
  q_b << xyz, rpy;

  // Pack all in full state q
  Eigen::VectorXd q = Eigen::VectorXd::Zero(n_dofs_[id] + base_n_dofs_[id]);
  q << q_b, q_m;

  //cout << "t_w_b" << endl << t_w_b.matrix() << endl;

  return q;
}

Eigen::VectorXd MultipleManipulatorsKinematics::getFullSystemStateFromObjectState(
  Eigen::VectorXd object_q)
{
  Eigen::VectorXd full_state;
  for (int i=0; i<n_manipulators_; i++){
    Eigen::VectorXd current_q;
    current_q = this->getSingleManipulatorStateFromObjectState(object_q, i);
    full_state.conservativeResize(full_state.rows() + current_q.rows(), 1);
    full_state.block(full_state.rows() - current_q.rows(), 0, 
      current_q.rows(), 1) = current_q;
  }
  return full_state;
}

Eigen::MatrixXd MultipleManipulatorsKinematics::getFullSystemStateFromObjectState(
  Eigen::MatrixXd object_q)
{
  Eigen::VectorXd first_state;
  Eigen::VectorXd first_object_state = object_q.row(0).transpose();
  first_state = this->getFullSystemStateFromObjectState(first_object_state);
  Eigen::MatrixXd full_states(object_q.rows(), first_state.size());

  for (int i=0; i<full_states.rows(); i++){
    Eigen::VectorXd current_state = object_q.row(i).transpose();
    full_states.row(i) = this->getFullSystemStateFromObjectState(
      current_state).transpose();
  }

  return full_states;
}

double wrapToPi(double x){
  x = fmod(x + M_PI,2*M_PI);
  if (x < 0){
    x += 2*M_PI;
  }
  return x - M_PI;
}