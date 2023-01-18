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

Eigen::VectorXd WpManipulatorKinematics::calculateOptimalSingleManipulatorState(
  Eigen::Affine3d grasp_transform, Eigen::VectorXd object_state)
{
  //Eigen::Vector3d rpy = grasp_transform.rotation().eulerAngles(2, 1, 0);
  //cout << "rpy: " << rpy.transpose() << endl;
  //rpy = object_state.block(3,0,3,1);
  //cout << "rpy from object: " << rpy.transpose() << endl;

  //Eigen::Vector3d p_o_ee = object_state.block(0,0,3,1) - grasp_transform.translation();
  //cout << "Delta p: " << p_o_ee.transpose() << endl;
  //cout << "Object position: " << object_state.block(0,0,3,1).transpose() << endl;
  //cout << "Tool translation: " << grasp_transform.translation().transpose() << endl;

  // Imagine object at the (0,0,0) position. The grasp translation vector is
  // used to define the DeltaZ, when it is rotated by the object pitch angle.
  // This is used to properly set the manipulator q later on.
  Eigen::Matrix3d rot_object;
  rot_object = Eigen::AngleAxisd(object_state(5), Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(object_state(4),  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(object_state(3), Eigen::Vector3d::UnitX());
  Eigen::Vector3d p_o_ee = rot_object*grasp_transform.translation();
  Eigen::Vector3d rpy = object_state.block(3,0,3,1);
  //cout << "grasp translation: " << grasp_transform.translation().transpose() << endl;
  //cout << "p_o_ee: " << p_o_ee.transpose() << endl;
  //cout << "rpy: " << rpy.transpose() << endl;

  Eigen::VectorXd q;
  // These ifs are non configurable for now.
  if (robot_model_name_ == "wp_manipulator"){
    q = Eigen::VectorXd::Zero(5);
    // The amount of rotation for each joint is determined from the roll,
    // pitch and yaw of the object. Since manipulator attachment points are on
    // different sides of the stick like object, one manipulator will have to
    // point down, and the other one up. Delta in z-axis determines the
    // direction these joints have to move.
    double sgn = signum(p_o_ee(2));
    //cout << "p_o_ee(2): " << p_o_ee(2) << endl;
    //cout << "signum: " << sgn << endl;
    //q << M_PI/4 + sgn*rpy(1)/2, M_PI/4 + sgn*rpy(1)/2, 0.557, -0.861, 0.304;
    
    // It is interesting that this has to be separated based on the desired
    // onbject pitch angle. Nevertheless, this way it works. I think it has
    // something to do with the fact that I am trying to set some rotations
    // from homogeneous transform directly into the manipulator q, and this
    // is not as straight forward as it seems.
    if (rpy(1) >= 0){
      q << M_PI/4 + sgn*rpy(1)/2, M_PI/4 + sgn*rpy(1)/2, 0.557, -0.861, 0.304;
    }
    else {
      q << M_PI/4 - sgn*rpy(1)/2, M_PI/4 - sgn*rpy(1)/2, 0.557, -0.861, 0.304;
    }
  }
  else if (robot_model_name_ == "wp_manipulator_3rx"){
    q = Eigen::VectorXd::Zero(3);
    double sgn = signum(p_o_ee(2));
    // Parameters from matlab analysis. The optimal parameters are lying on a
    // line with discontinuity at the middle. That's why we have _neg and _pos
    // line parameters. The line equation is q = m*pitch + c
    Eigen::Vector3d m; m << 0.5622, -0.1140, 0.5517;
    Eigen::Vector3d c_neg; c_neg << 0.9067, -0.5266, -0.3801;
    Eigen::Vector3d c_pos; c_pos << -0.9067, 0.5266, 0.3801;
    if (rpy(1) >= 0){
      //q = sgn*m*rpy(1) + c_pos;
      q = sgn*m*rpy(1) + c_pos;
    }
    else{
      // It should be +c_neg but then manipulator moves down towards propellers
      // Maybe the sensible thing is to approach this logically, having it this
      // way will keep the manipulator above the propellers. If the manipulator
      // is mounted below the center of gravity, it should be inverted for 180
      // degrees to retain the same calculation.
      //q = -sgn*m*rpy(1) - c_neg;
      q = -sgn*m*rpy(1) - c_neg;
    }
    /*if (rpy(1) >= 0){
      q << -0.861 + sgn*rpy(1)/3, 0.557 + sgn*rpy(1)/3, 0.304 + sgn*rpy(1)/3;
    }
    else {
      q << -0.861 - sgn*rpy(1)/3, 0.557 - sgn*rpy(1)/3, 0.304 - sgn*rpy(1)/3;
    }*/
    //-0.861 0.557 0.304
  }
  else if (robot_model_name_ == "asap_manipulator_4r"){
    q = Eigen::VectorXd::Zero(4);
    double sgn = signum(p_o_ee(2));
    // Let's try the same thing as for the wp_manipulator_3rx. The computed
    // line equations are a bit different, but the principle remains. The first
    // joint is always 0 so both m and c are 0 for it.
    Eigen::VectorXd m(4); m << 0, 0.6845, 0.0153, 0.3002;
    Eigen::VectorXd c_neg(4); c_neg << 0, 0.8707, -0.6696, -0.2011;
    Eigen::VectorXd c_pos(4); c_pos << 0, -0.8707, 0.6696, 0.2011;
    if (rpy(1) <= 0){
      //q = sgn*m*rpy(1) + c_pos;
      q = sgn*m*rpy(1) + c_pos;
    }
    else{
      q = -sgn*m*rpy(1) - c_neg;
    }

    //q << 0, -0.861 + rpy(1)/3, 0.557 + rpy(1)/3, 0.304 + rpy(1)/3;
  }
  else{
    cout << "[WpManipulatorKinematics] No such manipulator robot model." << endl;
    cout << "  Your provided robot model is: " << robot_model_name_ << endl;
  }

  // If wrong manipulator robot model is provided, return empty q. The caller should
  // check the size of q.
  return q;
}


double signum(double val){
  if (val >= 0) return 1.0;
  else return -1.0;
}