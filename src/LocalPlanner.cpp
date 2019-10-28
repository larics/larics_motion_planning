#include <larics_motion_planning/LocalPlanner.h>

LocalPlanner::LocalPlanner(string config_filename)
{
  string username = "/home/";
  username = username + getenv("USER") + "/";
  configureFromFile(username + config_filename);

  // Publishers
  multi_dof_trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
    "/euroc3/command/trajectory", 1);
  manipulator_joint_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
    "/euroc3/wp_manipulator/joint_trajectory", 1);
  multi_dof_trajectory_point_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
    "/uav/multi_dof_joint_trajectory_point", 1);


  // Subscribers
  nh_.subscribe("joint_trajectory", 1, &LocalPlanner::jointTrajectoryCallback, this);
  nh_.subscribe("/euroc3/msf_core/odometry", 1, &LocalPlanner::odometryCallback, this);
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
  username = username + getenv("USER") + "/";
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
  // Fixed transformation between uav and manipulator
  Eigen::Affine3d t_b_l0;
  t_b_l0 = Eigen::Affine3d::Identity();
  Eigen::Matrix3d rot_uav_manipulator;
  /*rot_uav_manipulator = Eigen::AngleAxisd(3.14159265359, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(0,  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(1.57079632679, Eigen::Vector3d::UnitX());*/
  rot_uav_manipulator = Eigen::AngleAxisd(-1.57079632679, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(1.57079632679,  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  t_b_l0.translate(Eigen::Vector3d(0, 0, 0.125));
  t_b_l0.rotate(rot_uav_manipulator);

  // Dequeue trajectory and compensate for manipulator movement
  ros::Rate r(100); // 100Hz is sampling time of the trajectory
  for (int i=0; i<msg.points.size(); i++){
    trajectory_msgs::JointTrajectoryPoint current_point = msg.points[i];

    // Get planned transform of uav in world frame
    Eigen::Affine3d t_w_b = Eigen::Affine3d::Identity();
    t_w_b.translate(Eigen::Vector3d(current_point.positions[0],
      current_point.positions[1], current_point.positions[2]));
    Eigen::Matrix3d r_w_b;
    // At this point roll and pitch are 0 since we don't plan for them
    r_w_b = Eigen::AngleAxisd(current_point.positions[5], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(current_point.positions[4],  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(current_point.positions[3],  Eigen::Vector3d::UnitX());
    t_w_b.rotate(r_w_b);

    // Planned transform from l0 to end effector.
    Eigen::VectorXd planned_manipulator_state(3);
    planned_manipulator_state << current_point.positions[6], current_point.positions[7], current_point.positions[8];
    Eigen::Affine3d t_l0_ee = kinematics_interface_->getEndEffectorTransform(
      planned_manipulator_state);
    // Calculate planned end effector pose in global coordinate system.
    Eigen::Affine3d t_w_ee = t_w_b*t_b_l0*t_l0_ee;

    // Now we have desired end effector transformation
    // Get transform of uav in world frame
    t_w_b = Eigen::Affine3d::Identity();
    t_w_b.translate(Eigen::Vector3d(uav_current_pose_.position.x,
      uav_current_pose_.position.y, uav_current_pose_.position.z));
    Eigen::Quaterniond q;
    q.x() = uav_current_pose_.orientation.x;
    q.y() = uav_current_pose_.orientation.y;
    q.z() = uav_current_pose_.orientation.z;
    q.w() = uav_current_pose_.orientation.w;
    // Set to rotation matrix
    r_w_b = q.normalized().toRotationMatrix();
    t_w_b.rotate(r_w_b);


    // With real uav state in transformation we can calculate desired transform
    // of end effector in L0 frame. This will go into inverse kinematics to
    // calculate new joint positions for manipulator.
    t_l0_ee = t_b_l0.inverse()*t_w_b.inverse()*t_w_ee;

    // Do aforementioned inverse kinematics
    bool found_ik;
    Eigen::VectorXd ik_solution;
    ik_solution = kinematics_interface_->calculateInverseKinematics(t_l0_ee, found_ik);
    if (found_ik == false) cout << i << endl;
    else{
      current_point.positions[6] = ik_solution.transpose()[0];
      current_point.positions[7] = ik_solution.transpose()[1];
      current_point.positions[8] = ik_solution.transpose()[2];
    }

    // Publish data to uav and manipulator
    trajectory_msgs::MultiDOFJointTrajectoryPoint multi_dof_point;
    geometry_msgs::Transform temp_transform;
    temp_transform.translation.x = msg.points[i].positions[0];
    temp_transform.translation.y = msg.points[i].positions[1];
    temp_transform.translation.z = msg.points[i].positions[2];
    temp_transform.rotation.x = 0;
    temp_transform.rotation.y = 0;
    temp_transform.rotation.z = sin(msg.points[i].positions[5]/2.0);
    temp_transform.rotation.w = cos(msg.points[i].positions[5]/2.0);
    // Velocities
    geometry_msgs::Twist temp_velocity;
    temp_velocity.linear.x = msg.points[i].velocities[0];
    temp_velocity.linear.y = msg.points[i].velocities[1];
    temp_velocity.linear.z = msg.points[i].velocities[2];
    // Accelerations
    geometry_msgs::Twist temp_acceleration;
    temp_acceleration.linear.x = msg.points[i].accelerations[0];
    temp_acceleration.linear.y = msg.points[i].accelerations[1];
    temp_acceleration.linear.z = msg.points[i].accelerations[2];
    // Push to point and trajectory
    multi_dof_point.transforms.push_back(temp_transform);
    multi_dof_point.velocities.push_back(temp_velocity);
    multi_dof_point.accelerations.push_back(temp_acceleration);
    // Trajectory
    trajectory_msgs::MultiDOFJointTrajectory multi_dof_trajectory;
    multi_dof_trajectory.points.push_back(multi_dof_point);

    // Manipulator as well
    trajectory_msgs::JointTrajectory manipulator_trajectory;
    trajectory_msgs::JointTrajectoryPoint manipulator_point;
    manipulator_point.positions.push_back(msg.points[i].positions[6]);
    manipulator_point.positions.push_back(msg.points[i].positions[7]);
    manipulator_point.positions.push_back(msg.points[i].positions[8]);
    for (int j=0; j<3; j++){
      manipulator_point.velocities.push_back(0);
      manipulator_point.accelerations.push_back(0);
      manipulator_point.effort.push_back(0);
    }
    manipulator_point.time_from_start = ros::Duration(0.001);
    // Fill the manipulator trajectory with current point
    manipulator_trajectory.points.push_back(manipulator_point);
    manipulator_trajectory.header.frame_id = "world";
    manipulator_trajectory.joint_names.push_back("joint1");
    manipulator_trajectory.joint_names.push_back("joint2");
    manipulator_trajectory.joint_names.push_back("joint3");
    manipulator_trajectory.header.stamp = ros::Time::now();
    // TODO: publish both messages
  }
}

void LocalPlanner::odometryCallback(const nav_msgs::Odometry &msg)
{
  uav_current_pose_ = msg.pose.pose;
}
