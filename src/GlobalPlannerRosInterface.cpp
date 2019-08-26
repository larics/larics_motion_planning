#include <larics_motion_planning/GlobalPlannerRosInterface.h>

GlobalPlannerRosInterface::GlobalPlannerRosInterface()
{
  string global_planner_config_file;

  ros::NodeHandle nh_private = ros::NodeHandle("~");

  nh_private.param("global_planner_config_file", global_planner_config_file, 
    string("/home/antun/catkin_ws/src/larics_motion_planning/config/one_file_config_example.yaml"));
  nh_private.param("rate", rate_, int(10));

  // Global planner config
  global_planner_ = make_shared<GlobalPlanner>(global_planner_config_file);
  visualization_changed_ = false;

  // Publishers
  // Multi degree of freedom trajectory
  multi_dof_trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "multi_dof_trajectory", 1);
  // Path publisher
  cartesian_path_pub_ = nh_.advertise<nav_msgs::Path>("cartesian_path", 1);
  // Joint trajectory publisher
  joint_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
    "joint_trajectory", 1);

  // TODO: Delete this service. Just testing service for now.
  empty_service_server_ = nh_.advertiseService("empty_service_test",
    &GlobalPlannerRosInterface::emptyCallback, this);
  // Service for planning the cartesian trajectory.
  cartesian_trajectory_server_ = nh_.advertiseService("cartesian_trajectory",
    &GlobalPlannerRosInterface::cartesianTrajectoryCallback, this);
  // Service for planning the multi dof trajectory.
  multi_dof_trajectory_server_ = nh_.advertiseService("multi_dof_trajectory",
    &GlobalPlannerRosInterface::multiDofTrajectoryCallback, this);
  // Service for visualizing arbitrary robot state
  visualize_state_server_ = nh_.advertiseService("visualize_state", 
    &GlobalPlannerRosInterface::visualizeStateCallback, this);
}

void GlobalPlannerRosInterface::run()
{
  ros::Rate loop_rate(rate_);

  while (ros::ok()){
    ros::spinOnce();
    if (visualization_changed_ == true){
      visualization_.publishAll();
      visualization_changed_ = false;
      visualization_.clearAll();
    }
    //visualization_.setStatePoints(global_planner_->getRobotStatePoints());
    //visualization_.publishStatePoints();
    loop_rate.sleep();
  }
}

bool GlobalPlannerRosInterface::emptyCallback(std_srvs::Empty::Request &req, 
  std_srvs::Empty::Response &res)
{
  cout << "Empty service working" << endl;
  Trajectory trajectory = global_planner_->getTrajectory();

  cout << "Cols: " << trajectory.position.cols() << " Rows: " << trajectory.position.rows() << endl;

  for (int i=0; i<trajectory.position.rows(); i++){
    trajectory.position(i, 3) = -trajectory.acceleration(i, 1)/9.81;
    trajectory.position(i, 4) = trajectory.acceleration(i, 0)/9.81;
    visualization_.setStatePoints(
      global_planner_->getRobotStatePoints((trajectory.position.row(i)).transpose()));
    visualization_.publishStatePoints();
    usleep(10000);
  }
  string tempstr;
  cout << "Press enter to publish non compensated trajectory" << endl;
  getline(cin, tempstr);
  joint_trajectory_pub_.publish(trajectoryToJointTrajectory(trajectory));
  usleep(1000000);
  // Compensation part
  // First get fixed transform between uav and manipulator
  Eigen::Affine3d t_b_l0;
  t_b_l0 = Eigen::Affine3d::Identity();
  Eigen::Matrix3d rot_uav_manipulator;
  rot_uav_manipulator = Eigen::AngleAxisd(3.14159265359, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(0,  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(1.57079632679, Eigen::Vector3d::UnitX());
  t_b_l0.translate(Eigen::Vector3d(0, 0, 0.075));
  t_b_l0.rotate(rot_uav_manipulator);

  shared_ptr<KinematicsInterface> kinematics = global_planner_->getKinematicsInterface();
  // Go through all trajectory points.
  for (int i=0; i<trajectory.position.rows(); i++){
    // Get transform of uav in world frame
    Eigen::Affine3d t_w_b = Eigen::Affine3d::Identity();
    t_w_b.translate(Eigen::Vector3d(trajectory.position(i, 0), 
      trajectory.position(i, 1), trajectory.position(i, 2)));
    Eigen::Matrix3d r_w_b;
    // At this point roll and pitch are 0 since we don't plan for them
    r_w_b = Eigen::AngleAxisd(trajectory.position(5), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(trajectory.position(4)*0.0,  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(trajectory.position(3)*0.0,  Eigen::Vector3d::UnitX());
    t_w_b.rotate(r_w_b);

    // Transform from l0 to end effector.
    // TODO: Provjeriti ovaj dio ako ne radi.
    Eigen::Affine3d t_l0_ee = kinematics->getEndEffectorTransform(
      (trajectory.position.block(i, 6, 1, 5)).transpose());
    // Calculate end effector pose in global coordinate system.
    Eigen::Affine3d t_w_ee = t_w_b*t_b_l0*t_l0_ee;

    // Now we have pose of the end effector that we desire, and it was planned
    // without any knowledge of roll and pitch. The idea is to include roll and
    // pitch now.
    t_w_b = Eigen::Affine3d::Identity();
    t_w_b.translate(Eigen::Vector3d(trajectory.position(i, 0), 
      trajectory.position(i, 1), trajectory.position(i, 2)));
    // At this point roll and pitch are 0 since we don't plan for them
    double roll = -trajectory.acceleration(i, 1)/9.81;
    double pitch = trajectory.acceleration(i, 0)/9.81;
    double dy = t_w_ee.translation().y() - t_w_b.translation().y();
    double dx = t_w_ee.translation().x() - t_w_b.translation().x();
    double yaw = atan2(dy, dx);
    r_w_b = Eigen::AngleAxisd(trajectory.position(5) /*yaw*/, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    t_w_b.rotate(r_w_b);

    // With new transform we can calculate true end effector position in
    // manipulator base frame. Inverse kinematics works in l0(base manipulator
    // frame).
    t_l0_ee = t_b_l0.inverse()*t_w_b.inverse()*t_w_ee;

    // Inverzna za svaku točku. Dodati funkciju inverzne da prima Affine3d
    // i da vraća VectorXd.
    bool found_ik;
    Eigen::VectorXd ik_solution;
    ik_solution = kinematics->calculateInverseKinematics(t_l0_ee, found_ik);
    if (found_ik == false) cout << i << " " << yaw << endl;
    else{
      trajectory.position.block(i, 6, 1, 5) = ik_solution.transpose();
    }
  }
  for (int i=0; i<trajectory.position.rows(); i++){
    trajectory.position(i, 3) = -trajectory.acceleration(i, 1)/9.81;
    trajectory.position(i, 4) = trajectory.acceleration(i, 0)/9.81;
    visualization_.setStatePoints(
      global_planner_->getRobotStatePoints((trajectory.position.row(i)).transpose()));
    visualization_.publishStatePoints();
    usleep(100000);
  }
  cout << "Press enter to publish compensated trajectory" << endl;
  getline(cin, tempstr);
  joint_trajectory_pub_.publish(trajectoryToJointTrajectory(trajectory));

  return true;
}

bool GlobalPlannerRosInterface::cartesianTrajectoryCallback(
  larics_motion_planning::CartesianTrajectory::Request &req, 
  larics_motion_planning::CartesianTrajectory::Response &res)
{
  visualization_changed_ = true;
  bool success = false;
  cout << "Cartesian Trajectory Callback" << endl;
  // Convert waypoints to planner message type
  Eigen::MatrixXd waypoints = this->navMsgsPathToEigenPath(req.waypoints);
  visualization_.setWaypoints(waypoints);
    
  // Plan path and trajectory.
  if (req.plan_path == true && req.plan_trajectory == true){
    success = global_planner_->planPathAndTrajectory(waypoints);
    // Get trajectory
    res.trajectory = this->trajectoryToMultiDofTrajectory(
      global_planner_->getTrajectory());
    visualization_.setTrajectory(global_planner_->getTrajectory());

    // Get path
    res.path = this->eigenPathToNavMsgsPath(global_planner_->getPath());
    res.path_length = global_planner_->getPathLength();
    visualization_.setPath(global_planner_->getPath());
  }
  else if (req.plan_path == true && req.plan_trajectory == false){
    success = global_planner_->planPath(waypoints);
    // Get path
    res.path = this->eigenPathToNavMsgsPath(global_planner_->getPath());
    res.path_length = global_planner_->getPathLength();
    visualization_.setPath(global_planner_->getPath());
  }
  else if (req.plan_path == false && req.plan_trajectory == true){
    success = global_planner_->planTrajectory(waypoints);
    // Get trajectory
    res.trajectory = this->trajectoryToMultiDofTrajectory(
      global_planner_->getTrajectory());
    visualization_.setTrajectory(global_planner_->getTrajectory());
  }
  else{
    return success;
  }

  // If path or trajectory are to be published, then publish them.
  if (req.publish_trajectory){
    multi_dof_trajectory_pub_.publish(res.trajectory);
  }
  if (req.publish_path){
    cartesian_path_pub_.publish(res.path);
  }

  return success;
}

bool GlobalPlannerRosInterface::multiDofTrajectoryCallback(
  larics_motion_planning::MultiDofTrajectory::Request &req, 
  larics_motion_planning::MultiDofTrajectory::Response &res)
{
  visualization_changed_ = true;
  bool success = false;
  cout << "Multi DOF Trajectory Callback" << endl;
  // Convert waypoints to planner message type
  if (req.waypoints.points.size() < 2){
    cout << "At least two points required for generating trajectory." << endl;
    return success;
  }
  Eigen::MatrixXd waypoints = this->jointTrajectoryToEigenWaypoints(req.waypoints);
  visualization_.setWaypoints(waypoints);
    
  // Plan path and trajectory.
  if (req.plan_path == true && req.plan_trajectory == true){
    success = global_planner_->planPathAndTrajectory(waypoints);
    // Get trajectory
    res.trajectory = this->trajectoryToJointTrajectory(
      global_planner_->getTrajectory());
    visualization_.setTrajectory(global_planner_->getTrajectory());

    // Get path
    res.path = this->eigenPathToJointTrajectory(global_planner_->getPath());
    res.path_length = global_planner_->getPathLength();
    visualization_.setPath(global_planner_->getPath());
  }
  else if (req.plan_path == true && req.plan_trajectory == false){
    success = global_planner_->planPath(waypoints);
    // Get path
    res.path = this->eigenPathToJointTrajectory(global_planner_->getPath());
    res.path_length = global_planner_->getPathLength();
    visualization_.setPath(global_planner_->getPath());
  }
  else if (req.plan_path == false && req.plan_trajectory == true){
    success = global_planner_->planTrajectory(waypoints);
    // Get trajectory
    res.trajectory = this->trajectoryToJointTrajectory(
      global_planner_->getTrajectory());
    visualization_.setTrajectory(global_planner_->getTrajectory());
  }
  else{
    return success;
  }

  // If path or trajectory are to be published, then publish them.
  /*if (req.publish_trajectory){
    multi_dof_trajectory_pub_.publish(res.trajectory);
  }
  if (req.publish_path){
    cartesian_path_pub_.publish(res.path);
  }*/
  return success;
}

bool GlobalPlannerRosInterface::visualizeStateCallback(
  larics_motion_planning::VisualizeState::Request &req, 
  larics_motion_planning::VisualizeState::Response &res)
{
  Eigen::VectorXd state(req.state.data.size());
  for (int i=0; i<req.state.data.size(); i++){
    state(i) = req.state.data[i];
  }
  visualization_.setStatePoints(global_planner_->getRobotStatePoints(state));
  visualization_.publishStatePoints();

  cout << req.state.data.size() << endl;
  return true;
}

Eigen::MatrixXd GlobalPlannerRosInterface::navMsgsPathToEigenPath(
  nav_msgs::Path nav_path)
{
  Eigen::MatrixXd eigen_path(nav_path.poses.size(), 3);

  // Go through the whole path and set it to Eigen::MatrixXd.
  for (int i=0; i<nav_path.poses.size(); i++){
    eigen_path(i,0) = nav_path.poses[i].pose.position.x;
    eigen_path(i,1) = nav_path.poses[i].pose.position.y;
    eigen_path(i,2) = nav_path.poses[i].pose.position.z;
  }

  return eigen_path;
}

nav_msgs::Path GlobalPlannerRosInterface::eigenPathToNavMsgsPath(
  Eigen::MatrixXd eigen_path)
{
  nav_msgs::Path nav_path;
  geometry_msgs::PoseStamped current_pose;

  // Convert eigen path to nav_msgs::Path
  for (int i=0; i<eigen_path.rows(); i++){
    // Since path has list poses first create PoseStamped for each pose.
    current_pose.pose.position.x = eigen_path(i,0);
    current_pose.pose.position.y = eigen_path(i,1);
    current_pose.pose.position.z = eigen_path(i,2);
    current_pose.pose.orientation.w = 1.0;

    // Then push the poses
    nav_path.poses.push_back(current_pose);
  }
  nav_path.header.stamp = ros::Time::now();

  return nav_path;
}


trajectory_msgs::MultiDOFJointTrajectory GlobalPlannerRosInterface::trajectoryToMultiDofTrajectory(
  Trajectory eigen_trajectory)
{
  trajectory_msgs::MultiDOFJointTrajectory ros_trajectory;
  geometry_msgs::Transform current_transform;
  geometry_msgs::Twist current_velocity, current_acceleration;

  // Fill in the MultiDOFJointTrajectory message
  for (int i=0; i<eigen_trajectory.position.rows(); i++){
    // This message consists of points of type below. Each point then has a 
    // list of transforms, velocities and accelerations. Since all these are
    // lists simply erase current_point by creating it at the beginning of 
    // the loop.
    trajectory_msgs::MultiDOFJointTrajectoryPoint current_point;

    // Fill in the position from trajectory.
    current_transform.translation.x = eigen_trajectory.position(i,0);
    current_transform.translation.y = eigen_trajectory.position(i,1);
    current_transform.translation.z = eigen_trajectory.position(i,2);
    current_transform.rotation.w = 1.0; // Disregard orientation. For now

    // Fill in the velocities from trajectory.
    current_velocity.linear.x = eigen_trajectory.velocity(i,0);
    current_velocity.linear.y = eigen_trajectory.velocity(i,1);
    current_velocity.linear.z = eigen_trajectory.velocity(i,2);

    // Fill in the accelerations from trajectory.
    current_acceleration.linear.x = eigen_trajectory.acceleration(i,0);
    current_acceleration.linear.y = eigen_trajectory.acceleration(i,1);
    current_acceleration.linear.z = eigen_trajectory.acceleration(i,2);

    // Push positions, velocities, accelerations and time to current point.
    current_point.transforms.push_back(current_transform);
    current_point.velocities.push_back(current_velocity);
    current_point.accelerations.push_back(current_acceleration);
    current_point.time_from_start = ros::Duration(eigen_trajectory.time(i));

    // Push point to the trajectory.
    ros_trajectory.points.push_back(current_point);
  }
  ros_trajectory.header.stamp = ros::Time::now();

  return ros_trajectory;
}

Eigen::MatrixXd GlobalPlannerRosInterface::jointTrajectoryToEigenWaypoints(
  trajectory_msgs::JointTrajectory joint_trajectory)
{
  Eigen::MatrixXd eigen_matrix(joint_trajectory.points.size(), 
    joint_trajectory.points[0].positions.size());

  // Go through the whole path and set it to Eigen::MatrixXd.
  for (int i=0; i<joint_trajectory.points.size(); i++){
    for (int j=0; j<joint_trajectory.points[0].positions.size(); j++){
      eigen_matrix(i,j) = joint_trajectory.points[i].positions[j];
    }
  }

  return eigen_matrix;
}

trajectory_msgs::JointTrajectory GlobalPlannerRosInterface::eigenPathToJointTrajectory(
  Eigen::MatrixXd eigen_path)
{
  trajectory_msgs::JointTrajectory joint_trajectory;
  int n_dofs = eigen_path.cols();

  // Go through all points in trajectory
  for (int i=0; i<eigen_path.rows(); i++){
    trajectory_msgs::JointTrajectoryPoint current_point;

    // Go through all degrees of freedom
    for (int j=0; j<n_dofs; j++){
      current_point.positions.push_back(eigen_path(i,j));
    }

    // Add current point to trajectory
    joint_trajectory.points.push_back(current_point);
  }

  return joint_trajectory;
}

trajectory_msgs::JointTrajectory GlobalPlannerRosInterface::trajectoryToJointTrajectory(
  Trajectory eigen_trajectory)
{
  trajectory_msgs::JointTrajectory joint_trajectory;
  int n_dofs = eigen_trajectory.position.cols();

  // Go through all points in trajectory
  for (int i=0; i<eigen_trajectory.position.rows(); i++){
    trajectory_msgs::JointTrajectoryPoint current_point;

    // Go through all degrees of freedom
    for (int j=0; j<n_dofs; j++){
      current_point.positions.push_back(eigen_trajectory.position(i,j));
      current_point.velocities.push_back(eigen_trajectory.velocity(i,j));
      current_point.accelerations.push_back(eigen_trajectory.acceleration(i,j));
      current_point.time_from_start = ros::Duration(eigen_trajectory.time(i));
    }

    // Add current point to trajectory
    joint_trajectory.points.push_back(current_point);
  }

  return joint_trajectory;
}