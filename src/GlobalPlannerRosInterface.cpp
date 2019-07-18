#include <GlobalPlannerRosInterface.h>

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

  // TODO: Delete this service. Just testing service for now.
  empty_service_server_ = nh_.advertiseService("empty_service_test",
    &GlobalPlannerRosInterface::emptyCallback, this);
  // Service for planning the cartesian trajectory.
  cartesian_trajectory_server_ = nh_.advertiseService("cartesian_trajectory",
    &GlobalPlannerRosInterface::cartesianTrajectoryCallback, this);
  // Service for planning the multi dof trajectory.
  multi_dof_trajectory_server_ = nh_.advertiseService("multi_dof_trajectory",
    &GlobalPlannerRosInterface::multiDofTrajectoryCallback, this);
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
    double roll = -trajectory.acceleration(i, 1)/9.81;
    double pitch = trajectory.acceleration(i, 0)/9.81;
    visualization_.setStatePoints(
      global_planner_->getRobotStatePoints((trajectory.position.row(i)).transpose(), roll, pitch));
    visualization_.publishStatePoints();
    usleep(10000);
  }
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