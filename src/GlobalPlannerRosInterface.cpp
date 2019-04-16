#include <GlobalPlannerRosInterface.h>

GlobalPlannerRosInterface::GlobalPlannerRosInterface(string s)
{
  global_planner_ = make_shared<GlobalPlanner>(s);

  // Publishers
  multi_dof_trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "multi_dof_trajectory", 1);
  cartesian_path_pub_ = nh_.advertise<nav_msgs::Path>("cartesian_path", 1);

  empty_service_server_ = nh_.advertiseService("empty_service_test",
    &GlobalPlannerRosInterface::emptyCallback, this);

  cartesian_trajectory_server_ = nh_.advertiseService("cartesian_trajectory",
    &GlobalPlannerRosInterface::cartesianTrajectoryCallback, this);
}

void GlobalPlannerRosInterface::run()
{
  ros::Rate loop_rate(10);

  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool GlobalPlannerRosInterface::emptyCallback(std_srvs::Empty::Request &req, 
  std_srvs::Empty::Response &res)
{
  cout << "Empty servce working" << endl;

  return true;
}

bool GlobalPlannerRosInterface::cartesianTrajectoryCallback(
  larics_motion_planning::CartesianTrajectory::Request &req, 
  larics_motion_planning::CartesianTrajectory::Response &res)
{
  cout << "Cartesian Trajectory Callback" << endl;
  // Convert waypoints to planner message type
  Eigen::MatrixXd waypoints = this->navMsgsPathToEigenMatrixXd(req.waypoints);
  // Plan path and trajectory.
  bool success = global_planner_->planPathAndTrajectory(waypoints);

  // Get trajectory
  res.trajectory = this->trajectoryToMultiDofTrajectory(
    global_planner_->getTrajectory());

  // Get path
  res.path = this->eigenMatrixXdToNavMsgsPath(global_planner_->getPath());

  if (req.publish_trajectory){
    multi_dof_trajectory_pub_.publish(res.trajectory);
  }
  if (req.publish_path){
    cartesian_path_pub_.publish(res.path);
  }

  return success;
}

Eigen::MatrixXd GlobalPlannerRosInterface::navMsgsPathToEigenMatrixXd(
  nav_msgs::Path nav_path)
{
  Eigen::MatrixXd eigen_path(nav_path.poses.size(), 3);

  for (int i=0; i<nav_path.poses.size(); i++){
    eigen_path(i,0) = nav_path.poses[i].pose.position.x;
    eigen_path(i,1) = nav_path.poses[i].pose.position.y;
    eigen_path(i,2) = nav_path.poses[i].pose.position.z;
  }

  return eigen_path;
}

nav_msgs::Path GlobalPlannerRosInterface::eigenMatrixXdToNavMsgsPath(
  Eigen::MatrixXd eigen_path)
{
  nav_msgs::Path nav_path;
  geometry_msgs::PoseStamped current_pose;

  for (int i=0; i<eigen_path.rows(); i++){
    current_pose.pose.position.x = eigen_path(i,0);
    current_pose.pose.position.y = eigen_path(i,1);
    current_pose.pose.position.z = eigen_path(i,2);
    current_pose.pose.orientation.w = 1.0;

    nav_path.poses.push_back(current_pose);
    nav_path.header.stamp = ros::Time::now();
  }

  return nav_path;
}


trajectory_msgs::MultiDOFJointTrajectory GlobalPlannerRosInterface::trajectoryToMultiDofTrajectory(
  Trajectory eigen_trajectory)
{
  trajectory_msgs::MultiDOFJointTrajectory ros_trajectory;
  geometry_msgs::Transform current_transform;
  geometry_msgs::Twist current_velocity, current_acceleration;

  for (int i=0; i<eigen_trajectory.position.rows(); i++){
    trajectory_msgs::MultiDOFJointTrajectoryPoint current_point;

    current_transform.translation.x = eigen_trajectory.position(i,0);
    current_transform.translation.y = eigen_trajectory.position(i,1);
    current_transform.translation.z = eigen_trajectory.position(i,2);
    current_transform.rotation.w = 1.0;

    current_velocity.linear.x = eigen_trajectory.velocity(i,0);
    current_velocity.linear.y = eigen_trajectory.velocity(i,1);
    current_velocity.linear.z = eigen_trajectory.velocity(i,2);

    current_acceleration.linear.x = eigen_trajectory.acceleration(i,0);
    current_acceleration.linear.y = eigen_trajectory.acceleration(i,1);
    current_acceleration.linear.z = eigen_trajectory.acceleration(i,2);

    current_point.transforms.push_back(current_transform);
    current_point.velocities.push_back(current_velocity);
    current_point.accelerations.push_back(current_acceleration);
    current_point.time_from_start = ros::Duration(eigen_trajectory.time(i));

    ros_trajectory.points.push_back(current_point);
    ros_trajectory.header.stamp = ros::Time::now();
  }

  return ros_trajectory;
}