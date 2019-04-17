#include <Visualization.h>

Visualization::Visualization()
{
  path_publisher_ = nh_.advertise<nav_msgs::Path>("path", 1);
  trajectory_publisher_ = nh_.advertise<nav_msgs::Path>("trajectory", 1);
  //waypoints_publisher_ = nh_.advertise<visualization_msgs::Marker>(
  //  "waypoints", 1);
}

void Visualization::setPath(Eigen::MatrixXd eigen_path, bool projection)
{
  path_ = this->eigenMatrixXdToNavMsgsPath(eigen_path, projection);
}

void Visualization::setPath(nav_msgs::Path path)
{
  path_ = path;
}

nav_msgs::Path Visualization::getPath()
{
  return path_;
}

void Visualization::publishPath()
{
  path_publisher_.publish(path_);
}

void Visualization::setTrajectory(Eigen::MatrixXd eigen_path, bool projection)
{
  trajectory_ = this->eigenMatrixXdToNavMsgsPath(eigen_path, projection);
}

void Visualization::setTrajectory(Trajectory trajectory, bool projection)
{
  trajectory_ = this->eigenMatrixXdToNavMsgsPath(trajectory.position, 
    projection);
}

void Visualization::setTrajectory(nav_msgs::Path trajectory)
{
  trajectory_ = trajectory;
}

nav_msgs::Path Visualization::getTrajectory()
{
  return trajectory_;
}

void Visualization::publishTrajectory()
{
 trajectory_publisher_.publish(trajectory_);
}

nav_msgs::Path Visualization::eigenMatrixXdToNavMsgsPath(
  Eigen::MatrixXd eigen_path, bool projection)
{
  nav_msgs::Path path;
  int n_points = eigen_path.rows();
  int n_dofs = eigen_path.cols();

  geometry_msgs::PoseStamped current_pose;

  // If number of degrees of freedom is 2 then use z from arguments.
  if(projection == true){
    for(int i=0; i<n_points; i++){
      current_pose.pose.position.x = eigen_path(i,0);
      current_pose.pose.position.y = eigen_path(i,1);
      current_pose.pose.position.z = 0.0;
      path.poses.push_back(current_pose);
    }
  }
  else{
    for(int i=0; i<n_points; i++){
      current_pose.pose.position.x = eigen_path(i,0);
      current_pose.pose.position.y = eigen_path(i,1);
      current_pose.pose.position.z = eigen_path(i,2);
      path.poses.push_back(current_pose);
    }
  }
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";
  return path;
}