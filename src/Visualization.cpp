#include <Visualization.h>

Visualization::Visualization()
{
  path_publisher_ = nh_.advertise<nav_msgs::Path>("path", 1);
  trajectory_publisher_ = nh_.advertise<nav_msgs::Path>("trajectory", 1);
}

bool Visualization::eigenPathToNavMsgsPath(Eigen::MatrixXd eigen_path, 
  double z)
{
  int n_points = eigen_path.rows();
  int n_dofs = eigen_path.cols();

  // Return false if path was planned in less than 2 dimensions.
  if(n_dofs < 2 || n_points < 2) return false;
  // Clear old path set new one.
  path_.poses.clear();
  geometry_msgs::PoseStamped current_pose;

  // If number of degrees of freedom is 2 then use z from arguments.
  if(n_dofs == 2){
    for(int i=0; i<n_points; i++){
      current_pose.pose.position.x = eigen_path(i,0);
      current_pose.pose.position.y = eigen_path(i,1);
      current_pose.pose.position.z = z;
      path_.poses.push_back(current_pose);
    }
  }
  else{
    for(int i=0; i<n_points; i++){
      current_pose.pose.position.x = eigen_path(i,0);
      current_pose.pose.position.y = eigen_path(i,1);
      current_pose.pose.position.z = eigen_path(i,2);
      path_.poses.push_back(current_pose);
    }
  }
  path_.header.stamp = ros::Time::now();
  path_.header.frame_id = "world";
  return true;
}

nav_msgs::Path Visualization::getPath()
{
  return path_;
}

void Visualization::publishPath()
{
  path_publisher_.publish(path_);
}

bool Visualization::eigenTrajectoryToNavMsgsPath(Trajectory eigen_trajectory, 
  double z)
{
  int n_points = eigen_trajectory.position.rows();
  int n_dofs = eigen_trajectory.position.cols();
  cout << "Points: " <<  n_points << " Dofs: " << n_dofs << endl;

  // Return false if path was planned in less than 2 dimensions.
  if(n_dofs < 2 || n_points < 2) return false;
  // Clear old path set new one.
  trajectory_.poses.clear();
  geometry_msgs::PoseStamped current_pose;

  // If number of degrees of freedom is 2 then use z from arguments.
  if(n_dofs == 2){
    for(int i=0; i<n_points; i++){
      current_pose.pose.position.x = eigen_trajectory.position(i,0);
      current_pose.pose.position.y = eigen_trajectory.position(i,1);
      current_pose.pose.position.z = z;
      trajectory_.poses.push_back(current_pose);
    }
  }
  else{
    for(int i=0; i<n_points; i++){
      current_pose.pose.position.x = eigen_trajectory.position(i,0);
      current_pose.pose.position.y = eigen_trajectory.position(i,1);
      current_pose.pose.position.z = eigen_trajectory.position(i,2);
      trajectory_.poses.push_back(current_pose);
    }
  }
  trajectory_.header.stamp = ros::Time::now();
  trajectory_.header.frame_id = "world";
  return true;
}

nav_msgs::Path Visualization::getTrajectory()
{
  return trajectory_;
}

void Visualization::publishTrajectory()
{
 trajectory_publisher_.publish(trajectory_);
}