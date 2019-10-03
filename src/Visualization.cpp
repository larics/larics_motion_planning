#include <larics_motion_planning/Visualization.h>

Visualization::Visualization()
{
  ros::NodeHandle nh_private = ros::NodeHandle("~");
  nh_private.param("state_scale", state_scale_, double(0.1));
  nh_private.param("waypoints_scale", waypoints_scale_, double(0.2));

  path_publisher_ = nh_.advertise<nav_msgs::Path>("visualization/path", 1);
  trajectory_publisher_ = nh_.advertise<nav_msgs::Path>(
    "visualization/trajectory", 1);
  waypoints_publisher_ = nh_.advertise<visualization_msgs::Marker>(
    "visualization/waypoints", 1);
  state_points_publisher_ = nh_.advertise<visualization_msgs::Marker>(
    "visualization/state_points", 1);
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

void Visualization::setWaypoints(Eigen::MatrixXd waypoints)
{
  waypoints_ = this->navMsgsPathToVisualizationMsgsMarker(
    this->eigenMatrixXdToNavMsgsPath(waypoints));
}

visualization_msgs::Marker Visualization::getWaypoints()
{
  return waypoints_;
}

void Visualization::publishWaypoints()
{
  waypoints_publisher_.publish(waypoints_);
}

void Visualization::setStatePoints(Eigen::MatrixXd points)
{
  state_points_ = this->navMsgsPathToVisualizationMsgsMarker(
    this->eigenMatrixXdToNavMsgsPath(points));

  state_points_.header.stamp = ros::Time::now();
  state_points_.header.frame_id = "world";
  state_points_.ns = "state_points";
  state_points_.id = 0;
  state_points_.scale.x = state_scale_;
  state_points_.scale.y = state_scale_;
  state_points_.scale.z = state_scale_;
  state_points_.lifetime = ros::Duration(0);
}

visualization_msgs::Marker Visualization::getStatePoints()
{
  return state_points_;
}

void Visualization::publishStatePoints()
{
  state_points_publisher_.publish(state_points_);
}

void Visualization::publishAll()
{
  this->publishPath();
  this->publishTrajectory();
  this->publishWaypoints();
  this->publishStatePoints();
}

void Visualization::clearAll()
{
  path_.poses.clear();
  trajectory_.poses.clear();
  waypoints_.points.clear();
  waypoints_.colors.clear();
  //state_points_.points.clear();
  //state_points_.colors.clear();
}



nav_msgs::Path Visualization::eigenMatrixXdToNavMsgsPath(
  Eigen::MatrixXd eigen_path, bool projection)
{
  nav_msgs::Path path;
  int n_points = eigen_path.rows();
  int n_dofs = eigen_path.cols();

  geometry_msgs::PoseStamped current_pose;

  // If number of degrees of freedom is 2 then project path to z=0 plane.
  if(projection == true || n_dofs <= 2){
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

visualization_msgs::Marker Visualization::navMsgsPathToVisualizationMsgsMarker(
  nav_msgs::Path path)
{
  visualization_msgs::Marker marker;

  // Setup how marker looks.
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "world";
  marker.ns = "waypoints";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = waypoints_scale_;
  marker.scale.y = waypoints_scale_;
  marker.scale.z = waypoints_scale_;
  marker.lifetime = ros::Duration(0);

  geometry_msgs::Point current_point;
  std_msgs::ColorRGBA point_color;
  point_color.r = 0.91;
  point_color.g = 0.43;
  point_color.b = 0.15;
  point_color.a = 1.0;
  for (int i=0; i<path.poses.size(); i++){
    current_point.x = path.poses[i].pose.position.x;
    current_point.y = path.poses[i].pose.position.y;
    current_point.z = path.poses[i].pose.position.z;

    marker.points.push_back(current_point);
    marker.colors.push_back(point_color);
  }

  return marker;
}