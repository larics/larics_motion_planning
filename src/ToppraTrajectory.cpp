#include "ToppraTrajectory.h"

ToppraTrajectory::ToppraTrajectory(string config_filename)
{
  cout << "Initializing TOPP-RA trajectory with file config." << endl;
}

ToppraTrajectory::ToppraTrajectory(Eigen::MatrixXd config_matrix) :
  dynamic_constraints_(config_matrix),
  n_dofs_(dynamic_constraints_.cols())
{
  cout << "Initializing TOPP-RA trajectory with matrix config." << endl;
  nh_ = ros::NodeHandle();
  generate_trajectory_client_ = nh_.serviceClient<topp_ros::GenerateTrajectory>(
    "generate_toppra_trajectory");
}

ToppraTrajectory::~ToppraTrajectory()
{
  
}

bool ToppraTrajectory::generateTrajectory(Eigen::MatrixXd positions)
{
  if (positions.cols() != n_dofs_) return false;
  if (positions.rows() < 2) return false;

  // Setup local variables
  int n_waypoints = positions.rows();
  topp_ros::GenerateTrajectory srv;

  for (int i=0; i<n_waypoints; i++){
    // Go through waypoints in order to generate the message
    trajectory_msgs::JointTrajectoryPoint current_waypoint;

    for (int j=0; j<n_dofs_; j++){
      // Add value for each degree of freedom to current waypoint.
      current_waypoint.positions.push_back(positions(i,j));
    }

    // On first waypoint add constraints since the code that generates the
    // trajectory requires dynamic constraints.
    if (i==0){
      for (int j=0; j<n_dofs_; j++){
      current_waypoint.velocities.push_back(dynamic_constraints_(0,j));
      current_waypoint.accelerations.push_back(dynamic_constraints_(1,j));
      }
    }

    srv.request.waypoints.points.push_back(current_waypoint);
  }
  srv.request.sampling_frequency = 100.0;

  // Call the service
  bool service_success = generate_trajectory_client_.call(srv);
  if (service_success){
    // Convert the trajectory to eigen matrix.
    sampleTrajectory(srv.response.trajectory);
  }
  else return false;

  return true;
}

Trajectory ToppraTrajectory::getTrajectory()
{
  return sampled_trajectory_;
}


void ToppraTrajectory::sampleTrajectory(
      trajectory_msgs::JointTrajectory joint_trajectory)
{
  // This is easy since service returns sampled trajectory. This function
  // converts it to Trajectory type

  // First resize the trajectory to accomodate for all the points.
  sampled_trajectory_.position.resize(joint_trajectory.points.size(), 
    n_dofs_);
  sampled_trajectory_.velocity.resize(joint_trajectory.points.size(), 
    n_dofs_);
  sampled_trajectory_.acceleration.resize(joint_trajectory.points.size(), 
    n_dofs_);
  sampled_trajectory_.jerk.resize(joint_trajectory.points.size(), 
    n_dofs_);
  sampled_trajectory_.split.resize(joint_trajectory.points.size(), 
    n_dofs_);
  sampled_trajectory_.time.resize(joint_trajectory.points.size());

  // Next, start packing trajectory
  for (int i=0; i<joint_trajectory.points.size(); i++){
    sampled_trajectory_.time(i) = 
      joint_trajectory.points[i].time_from_start.toSec();
    for (int j=0; j<n_dofs_; j++){
      sampled_trajectory_.position(i,j) = 
        joint_trajectory.points[i].positions[j];
      sampled_trajectory_.velocity(i,j) = 
        joint_trajectory.points[i].velocities[j];
      sampled_trajectory_.acceleration(i,j) = 
        joint_trajectory.points[i].accelerations[j];
    }
  }
}