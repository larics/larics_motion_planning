#include <larics_motion_planning/ToppraTrajectory.h>

ToppraTrajectory::ToppraTrajectory(string config_filename)
{
  string username = "/home/";
  username = username + getenv("USER") + "/";

  // Set up node handle and create service
  nh_ = ros::NodeHandle();
  generate_trajectory_client_ = nh_.serviceClient<topp_ros::GenerateTrajectory>(
    "generate_toppra_trajectory");

  configureFromFile(username + config_filename);
}

ToppraTrajectory::ToppraTrajectory(Eigen::MatrixXd config_matrix, double sampling_frequency) :
  dynamic_constraints_(config_matrix),
  n_dofs_(dynamic_constraints_.cols()),
  sampling_frequency_(sampling_frequency)
{
  cout << "Initializing TOPP-RA trajectory with matrix config." << endl;

  // Set up node handle and create service
  nh_ = ros::NodeHandle();
  generate_trajectory_client_ = nh_.serviceClient<topp_ros::GenerateTrajectory>(
    "generate_toppra_trajectory");
}

ToppraTrajectory::~ToppraTrajectory()
{

}

bool ToppraTrajectory::configureFromFile(string config_filename)
{
  cout << "Initializing TOPP-RA trajectory from file:" << endl;
  cout << "  " << config_filename << endl;
  // Open yaml file with configuration
  YAML::Node config = YAML::LoadFile(config_filename);
  std::vector<double> velocities, accelerations;

  // Load everything from yaml file
  velocities = config["toppra_trajectory"]["velocities"].as<
    std::vector<double> >();
  accelerations = config["toppra_trajectory"]["accelerations"].as<
    std::vector<double> >();

  if (velocities.size() == accelerations.size()){
    // Set degrees of freedom number
    n_dofs_ = velocities.size();
    // Set up size of dynamic constraints.
    dynamic_constraints_ = Eigen::MatrixXd(2,velocities.size());

    // Fill the dynamic constraints
    for (int i=0; i<velocities.size(); i++)
    {
      dynamic_constraints_(0,i) = velocities[i];
      dynamic_constraints_(1,i) = accelerations[i];
    }

    // Angle flags
    is_angular_ = config["toppra_trajectory"]["is_angular"].as<std::vector<int> >();

    // Set up sampling frequency
    sampling_frequency_ = config["toppra_trajectory"]["sampling_frequency"].as<double>();
  }
  else{
    cout << "Error in config: Velocity and acceleration constraints must have ";
    cout << "the same number of elements." << endl;
    exit(0);
  }
}

bool ToppraTrajectory::generateTrajectory(Eigen::MatrixXd positions)
{
  if (positions.cols() != n_dofs_) return false;
  if (positions.rows() < 2) return false;

  // TODO: account for multiple turns. This will solve the problem if vehicle
  // turns only once but if it turns for multiple circles it will only add 2PI.
  // Check how many times is the difference greater than PI and add angle 
  // accordingly(don't know how much at this point).
  for (int i=0; i<n_dofs_; i++){
    if (is_angular_[i] == 1){
      for (int j=1; j<positions.rows(); j++){
        /*cout << "bf: " << positions(j-1,i) << " " << positions(j,i) << " " << positions(j-1,i) - positions(j,i) << endl;
        if ((positions(j-1,i) - positions(j,i)) > M_PI)  positions(j,i) += 2.0*M_PI;
        else if ((positions(j-1,i) - positions(j,i)) < -M_PI)  positions(j,i) -= 2.0*M_PI;
        cout << "af: " << positions(j-1,i) << " " << positions(j,i) << " " << positions(j-1,i) - positions(j,i) << endl;
        */
        double delta = positions(j-1,i) - positions(j,i);
        //cout << "bf: " << positions(j-1,i) << " " << positions(j,i) << " " << " " << delta << " " << ceil(fabs(delta)/(2.0*M_PI)) << endl;
        if (delta > M_PI)  positions(j,i) += ceil(floor(fabs(delta)/M_PI)/(2.0))*2.0*M_PI;
        else if (delta < -M_PI)  positions(j,i) -= ceil(floor(fabs(delta)/M_PI)/(2.0))*2.0*M_PI;
        //cout << "af: " << positions(j-1,i) << " " << positions(j,i) << " " << " " << delta << " " << ceil(fabs(delta)/(2.0*M_PI)) << endl;
      }
    }
  }

/*  double delta = positions(j-1,i) - positions(j,i);
    cout << positions(j-1,i) << " " << positions(j,i) << " " << " " << delta << " " << ceil(fabs(delta)/(2.0*M_PI)) << endl;
    if (delta > M_PI)  positions(j,i) += ceil(fabs(delta)/(2.0*M_PI))*2.0*M_PI;
    else if (delta < -M_PI)  positions(j,i) -= ceil(fabs(delta)/(2.0*M_PI))*2.0*M_PI;
  */

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
  srv.request.sampling_frequency = sampling_frequency_;

  // Call the service
  bool service_success = generate_trajectory_client_.call(srv);
  if (service_success){
    // Convert the trajectory to eigen matrix.
    sampleTrajectory(srv.response.trajectory);

    for (int i=0; i<n_dofs_; i++){
      if (is_angular_[i] == 1){
        for (int j=1; j<sampled_trajectory_.position.rows(); j++){
          sampled_trajectory_.position(j,i) = atan2(
            sin(sampled_trajectory_.position(j,i)), 
            cos(sampled_trajectory_.position(j,i)));
        }
      }
    }
  }
  else return false;

  return true;
}

Trajectory ToppraTrajectory::getTrajectory()
{
  return sampled_trajectory_;
}

bool ToppraTrajectory::setDynamicConstraints(Eigen::MatrixXd dynamic_constraints)
{
  if ((dynamic_constraints_.rows() == dynamic_constraints.rows()) && 
    (dynamic_constraints_.cols() == dynamic_constraints.cols())){
    dynamic_constraints_ = dynamic_constraints;
  }
  else{
    return false;
  }
  return true;
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
