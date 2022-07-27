#include <larics_motion_planning/TrajectoryInterface.h>
#include <larics_motion_planning/MotionPlanningUtil.h>

TrajectoryInterface::TrajectoryInterface()
{
  
}

TrajectoryInterface::TrajectoryInterface(string config_filename)
{
  cout << "Hello from TrajectoryInterface" << endl;
  cout << "Initializing trajectory interface from file:" << endl;
  cout << "  " << config_filename << endl;

  // Open yaml file with configuration
  YAML::Node config = YAML::LoadFile(motion_util::getUserPrefix() + config_filename);

  // Load everything from yaml file
  // Indexes for constant velocity reparametrization
  if (YAML::Node parameter = config["trajectory_planner"]["constant_velocity_reparametrization"]["axes_index_list"]){
    constant_velocity_reparametrization_indexes_ = 
      config["trajectory_planner"]["constant_velocity_reparametrization"]["axes_index_list"].as<
      std::vector<int> >();
  }
  else {
    string ys = "\033[0;33m";
    string ye = "\033[0m";
    cout << ys << "  [WARNING] Missing field in config" << ye << endl;
    cout << ys << "    trajectory_planner: constant_velocity_reparametrization: axes_index_list" << ye << endl;
  }
  // Velocity constraints for constant velocity reparametrization
  if (YAML::Node parameter = config["trajectory_planner"]["constant_velocity_reparametrization"]["velocity_constraints"]){
    constant_velocity_reparametrization_velocities_ = 
      config["trajectory_planner"]["constant_velocity_reparametrization"]["velocity_constraints"].as<
      std::vector<double> >();
  }
  else {
    string ys = "\033[0;33m";
    string ye = "\033[0m";
    cout << ys << "  [WARNING] Missing field in config" << ye << endl;
    cout << ys << "    trajectory_planner: constant_velocity_reparametrization: velocity_constraints" << ye << endl;
  }

}

bool TrajectoryInterface::constantVelocityReparametrization(
  Trajectory& trajectory, Eigen::MatrixXd waypoints)
{
  // Sanity check. If some index is greater than number of dof, return
  // without changing initial trajectory.
  //cout << waypoints.rows() << " ";
  //cout << waypoints.cols() << endl;
  for (int i=0; i<constant_velocity_reparametrization_indexes_.size(); i++){
    if (constant_velocity_reparametrization_indexes_[i] >=
      trajectory.position.cols()){
      cout << "Linear reparametrization indexes out of range!" << endl;
      return false;
    }
  }

  // Allocate waypoint start indexes in trajectory
  std::vector<int> waypoint_indexes(waypoints.rows());
  // First and last index correspond to the first and last waypoint
  if (waypoint_indexes.size() > 0){
    waypoint_indexes[0] = 0;
    waypoint_indexes[waypoint_indexes.size()-1] = trajectory.position.rows()-1;
  }
  int current_trajectory_index = 1;
  for (int i=1; i<waypoint_indexes.size()-1; i++){
    double delta = 0.0, delta_min = 1000.0;
    for (int j=current_trajectory_index; j<trajectory.position.rows(); j++){
      //cout << "First pass" << endl;
      delta = (waypoints.row(i).transpose() - 
        trajectory.position.row(j).transpose()).norm();
      //cout << delta << endl;
      if (delta < delta_min){
        delta_min = delta;
        waypoint_indexes[i] = j;
        current_trajectory_index = j;
      }
    }
  }
  //for (int i=0; i<waypoint_indexes.size(); i++){
  //  cout << "Waypoint index: " << waypoint_indexes[i] << endl;
  //}

  // Reparametrize all desired axes independently. 
  for (int i=0; i<constant_velocity_reparametrization_indexes_.size(); i++){
    int current_axis = constant_velocity_reparametrization_indexes_[i];
    // 
    for (int j=0; j<waypoint_indexes.size()-1; j++){
      double delta_t = trajectory.time(waypoint_indexes[j+1]) - 
        trajectory.time(waypoint_indexes[j]);
      // The velocity should be the one from the config. If that one is too
      // low, than we calculate the needed velocity. This is the reason why
      // max function is used
      double v = max(constant_velocity_reparametrization_velocities_[i], 
        fabs(waypoints(j+1,current_axis) - waypoints(j,current_axis))/delta_t);

      double dt = trajectory.time(waypoint_indexes[j]+1) - 
        trajectory.time(waypoint_indexes[j]);
      double delta = waypoints(j+1,current_axis) - waypoints(j,current_axis);
      // Number of indexes through which the trajectory will be changed. 
      // Rounding up might cause some trouble if the velocity from the config
      // is too low, but this was not tested.
      int di = ceil(fabs(delta)/(v*dt));
      // Based on number of indexes, calculate how much do we have to move
      // through each index. This will result in linear function.
      double dx;
      if (di == 0) dx = 0;
      else dx = delta/di;

      // The initial value is the waypoint value.
      double value = waypoints(j,current_axis);
      // The idea is not to start at waypoint j and get to the final value as
      // quickly as possible, but to start as close as possible to waypoint j+1
      // and achieve final value at waypoint j+1.
      for (int k=waypoint_indexes[j]+1; k<waypoint_indexes[j+1]-di; k++){
        // Don't move anywhere until we reach the interval where we change
        // the value of position.
        trajectory.position(k,current_axis) = value;
        trajectory.velocity(k,current_axis) = 0.0;
        trajectory.acceleration(k,current_axis) = 0.0;
       }
      for (int k=waypoint_indexes[j+1]-di; k<waypoint_indexes[j+1]; k++){
        // Increase value by dx at each point
        value += dx;
        trajectory.position(k,current_axis) = value;
        trajectory.velocity(k,current_axis) = v;
        trajectory.acceleration(k,current_axis) = 0.0;
      }
    }
  }

  return true;
}
