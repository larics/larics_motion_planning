#include <larics_motion_planning/ParabolicAirdropPlanner.h>

ParabolicAirdropPlanner::ParabolicAirdropPlanner(
  string config_filename):GlobalPlanner(config_filename)
{
  cout << "Constructor" << endl;
  point_checker_ = make_shared<SimpleStateValidityCheckers>(map_interface_,
    "point", Eigen::VectorXd());

  dx_.resize(11);
  dx_ << 2.5, 3.0, 2.0, 3.5, 1.5, 4.0, 1.0, 4.5, 5.0, 5.5, 6.0;
  v_.resize(11);
  v_ << 2.5, 3.0, 2.0, 3.5, 1.5, 4.0, 1.0, 4.5, 5.0, 5.5, 6.0;
  alpha_.resize(10);
  alpha_ << 20.0, 25.0, 15.0, 30.0, 10.0, 5.0, 0.0, 35.0, 40.0, 45.0;
}

void ParabolicAirdropPlanner::setMapInterface(shared_ptr<MapInterface> map)
{
  map_interface_ = map;
  YAML::Node config = YAML::LoadFile(config_filename_);
  // Reset state validity checkers
  if (state_validity_checker_type_ == "ball" ||
    state_validity_checker_type_ == "sphere" ||
    state_validity_checker_type_ == "point" ||
    state_validity_checker_type_ == "circle" ||
    state_validity_checker_type_ == "cylinder" ||
    state_validity_checker_type_ == "rectangle" ||
    state_validity_checker_type_ == "prism"){
    state_validity_checker_interface_ = make_shared<SimpleStateValidityCheckers>(
      config["global_planner"]["state_validity_checker_config_file"].as<string>(),
      map_interface_, state_validity_checker_type_);
    cout << "State validity checker type is: " << state_validity_checker_type_ << endl;
  }
  else if (state_validity_checker_type_ == "uav_and_wp_manipulator"){
    // First set up kinematics for wp manipulator.
    kinematics_interface_ = make_shared<WpManipulatorKinematics>(
      config["global_planner"]["kinematics_config_file"].as<string>());
    // Set up validity checker for uav and wp manipulator
    state_validity_checker_interface_ = make_shared<UavWpManipulatorStateValidityChecker>(
      config["global_planner"]["state_validity_checker_config_file"].as<string>(),
      map_interface_, kinematics_interface_);
    cout << "State validity checker type is: uav_and_wp_manipulator" << endl;
  }
  else{
    cout << "State validity checker type is: " << state_validity_checker_type_ << endl;
    cout << "  This type is not supported!" << endl;
    exit(0);
  }

  // Reset point checker for parabola
  point_checker_ = make_shared<SimpleStateValidityCheckers>(map_interface_,
    "point", Eigen::VectorXd());
}

bool ParabolicAirdropPlanner::generateParabolicAirdropTrajectory(
  Eigen::VectorXd uav_pose, Eigen::VectorXd target_pose)
{
  // Iterate through dx, velocity and alpha to get parabola parameters.
  // TODO: create lists for dx, v and alpha so the search time reduces.
  //   THIS IS IMPORTANT TO DO!

  parabola_set_points_ = Eigen::MatrixXd(0,3);
  double g=9.81;
  //for (double dx=1.0; dx<=8.0; dx+=0.5){
    //for (double v=0.5; v<=6.0; v+=0.5){
      //for (double alpha=0.0; alpha<=deg2rad(45.0); alpha+=deg2rad(5.0)){
  for (int i=0; i<dx_.size(); i++){
    double dx = dx_(i);
    for (int j=0; j<v_.size(); j++){
      double v = v_(j);
      for (int k=0; k<alpha_.size(); k++){
        double alpha = deg2rad(alpha_(k));
        // Calculate displacement in z axis and time of impact
        double dz = -tan(alpha)*(dx) + 0.5*g*dx*dx/(v*v*cos(alpha)*cos(alpha));
        double t = dx/(v*cos(alpha));

        if (dz < 50.0){

          // Construct the parabola. Note that this is not final one, it still
          // has to be transformed to world.
          Eigen::MatrixXd parabola;
          parabola = constructParabola(dx, dz, alpha, v, t, g);

          // Translate and rotate parabola to find suitable one. We do this in
          // a for loop by changing yaw angle.
          for (double parabola_yaw=0.0; parabola_yaw<=deg2rad(360.0);
            parabola_yaw+=deg2rad(22.5)){

            Eigen::MatrixXd transformed_parabola;
            transformed_parabola = transformParabola(parabola, target_pose,
              parabola_yaw, dx);

            // Yaw won't change in stopping trajectory but it still needs to 
            // be planned
            double q0 = uav_pose(6);
            double q3 = uav_pose(5);
            double yaw = atan2(2*q0*q3, 1-2*(q3*q3));
            // Plan stopping trajectory
            Eigen::MatrixXd conditions(4, 6);
            conditions << transformed_parabola(0,0), transformed_parabola(0,0), 
              v*cos(alpha)*cos(parabola_yaw), 0, 0, 0,
              transformed_parabola(0,1), transformed_parabola(0,1), 
              v*cos(alpha)*sin(parabola_yaw), 0, 0, 0,
              transformed_parabola(0,2), transformed_parabola(0,2), 
              v*sin(alpha), 0, 0.3, 0,
              yaw, yaw, 0, 0, 0, 0;
            Eigen::MatrixXd constraints(4, 2);
            constraints << 8, 2.5, 8, 2.5, 5, 2.5, 2, 2;
            spline_interpolator_.generateTrajectory(conditions, constraints, 
              0.01);
            Trajectory stopping_trajectory = spline_interpolator_.getTrajectory();
            //cout << stopping_trajectory.acceleration << endl;

            bool valid_flag = true;
            // Check parabola candidate for collision
            valid_flag &= checkParabolaForCollision(transformed_parabola);
            // Check stopping trajectory for collision
            valid_flag &= checkTrajectoryForCollision(stopping_trajectory);
            if (valid_flag == false) continue;

            // Plan collision free trajectory to dropoff point
            // Set up waypoints
            Eigen::MatrixXd waypoints(2, 4);
            // Fill waypoints
            waypoints << uav_pose(0), uav_pose(1), uav_pose(2), yaw, 
              transformed_parabola(0, 0), transformed_parabola(0, 1), 
              transformed_parabola(0, 2), yaw;
            // Plan trajectory
            if (this->planPathAndTrajectory(waypoints) == false) continue;
            Trajectory trajectory = trajectory_interface_->getTrajectory();

            // Plan dropoff spline
            Trajectory dropoff_trajectory;
            int dropoff_index = 0;
            dropoff_trajectory = planDropoffSpline(trajectory, v, 
              alpha, parabola_yaw, dropoff_index);
            if (dropoff_trajectory.time.size() == 0) continue;


            // Now that we have all three trajectories we have to concatenate
            // them
            Trajectory all = concatenateTrajectories(trajectory,
              dropoff_trajectory, dropoff_index);
            //cout << all.position.rows() << endl;
            airdrop_trajectory_ = concatenateTrajectories(all, 
              stopping_trajectory);
            // Add drop point column to the trajectory. It is at the start of 
            // stopping trajectory.
            airdrop_trajectory_ = addDropPointColumn(airdrop_trajectory_, 
              airdrop_trajectory_.position.rows() - 
              stopping_trajectory.position.rows());
            //cout << airdrop_trajectory_.velocity << endl;
            //cout << all.time << endl;


            return true;


            /*if (v > 3.0){
              parabola_set_points_.conservativeResize(parabola_set_points_.rows() +
                transformed_parabola.rows(), 3);
              parabola_set_points_.block(
                parabola_set_points_.rows() - transformed_parabola.rows(), 0, 
                transformed_parabola.rows(), 3) = transformed_parabola;
            }*/
          }
        }

        // Check collision of that spline part
        // Mozda da probamo naci najbolju parabolu ili set najboljih po nekom
        //   kriteriju. Zbroj dx i dz mozda da bude najmanji? Zbroj*v da je
        //   najmanji?

      }
    }
  }

  return false;
}

Eigen::MatrixXd ParabolicAirdropPlanner::constructParabola(double dx,
  double dz, double alpha, double v, double duration, double g)
{
  double ts = 0.01;
  int n = round(duration/ts);
  double t = 0.0;
  Eigen::MatrixXd parabola(n+1, 3);

  for (int i=0; i<=n; i++){
    parabola(i, 0) = v*cos(alpha)*t;
    parabola(i, 1) = 0.0;
    parabola(i, 2) = v*sin(alpha)*t - 0.5*g*t*t + dz;
    t+=ts;
  }

  return parabola;
}

Eigen::MatrixXd ParabolicAirdropPlanner::transformParabola(Eigen::MatrixXd parabola,
  Eigen::VectorXd target, double yaw, double dp)
{
  Eigen::MatrixXd transformed_parabola(parabola.rows(), parabola.cols());

  double cy = cos(yaw);
  double sy = sin(yaw);
  for (int i=0; i<parabola.rows(); i++){
    transformed_parabola(i, 0) = target(0) - dp*cy + parabola(i, 0)*cy - parabola(i, 1)*sy;
    transformed_parabola(i, 1) = target(1) - dp*sy + parabola(i, 0)*sy + parabola(i, 1)*cy;
    transformed_parabola(i, 2) = target(2) + parabola(i, 2);
  }

  return transformed_parabola;
}

bool ParabolicAirdropPlanner::checkParabolaForCollision(
  Eigen::MatrixXd parabola)
{
  bool is_valid = true;
  
  for (int i=0; i<parabola.rows(); i++){
    if (parabola(i, 2) > 100) cout << parabola.row(i) << endl;
    is_valid &= point_checker_->isStateValid((parabola.row(i)).transpose());
  }

  return is_valid;
}

bool ParabolicAirdropPlanner::checkTrajectoryForCollision(
  Trajectory trajectory)
{
  // TODO: implement this function, we don't need it know because we will not
  // work in cluttered environments.
  return true;
}

Trajectory ParabolicAirdropPlanner::planDropoffSpline(
  Trajectory trajectory, double v, double alpha, double parabola_yaw, 
  int &dropoff_index)
{
  cout << "Planning dropoff spline" << endl;
  // Calculate endpoint velocity
  double vx = v*cos(alpha)*cos(parabola_yaw);
  double vy = v*cos(alpha)*sin(parabola_yaw);
  double vz = v*sin(alpha);

  // Go backwards through the trajectory and find appropriate start point for
  // the dropoff spline.
  double length = 0.0;
  int end = trajectory.position.rows()-1;
  for (int i=trajectory.position.rows()-2; i>=0; i--){
    double dx = trajectory.position(i+1, 0) - trajectory.position(i, 0);
    double dy = trajectory.position(i+1, 1) - trajectory.position(i, 1);
    double dz = trajectory.position(i+1, 2) - trajectory.position(i, 2);
    length += sqrt(dx*dx + dy*dy + dz*dz);

    // If line integral from the end is greater than x meters than we can plan
    // the dropoff trajectory.
    if (length > 2.0){
      double dt = trajectory.time(trajectory.time.size()-1) - trajectory.time(i);
      // Plan dropoff trajectory
      Eigen::MatrixXd conditions(4, 6);
      conditions << trajectory.position(i,0), trajectory.position(end,0), 
        trajectory.velocity(i,0), vx, trajectory.acceleration(i,0), 0,
        trajectory.position(i,1), trajectory.position(end,1), 
        trajectory.velocity(i,1), vy, trajectory.acceleration(i,1), 0,
        trajectory.position(i,2), trajectory.position(end,2), 
        trajectory.velocity(i,2), vz, trajectory.acceleration(i,2), 0.3,
        trajectory.position(i,3), trajectory.position(end,3), 
        trajectory.velocity(i,3), 0, trajectory.acceleration(i,3), 0;

      Eigen::MatrixXd constraints(4, 2);
      constraints << 8, 2.5, 8, 2.5, 5, 2.5, 2, 2;
      spline_interpolator_.generateTrajectory(conditions, constraints, 
        0.01);
      Trajectory dropoff_spline = spline_interpolator_.getTrajectory();
      if (checkTrajectoryForCollision(dropoff_spline) == true){
        dropoff_index = i;
        return dropoff_spline;
      }
    }
  }

  // If there is no spline that is feasible then return empty trajectory.
  Trajectory empty_trajectory;
  return empty_trajectory;
}

Trajectory ParabolicAirdropPlanner::concatenateTrajectories(Trajectory first,
  Trajectory second, int index)
{
  Trajectory concatenated;
  // We want to extract the first trajectory up to penultimate point. 
  // The last point of first trajectory is the same as first point of second
  // trajectory, and the second one we will use from start.
  if (index == -1) index = first.time.size()-2;

  // Resizing the trajectory to appropriate size. Note that index is size up to
  // penultimate point.
  concatenated.position.resize(index + second.time.size(), first.position.cols());
  concatenated.velocity.resize(index + second.time.size(), first.velocity.cols());
  concatenated.acceleration.resize(index + second.time.size(), first.acceleration.cols());
  concatenated.time.resize(index + second.time.size());

  // Filling in the full trajectory.
  // First we put all the elements of the first trajectory except the
  // penultimate one.
  concatenated.position.block(0, 0, index, first.position.cols()) = first.position.block(
    0, 0, index, first.position.cols());
  concatenated.velocity.block(0, 0, index, first.velocity.cols()) = first.velocity.block(
    0, 0, index, first.velocity.cols());
  concatenated.acceleration.block(0, 0, index, first.acceleration.cols()) = first.acceleration.block(
    0, 0, index, first.acceleration.cols());
  concatenated.time.block(0, 0, index, 1) = first.time.block(0, 0, index, 1);

  // Second we put in the whole second trajectory.
  concatenated.position.block(index, 0, second.position.rows(), second.position.cols()) = 
    second.position.block(0, 0, second.position.rows(), second.position.cols());
  concatenated.velocity.block(index, 0, second.velocity.rows(), second.velocity.cols()) = 
    second.velocity.block(0, 0, second.velocity.rows(), second.velocity.cols());
  concatenated.acceleration.block(index, 0, second.acceleration.rows(), second.acceleration.cols()) = 
    second.acceleration.block(0, 0, second.acceleration.rows(), second.acceleration.cols());
  second.time += Eigen::VectorXd::Constant(second.time.size(), first.time(index));
  concatenated.time.block(index, 0, second.time.size(), 1) = second.time;

  return concatenated;
}

Trajectory ParabolicAirdropPlanner::addDropPointColumn(Trajectory trajectory,
  int index)
{
  trajectory.position.conservativeResize(trajectory.position.rows(), 
    trajectory.position.cols()+1);
  trajectory.velocity.conservativeResize(trajectory.velocity.rows(), 
    trajectory.velocity.cols()+1);
  trajectory.acceleration.conservativeResize(trajectory.acceleration.rows(), 
    trajectory.acceleration.cols()+1);

  trajectory.position.block(0, trajectory.position.cols()-1, index, 1) = 
    Eigen::VectorXd::Constant(index-1, 0);
  trajectory.position.block(index, trajectory.position.cols()-1, trajectory.position.rows()-index, 1) = 
    Eigen::VectorXd::Constant(trajectory.position.rows()-index, 1);

  trajectory.velocity.block(0, trajectory.velocity.cols()-1, 
    trajectory.velocity.rows(), 1) = trajectory.position.col(trajectory.position.cols()-1);
  trajectory.acceleration.block(0, trajectory.acceleration.cols()-1, 
    trajectory.acceleration.rows(), 1) = trajectory.position.col(trajectory.position.cols()-1);

  return trajectory;
}

inline double deg2rad(double deg)
{
  return deg*M_PI/180.0;
}
