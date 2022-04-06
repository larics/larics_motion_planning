#include <larics_motion_planning/ParabolicAirdropPlanner.h>

ParabolicAirdropPlanner::ParabolicAirdropPlanner(
  string config_filename):GlobalPlanner(config_filename)
{
  cout << "Constructor" << endl;
  point_checker_ = make_shared<SimpleStateValidityCheckers>(map_interface_,
    "point", Eigen::VectorXd());

  //dx_.resize(11);
  //dx_ << 2.5, 3.0, 2.0, 3.5, 1.5, 4.0, 1.0, 4.5, 5.0, 5.5, 6.0;
  //v_.resize(11);
  //v_ << 2.5, 3.0, 2.0, 3.5, 1.5, 4.0, 1.0, 4.5, 5.0, 5.5, 6.0;
  //alpha_.resize(10);
  //alpha_ << 20.0, 25.0, 15.0, 30.0, 10.0, 5.0, 0.0, 35.0, 40.0, 45.0;

  string username = "/home/";
  username = username + getenv("USER") + "/";
  size_t found = config_filename.find(username);
  if (found != string::npos) configureParabolicAirdropFromFile(config_filename);
  else configureParabolicAirdropFromFile(username + config_filename);
}

bool ParabolicAirdropPlanner::configureParabolicAirdropFromFile(
  string config_filename)
{
  cout << "Additional configuration of parabolic airdrop params from file: " << endl;
  cout << "  " << config_filename << endl;

  YAML::Node config = YAML::LoadFile(config_filename_);

  dx_ = config["global_planner"]["parabolic_airdrop"]["d"].as<std::vector<double> >();
  v_ = config["global_planner"]["parabolic_airdrop"]["v"].as<std::vector<double> >();
  alpha_ = config["global_planner"]["parabolic_airdrop"]["alpha"].as<std::vector<double> >();

  std::vector<double> stop_vel;
  std::vector<double> stop_acc;
  std::vector<double> drop_vel;
  std::vector<double> drop_acc;
  std::vector<double> intermediate_acc;
  stop_vel = config["global_planner"]["parabolic_airdrop"]["stopping_velocity"].as<std::vector<double> >();
  stop_acc = config["global_planner"]["parabolic_airdrop"]["stopping_acceleration"].as<std::vector<double> >();
  drop_vel = config["global_planner"]["parabolic_airdrop"]["dropoff_velocity"].as<std::vector<double> >();
  drop_acc = config["global_planner"]["parabolic_airdrop"]["dropoff_acceleration"].as<std::vector<double> >();
  intermediate_acc = config["global_planner"]["parabolic_airdrop"]["intermediate_acceleration"].as<std::vector<double> >();
  stopping_trajectory_constraints_.resize(4,2);
  dropoff_trajectory_constraints_.resize(4,2);
  intermediate_acceleration_.resize(4);
  for (int i=0; i<4; i++){
    stopping_trajectory_constraints_(i,0) = stop_vel[i];
    stopping_trajectory_constraints_(i,1) = stop_acc[i];
    dropoff_trajectory_constraints_(i,0) = drop_vel[i];
    dropoff_trajectory_constraints_(i,1) = drop_acc[i];
    intermediate_acceleration_(i) = intermediate_acc[i];
  }

  // Max line integral is used in dropoff spline calculation.
  dropoff_max_line_integral_ = config["global_planner"]["parabolic_airdrop"]["max_line_integral"].as<double>();


  // Horizontal acceleration option
  use_horizontal_stopping_acceleration_ = config["global_planner"]["parabolic_airdrop"]["use_horizontal_stopping_acceleration"].as<bool>();
  horizontal_stopping_acceleration_ = config["global_planner"]["parabolic_airdrop"]["horizontal_stopping_acceleration"].as<double>();
  // Intermediate
  use_horizontal_intermediate_acceleration_ = config["global_planner"]["parabolic_airdrop"]["use_horizontal_intermediate_acceleration"].as<bool>();
  horizontal_intermediate_acceleration_ = config["global_planner"]["parabolic_airdrop"]["horizontal_intermediate_acceleration"].as<double>();
  // Dropoff
  use_horizontal_dropoff_acceleration_ = config["global_planner"]["parabolic_airdrop"]["use_horizontal_dropoff_acceleration"].as<bool>();
  horizontal_dropoff_acceleration_ = config["global_planner"]["parabolic_airdrop"]["horizontal_dropoff_acceleration"].as<double>();
  
  // Parabola config elements.
  psi_increment_ = config["global_planner"]["parabolic_airdrop"]["yaw_increment"].as<double>();
  max_dz_ = config["global_planner"]["parabolic_airdrop"]["max_dz"].as<double>();
  payload_z_offset_ = config["global_planner"]["parabolic_airdrop"]["payload_z_offset"].as<double>();
  spline_sampling_time_ = config["global_planner"]["parabolic_airdrop"]["spline_sampling_time"].as<double>();

  // Configure min and max psi. Will always be 0-360deg but user can override
  // it by calling proper function
  psi_min_ = 0.0;
  psi_max_ = 360.0;

  return true;
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
  Eigen::VectorXd uav_pose, Eigen::VectorXd target_pose, bool plan_path)
{
  parabola_set_points_ = Eigen::MatrixXd(0,3);
  double g=9.81;
  //for (double dx=1.0; dx<=8.0; dx+=0.5){
    //for (double v=0.5; v<=6.0; v+=0.5){
      //for (double alpha=0.0; alpha<=deg2rad(45.0); alpha+=deg2rad(5.0)){
  for (int i=0; i<dx_.size(); i++){
    double dx = dx_[i];
    for (int j=0; j<v_.size(); j++){
      double v = v_[j];
      for (int k=0; k<alpha_.size(); k++){
        double alpha = deg2rad(alpha_[k]);
        // Calculate displacement in z axis and time of impact
        double dz = -tan(alpha)*(dx) + 0.5*g*dx*dx/(v*v*cos(alpha)*cos(alpha));
        double t = dx/(v*cos(alpha));

        if (dz < max_dz_){

          // Construct the parabola. Note that this is not final one, it still
          // has to be transformed to world.
          Eigen::MatrixXd parabola;
          parabola = constructParabola(dx, dz, alpha, v, t, g);

          // Translate and rotate parabola to find suitable one. We do this in
          // a for loop by changing yaw angle.
          for (double psi=deg2rad(psi_min_); psi<=deg2rad(psi_max_);
            psi+=deg2rad(psi_increment_)){

            Eigen::MatrixXd transformed_parabola;
            transformed_parabola = transformParabola(parabola, target_pose,
              psi, dx); 

            // Yaw won't change in stopping trajectory but it still needs to 
            // be planned
            double q0 = uav_pose(6);
            double q3 = uav_pose(5);
            double yaw = atan2(2*q0*q3, 1-2*(q3*q3));
            
            // For horizontal intermediate acceleration, only one number is
            // provided. Based on the release direction, calculate intermediate
            // acceleration for x and y axes that points opposite to release
            // direction
            double intermediate_acc_x, intermediate_acc_y;
            // TODO: Provjeriti ovu promjenu u false, ali mislim da je to bio bug
            if (use_horizontal_intermediate_acceleration_ == false){
              intermediate_acc_x = intermediate_acceleration_(0);
              intermediate_acc_y = intermediate_acceleration_(1);
            }
            else{
              intermediate_acc_x = -horizontal_intermediate_acceleration_*cos(psi);
              intermediate_acc_y = -horizontal_intermediate_acceleration_*sin(psi);
            }

            // Plan stopping trajectory
            // Add payload_offset_z_ to account for payload displacement.
            Eigen::MatrixXd conditions(4, 6);
            conditions << transformed_parabola(0,0), transformed_parabola(0,0), 
              v*cos(alpha)*cos(psi), 0, intermediate_acc_x, 0,
              transformed_parabola(0,1), transformed_parabola(0,1), 
              v*cos(alpha)*sin(psi), 0, intermediate_acc_y, 0,
              transformed_parabola(0,2)+payload_z_offset_, transformed_parabola(0,2)+payload_z_offset_, 
              v*sin(alpha), 0, intermediate_acceleration_(2), 0,
              psi, psi, 0, 0, intermediate_acceleration_(3), 0;
            // Change stopping trajectory constraints if user configured
            // horizontal acceleration
            if (use_horizontal_stopping_acceleration_ == true){
              stopping_trajectory_constraints_(0,1) = fabs(horizontal_stopping_acceleration_*cos(psi));
              if (stopping_trajectory_constraints_(0,1) < 0.1){
                stopping_trajectory_constraints_(0,1) = 0.1;
              }
              // Tu je problem NE MOŽE SINUS ZA CONSTRAINTOVE
              stopping_trajectory_constraints_(1,1) = fabs(horizontal_stopping_acceleration_*sin(psi));
              if (stopping_trajectory_constraints_(1,1) < 0.1){
                stopping_trajectory_constraints_(1,1) = 0.1;
              }
            }
            spline_interpolator_.generateTrajectoryNoSync(conditions, 
              stopping_trajectory_constraints_, spline_sampling_time_);
            Trajectory stopping_trajectory = spline_interpolator_.getTrajectory();
            //cout << stopping_trajectory.acceleration << endl;
            bool valid_flag = true;
            // Check parabola candidate for collision
            valid_flag &= checkParabolaForCollision(transformed_parabola);
            cout << "Parabola validity: " << valid_flag << endl;
            // TODO: remove this later. Override for outdoor kopterworx case.
            valid_flag = true;
            // Check stopping trajectory for collision
            valid_flag &= checkTrajectoryForCollision(stopping_trajectory);
            if (valid_flag == false) continue;

            // Plan collision free trajectory to dropoff point
            // Set up waypoints
            Eigen::MatrixXd waypoints(2, 4);
            // Fill waypoints
            waypoints << uav_pose(0), uav_pose(1), uav_pose(2), yaw, 
              //uav_pose(0), uav_pose(1), transformed_parabola(0, 2)+payload_z_offset_, yaw, 
              transformed_parabola(0, 0), transformed_parabola(0, 1), 
              transformed_parabola(0, 2)+payload_z_offset_, psi;
            // Plan trajectory
            if (plan_path == true) {
              if (this->planPathAndTrajectory(waypoints) == false) continue;
            }
            else {
              if (this->planTrajectory(waypoints) == false) continue;
            }
            Trajectory trajectory = trajectory_interface_->getTrajectory();

            // Plan dropoff spline
            Trajectory dropoff_trajectory;
            int dropoff_index = 0;
            dropoff_trajectory = planDropoffSpline(trajectory, v, 
              alpha, psi, dropoff_index);
            if (dropoff_trajectory.time.size() == 0) continue;
            cout << "Dropoff index: " << dropoff_index << endl;
            
            valid_flag &= checkTrajectoryForCollision(dropoff_trajectory);
            if (valid_flag == false) continue;



            // Now that we have all three trajectories we have to concatenate
            // them
            Trajectory all = concatenateTrajectories(trajectory,
              dropoff_trajectory, dropoff_index);
            cout << "intermediate: " << all.position.rows() << endl;
            //cout << all.position.rows() << endl;
            airdrop_trajectory_ = concatenateTrajectories(all, 
              stopping_trajectory);
            // Add drop point column to the trajectory. It is at the start of 
            // stopping trajectory.
            airdrop_trajectory_ = addDropPointColumn(airdrop_trajectory_, 
              airdrop_trajectory_.position.rows() - 
              stopping_trajectory.position.rows());
            cout << "Best trajectory length: " << dropoff_trajectory.position.rows() << endl;
            cout << "Stop trajectory length: " << stopping_trajectory.position.rows() << endl;
            cout << "Trajectory length: " << trajectory.position.rows() << endl;
            cout << "Airdrop trajectory length: " << airdrop_trajectory_.position.rows() << endl;
            //cout << airdrop_trajectory_.velocity << endl;
            //cout << all.time << endl;

            cout << "Final parabola params: " << endl;
            cout << "  d = " << dx << endl << "  v0 = " << v << endl;
            cout << "  theta = " << alpha << endl << "  psi = " << psi << endl;
            cout << "  z = " << dz << endl << "  t = " << t << endl;

            // Fill out information vector
            info_vector_.conservativeResize(16);
            info_vector_ << dx, dz, v, alpha, psi, t,
              double(airdrop_trajectory_.position.rows())*spline_sampling_time_,
              double(dropoff_trajectory.position.rows())*spline_sampling_time_,
              double(stopping_trajectory.position.rows())*spline_sampling_time_,
              target_pose(0), target_pose(1), target_pose(2),
              transformed_parabola(0, 0), transformed_parabola(0, 1), 
              transformed_parabola(0, 2)+payload_z_offset_, yaw;


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

bool ParabolicAirdropPlanner::generateParabolicAirdropTrajectory(
  Eigen::VectorXd uav_pose, Eigen::VectorXd target_pose, bool plan_path,
  double psi_min, double psi_increment, double psi_max)
{ 
  // Save the old value of increment
  double psi_increment_old = psi_increment_;
  // Override min, max and increment
  psi_min_ = psi_min;
  psi_max_ = psi_max;
  psi_increment_ = psi_increment;

  // Call trajectory planning function
  bool success = this->generateParabolicAirdropTrajectory(uav_pose,
    target_pose, plan_path);

  // Restore overriden values.
  psi_min_ = 0.0;
  psi_max_ = 360.0;
  psi_increment_ = psi_increment_old;

  return success;
}

bool ParabolicAirdropPlanner::generateParabolicAirdropTrajectory(
  Eigen::VectorXd uav_pose, Eigen::VectorXd target_pose, bool plan_path, 
  Eigen::VectorXd parabola_params)
{
  parabola_set_points_ = Eigen::MatrixXd(0,3);
  double g=9.81;
  
  // Parabola params are [v0, dz, alpha, dx, psi]
  double v = parabola_params(0);
  double dz = parabola_params(1);
  double alpha = parabola_params(2);
  double dx = parabola_params(3);
  double psi = parabola_params(4);
  // Time can be calculated
  double t = dx/(v*cos(alpha));

  // Construct parabola in x-z plane
  Eigen::MatrixXd parabola;
  parabola = constructParabola(dx, dz, alpha, v, t, g);
  // Transform parabola to xyz
  Eigen::MatrixXd transformed_parabola;
  transformed_parabola = transformParabola(parabola, target_pose,
    psi, dx);

  // Yaw won't change in stopping trajectory but it still needs to 
  // be planned
  double q0 = uav_pose(6);
  double q3 = uav_pose(5);
  double yaw = atan2(2*q0*q3, 1-2*(q3*q3));

  // For horizontal intermediate acceleration, only one number is
  // provided. Based on the release direction, calculate intermediate
  // acceleration for x and y axes that points opposite to release
  // direction
  double intermediate_acc_x, intermediate_acc_y;
  if (use_horizontal_intermediate_acceleration_ == false){
    intermediate_acc_x = intermediate_acceleration_(0);
    intermediate_acc_y = intermediate_acceleration_(1);
  }
  else{
    intermediate_acc_x = -horizontal_intermediate_acceleration_*cos(psi);
    intermediate_acc_y = -horizontal_intermediate_acceleration_*sin(psi);
  }

  // Plan stopping trajectory
  // Add payload_offset_z_ to account for payload displacement.
  Eigen::MatrixXd conditions(4, 6);
  conditions << transformed_parabola(0,0), transformed_parabola(0,0), 
    v*cos(alpha)*cos(psi), 0, intermediate_acc_x, 0,
    transformed_parabola(0,1), transformed_parabola(0,1), 
    v*cos(alpha)*sin(psi), 0, intermediate_acc_y, 0,
    transformed_parabola(0,2)+payload_z_offset_, transformed_parabola(0,2)+payload_z_offset_, 
    v*sin(alpha), 0, intermediate_acceleration_(2), 0,
    psi, psi, 0, 0, intermediate_acceleration_(3), 0;
  if (use_horizontal_stopping_acceleration_ == true){
    stopping_trajectory_constraints_(0,1) = fabs(horizontal_stopping_acceleration_*cos(psi));
    if (stopping_trajectory_constraints_(0,1) < 0.1){
      stopping_trajectory_constraints_(0,1) = 0.1;
    }
    // Tu je problem NE MOŽE SINUS ZA CONSTRAINTOVE
    stopping_trajectory_constraints_(1,1) = fabs(horizontal_stopping_acceleration_*sin(psi));
    if (stopping_trajectory_constraints_(1,1) < 0.1){
      stopping_trajectory_constraints_(1,1) = 0.1;
    }
  }
  spline_interpolator_.generateTrajectoryNoSync(conditions, 
    stopping_trajectory_constraints_, spline_sampling_time_);
  Trajectory stopping_trajectory = spline_interpolator_.getTrajectory();
  //cout << stopping_trajectory.acceleration << endl;

  bool valid_flag = true;
  // Check parabola candidate for collision
  valid_flag &= checkParabolaForCollision(transformed_parabola);
  // Check stopping trajectory for collision
  valid_flag &= checkTrajectoryForCollision(stopping_trajectory);

  // Plan collision free trajectory to dropoff point
  // Set up waypoints
  Eigen::MatrixXd waypoints(2, 4);
  // Fill waypoints
  waypoints << uav_pose(0), uav_pose(1), uav_pose(2), yaw, 
    //uav_pose(0), uav_pose(1), transformed_parabola(0, 2)+payload_z_offset_, yaw, 
    transformed_parabola(0, 0), transformed_parabola(0, 1), 
    transformed_parabola(0, 2)+payload_z_offset_, psi;
  // Plan trajectory
  if (plan_path == true) {
    this->planPathAndTrajectory(waypoints);
  }
  else {
    this->planTrajectory(waypoints);
  }
  Trajectory trajectory = trajectory_interface_->getTrajectory();
  // Plan dropoff spline
  Trajectory dropoff_trajectory;
  int dropoff_index = 0;
  dropoff_trajectory = planDropoffSpline(trajectory, v, 
    alpha, psi, dropoff_index);
  cout << "Dropoff index: " << dropoff_index << endl;


  // Now that we have all three trajectories we have to concatenate
  // them
  Trajectory all = concatenateTrajectories(trajectory,
    dropoff_trajectory, dropoff_index);
  cout << "intermediate: " << all.position.rows() << endl;
  //cout << all.position.rows() << endl;
  airdrop_trajectory_ = concatenateTrajectories(all, 
    stopping_trajectory);
  // Add drop point column to the trajectory. It is at the start of 
  // stopping trajectory.
  airdrop_trajectory_ = addDropPointColumn(airdrop_trajectory_, 
    airdrop_trajectory_.position.rows() - 
    stopping_trajectory.position.rows());
  cout << "Best trajectory length: " << dropoff_trajectory.position.rows() << endl;
  cout << "Stop trajectory length: " << stopping_trajectory.position.rows() << endl;
  cout << "Trajectory length: " << trajectory.position.rows() << endl;
  cout << "Airdrop trajectory length: " << airdrop_trajectory_.position.rows() << endl;
  //cout << airdrop_trajectory_.velocity << endl;
  //cout << all.time << endl;

  cout << "Final parabola params: " << endl;
  cout << "  d = " << dx << endl << "  v0 = " << v << endl;
  cout << "  theta = " << alpha << endl << "  psi = " << psi << endl;
  cout << "  z = " << dz << endl << "  t = " << t << endl;

  // Fill out information vector
  info_vector_.conservativeResize(16);
  info_vector_ << dx, dz, v, alpha, psi, t,
    double(airdrop_trajectory_.position.rows())*spline_sampling_time_,
    double(dropoff_trajectory.position.rows())*spline_sampling_time_,
    double(stopping_trajectory.position.rows())*spline_sampling_time_,
    target_pose(0), target_pose(1), target_pose(2),
    transformed_parabola(0, 0), transformed_parabola(0, 1), 
    transformed_parabola(0, 2)+payload_z_offset_, yaw;

  return true;
}

Eigen::MatrixXd ParabolicAirdropPlanner::constructParabola(double dx,
  double dz, double alpha, double v, double duration, double g)
{
  double ts = spline_sampling_time_;
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
    if (parabola(i, 2) > 100) cout << "Parabola row: " << endl << parabola.row(i) << endl;
    is_valid &= point_checker_->isStateValid((parabola.row(i)).transpose());
  }

  return is_valid;
}

bool ParabolicAirdropPlanner::checkTrajectoryForCollision(
  Trajectory trajectory)
{
  bool is_valid = true;
  
  for (int i=0; i<trajectory.position.rows(); i++){
    //if (parabola(i, 2) > 100) cout << "Parabola row: " << endl << parabola.row(i) << endl;
    is_valid &= state_validity_checker_interface_->isStateValid(
      (trajectory.position.row(i)).transpose());
  }

  return is_valid;
}

Trajectory ParabolicAirdropPlanner::planDropoffSpline(
  Trajectory trajectory, double v, double alpha, double psi, 
  int &dropoff_index)
{
  cout << "Planning dropoff spline" << endl;
  // Calculate endpoint velocity
  double vx = v*cos(alpha)*cos(psi);
  double vy = v*cos(alpha)*sin(psi);
  double vz = v*sin(alpha);
  // Play around with dropoff constraints
  /*Eigen::MatrixXd dropoff_constraints(4,2);
  dropoff_constraints << dropoff_trajectory_constraints_;
  
  //cout << dropoff_trajectory_constraints_ << endl;
  
  double factor = 1.33;
  dropoff_constraints(0,0) = min(fabs(vx)*factor, dropoff_trajectory_constraints_(0,0));
  dropoff_constraints(1,0) = min(fabs(vy)*factor, dropoff_trajectory_constraints_(1,0));
  dropoff_constraints(2,0) = min(fabs(vz)*factor, dropoff_trajectory_constraints_(2,0));
  //cout << dropoff_constraints << endl;
  dropoff_constraints(0,0) = max(0.5, dropoff_constraints(0,0));
  dropoff_constraints(1,0) = max(0.5, dropoff_constraints(1,0));
  dropoff_constraints(2,0) = max(0.5, dropoff_constraints(2,0));
  //cout << dropoff_constraints << endl;
  //exit(0);*/

  // Account for horizontal intermediate acceleration
  double intermediate_acc_x, intermediate_acc_y;
  if (use_horizontal_intermediate_acceleration_ == false){
    intermediate_acc_x = intermediate_acceleration_(0);
    intermediate_acc_y = intermediate_acceleration_(1);
  }
  else{
    intermediate_acc_x = -horizontal_intermediate_acceleration_*cos(psi);
    intermediate_acc_y = -horizontal_intermediate_acceleration_*sin(psi);
  }

  // If horizontal dropoff acceleration constraint is being used account for it
  if (use_horizontal_dropoff_acceleration_ == true){
    dropoff_trajectory_constraints_(0,1) = fabs(horizontal_dropoff_acceleration_*cos(psi));
    if (dropoff_trajectory_constraints_(0,1) < 0.1){
      dropoff_trajectory_constraints_(0,1) = 0.1;
    }
    dropoff_trajectory_constraints_(1,1) = fabs(horizontal_dropoff_acceleration_*sin(psi));
    if (dropoff_trajectory_constraints_(1,1) < 0.1){
      dropoff_trajectory_constraints_(1,1) = 0.1;
    }
  }

  // Go backwards through the trajectory and find appropriate start point for
  // the dropoff spline.
  double line_integral = 0.5;
  double max_line_integral = dropoff_max_line_integral_;
  double line_integral_increment = 0.5;
  double current_cutoff = 0.0;
  int end = trajectory.position.rows()-1;
  std::vector<Trajectory> spline_list;
  std::vector<int> spline_index;
  for (int i=trajectory.position.rows()-2; i>=0; i--){
    double dx = trajectory.position(i+1, 0) - trajectory.position(i, 0);
    double dy = trajectory.position(i+1, 1) - trajectory.position(i, 1);
    double dz = trajectory.position(i+1, 2) - trajectory.position(i, 2);
    line_integral += sqrt(dx*dx + dy*dy + dz*dz);

    // If line integral from the end is greater than x meters than we can plan
    // the dropoff trajectory. We are incrementing this integral from end until
    // max_line_integral.
    if ((line_integral > current_cutoff) && (line_integral <= max_line_integral)){
      current_cutoff += line_integral_increment;
      double dt = trajectory.time(trajectory.time.size()-1) - trajectory.time(i);
      // Plan dropoff trajectory
      Eigen::MatrixXd conditions(4, 6);
      conditions << trajectory.position(i,0), trajectory.position(end,0), 
        trajectory.velocity(i,0), vx, trajectory.acceleration(i,0), intermediate_acc_x,
        trajectory.position(i,1), trajectory.position(end,1), 
        trajectory.velocity(i,1), vy, trajectory.acceleration(i,1), intermediate_acc_y,
        trajectory.position(i,2), trajectory.position(end,2), 
        trajectory.velocity(i,2), vz, trajectory.acceleration(i,2), intermediate_acceleration_(2),
        trajectory.position(i,3), trajectory.position(end,3), 
        trajectory.velocity(i,3), 0, trajectory.acceleration(i,3), intermediate_acceleration_(3);

      // Generate current spline
      spline_interpolator_.generateTrajectory(conditions, 
        dropoff_trajectory_constraints_, spline_sampling_time_);
      Trajectory dropoff_spline = spline_interpolator_.getTrajectory();
      //if (checkTrajectoryForCollision(dropoff_spline) == true){
      //  dropoff_index = i;
      //  return dropoff_spline;
      //}

      dropoff_index = i;
      spline_list.push_back(dropoff_spline);
      spline_index.push_back(dropoff_index);
      //if (line_integral > 6.75) exit(0);
    }
  }

  int best_trajectory_index = chooseBestTrajectory(spline_list, spline_index, 
    trajectory);
  
  // Set it up for visualization
  parabola_set_points_ = Eigen::MatrixXd(0,3);
  for (int i=0; i<spline_list.size(); i++){
    parabola_set_points_.conservativeResize(parabola_set_points_.rows() +
      spline_list[i].position.rows(), 3);
     parabola_set_points_.block(
      parabola_set_points_.rows() - spline_list[i].position.rows(), 0, 
      spline_list[i].position.rows(), 3) = spline_list[i].position.block(
      0, 0, spline_list[i].position.rows(), 3);
  }
  parabola_set_points_.conservativeResize(parabola_set_points_.rows() +
    trajectory.position.rows(), 3);
  parabola_set_points_.block(
      parabola_set_points_.rows() - trajectory.position.rows(), 0, 
      trajectory.position.rows(), 3) = trajectory.position.block(
      0, 0, trajectory.position.rows(), 3);

  if (best_trajectory_index == -1){
    Trajectory empty_trajectory;
    return empty_trajectory;
  }
  else {
    dropoff_index = spline_index[best_trajectory_index];
    //cout << "Dropoff: " << dropoff_index << endl;
    return spline_list[best_trajectory_index];
  }

  // If there is no spline that is feasible then return empty trajectory.
  Trajectory empty_trajectory;
  return empty_trajectory;
}

int ParabolicAirdropPlanner::chooseBestTrajectory(
  std::vector<Trajectory> spline_list, std::vector<int> spline_index, 
  Trajectory initial_trajectory)
{
  int best_index = -1;
  double min_measure = 10000.0; // something unrealistically big

  // Go through all splines and find the one with 
  for (int i=0; i<spline_index.size(); i++){
    // Consider only trajectories with no collision.
    if (checkTrajectoryForCollision(spline_list[i]) == true){
      double line_integral = trajectoryLineIntegral(spline_list[i]);
      double duration = spline_list[i].time(spline_list[i].time.size()-1);
      double position_rms = calculateTrajectoryRms(spline_list[i], 
        initial_trajectory, "position");
      double velocity_rms = calculateTrajectoryRms(spline_list[i], 
        initial_trajectory, "velocity");
      double acceleration_rms = calculateTrajectoryRms(spline_list[i], 
        initial_trajectory, "acceleration");
      int start_index = spline_index[i];//initial_trajectory.position.rows() - 
        //spline_list[i].position.rows() - 1;
      double trajectory_segment_line_integral = trajectoryLineIntegral(
        initial_trajectory, start_index, -1);
      double ratio = line_integral/trajectory_segment_line_integral;
      //cout << "Spline " << i << endl;
      //cout << "Start index " << start_index << endl;
      //cout << "Traj len " << initial_trajectory.position.rows() << endl;
      //cout << "Spline len " << spline_list[i].position.rows() << endl;
      //cout << "Line integral " << line_integral << endl;
      //cout << "Initial trajectory line integral " << trajectory_segment_line_integral << endl;
      //cout << "Ratio " << ratio << endl;
      //cout << "Duration " << duration << endl;
      //cout << "RMS position " << position_rms << endl;
      //cout << "RMS velocity " << velocity_rms << endl;
      //cout << "RMS acceleration " << acceleration_rms << endl;

      double measure = ratio;//*line_integral;
      //cout << "Line integral is " << i << " " << line_integral << " " << measure << endl;
      //cout << "Trajectory length " << spline_list[i].position.rows() << endl;
      // If this indeed is a better trajectory then set it as such
      if (measure < min_measure){
        min_measure = measure;
        best_index = i;
      }
    }
  }

  return best_index;
}

double ParabolicAirdropPlanner::trajectoryLineIntegral(Trajectory trajectory)
{
  double line_integral = 0.0;
  for (int i=0; i<trajectory.position.rows()-1; i++){
    // Find delta
    double dx = trajectory.position(i,0) - trajectory.position(i+1,0);
    double dy = trajectory.position(i,1) - trajectory.position(i+1,1);
    double dz = trajectory.position(i,2) - trajectory.position(i+1,2);

    line_integral += sqrt(dx*dx + dy*dy + dz*dz);
  }

  return line_integral;
}

double ParabolicAirdropPlanner::trajectoryLineIntegral(Trajectory trajectory,
  int start_index, int end_index)
{
  double line_integral = 0.0;
  if ((end_index < 0) || (end_index > (trajectory.position.rows()-1))){
    end_index = trajectory.position.rows() - 1;
  }
  if ((start_index < 0) || (start_index > end_index)){
    start_index = 0;
  }
  for (int i=start_index; i<end_index; i++){
    // Find delta
    double dx = trajectory.position(i,0) - trajectory.position(i+1,0);
    double dy = trajectory.position(i,1) - trajectory.position(i+1,1);
    double dz = trajectory.position(i,2) - trajectory.position(i+1,2);

    line_integral += sqrt(dx*dx + dy*dy + dz*dz);
  }

  return line_integral;
}

double ParabolicAirdropPlanner::calculateTrajectoryRms(Trajectory spline, 
  Trajectory trajectory, string field)
{
  double rms_squared = 0.0;
  double rms = -1.0;

  if (field.compare("position") == 0){
    int trajectory_index_offset = trajectory.position.rows() - 
      spline.position.rows();
    for (int i=0; i<spline.position.rows(); i++){
      double dx = trajectory.position(i+trajectory_index_offset, 0) - 
        spline.position(i,0);
      double dy = trajectory.position(i+trajectory_index_offset, 1) - 
        spline.position(i,1);
      double dz = trajectory.position(i+trajectory_index_offset, 2) - 
        spline.position(i,2);

      rms_squared += dx*dx + dy*dy*dz*dz;
    }

    rms = sqrt(rms_squared/double(spline.position.rows()));
  }
  else if (field.compare("velocity") == 0){
    int trajectory_index_offset = trajectory.velocity.rows() - 
      spline.velocity.rows();
    for (int i=0; i<spline.velocity.rows(); i++){
      double dx = trajectory.velocity(i+trajectory_index_offset, 0) - 
        spline.velocity(i,0);
      double dy = trajectory.velocity(i+trajectory_index_offset, 1) - 
        spline.velocity(i,1);
      double dz = trajectory.velocity(i+trajectory_index_offset, 2) - 
        spline.velocity(i,2);

      rms_squared += dx*dx + dy*dy*dz*dz;
    }

    rms = sqrt(rms_squared/double(spline.velocity.rows()));
  }
  else if (field.compare("acceleration") == 0){
    int trajectory_index_offset = trajectory.acceleration.rows() - 
      spline.acceleration.rows();
    for (int i=0; i<spline.acceleration.rows(); i++){
      double dx = trajectory.acceleration(i+trajectory_index_offset, 0) - 
        spline.acceleration(i,0);
      double dy = trajectory.acceleration(i+trajectory_index_offset, 1) - 
        spline.acceleration(i,1);
      double dz = trajectory.acceleration(i+trajectory_index_offset, 2) - 
        spline.acceleration(i,2);

      rms_squared += dx*dx + dy*dy*dz*dz;
    }

    rms = sqrt(rms_squared/double(spline.acceleration.rows()));
  }

  return rms;
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
