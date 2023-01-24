#include <larics_motion_planning/GlobalPlannerRosInterface.h>

GlobalPlannerRosInterface::GlobalPlannerRosInterface()
{
  ros::NodeHandle nh_private = ros::NodeHandle("~");

  nh_private.param("global_planner_config_file", global_planner_config_file_, 
    string("catkin_ws/src/larics_motion_planning/config/uav_and_wp_manipulator_3r_config.yaml"));
  nh_private.param("rate", rate_, int(10));
  // Model uav namespace is used for executing model trajectory service that
  // is required before applying model corrections
  string model_uav_namespace;
  nh_private.param("model_uav_namespace", model_uav_namespace, 
    string("/model_uav"));
  nh_private.param("model_animation_dt_us", model_animation_dt_, 
    int(10));
  nh_private.param("model_animation_flag", model_animation_flag_,
    bool(false));

  // Get the transform between the uav and manipulator as 6 parameters. The list
  // argument passing does not work with private params, but it should work with
  // global parameters.
  double man_x, man_y, man_z, man_r, man_p, man_yaw;
  nh_private.param("t_uav_man/x", man_x, 0.0); transform_uav_manipulator_.push_back(man_x);
  nh_private.param("t_uav_man/y", man_y, 0.0); transform_uav_manipulator_.push_back(man_y);
  nh_private.param("t_uav_man/z", man_z, 0.0); transform_uav_manipulator_.push_back(man_z);
  nh_private.param("t_uav_man/roll", man_r, 0.0); transform_uav_manipulator_.push_back(man_r);
  nh_private.param("t_uav_man/pitch", man_p, 0.0); transform_uav_manipulator_.push_back(man_p);
  nh_private.param("t_uav_man/yaw", man_yaw, 0.0); transform_uav_manipulator_.push_back(man_yaw);
  nh_private.param("manipulator_dof", manipulator_dof_, 5);
  nh_private.param("model_planning_user_confirmation_flag", model_planning_user_confirmation_, true);
  cout << "[GlobalPlannerRosInterface] Transform: ";
  for (int i=0; i<transform_uav_manipulator_.size(); i++){
    cout << transform_uav_manipulator_[i] << ", ";
  }
  cout << endl << endl;

  // Global planner config
  //global_planner_ = make_shared<GlobalPlanner>(global_planner_config_file_);
  global_planner_ = make_shared<ParabolicAirdropPlanner>(global_planner_config_file_);
  octomapmap_ = dynamic_pointer_cast<OctomapMap>(global_planner_->getMapInterface());
  octomap_sub_ = nh_.subscribe("octomap_binary", 1, &OctomapMap::setOctomapFromRosMessage,
    octomapmap_.get());
  visualization_changed_ = false;

  // Publishers
  // Multi degree of freedom trajectory
  multi_dof_trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "multi_dof_trajectory", 1);
  // Path publisher
  cartesian_path_pub_ = nh_.advertise<nav_msgs::Path>("cartesian_path", 1);
  // Path publisher as a joint trajectory message without velocity and acceleration
  joint_trajectory_path_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
    "path_as_joint_trajectory", 1);
  // Joint trajectory publisher
  bool latch_trajectory;
  nh_private.param("latch_trajectory", latch_trajectory, bool(false));
  joint_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
    "joint_trajectory", 1, latch_trajectory);
  // Parabolic airdrop info vector
  bool latch_info_vector;
  nh_private.param("latch_info_vector", latch_info_vector, bool(false));
  parabolic_airdrop_info_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
    "parabolic_airdrop/info_vector", 1, latch_info_vector);

  // Service for model correction trajectory.
  model_correction_service_server_ = nh_.advertiseService(
    "model_correction_trajectory",
    &GlobalPlannerRosInterface::modelCorrectedTrajectoryCallback, this);
  // Service for multiple manipulators model correction trajectory.
  multiple_manipulators_model_correction_service_server_ = nh_.advertiseService(
    "multiple_manipulators_model_correction_trajectory",
    &GlobalPlannerRosInterface::multipleManipulatorsModelCorrectedTrajectoryCallback,
    this);
  // Service for multiple manipulators object trajectory planning
  multiple_manipulators_object_service_server_ = nh_.advertiseService(
    "multiple_manipulators_object_trajectory",
    &GlobalPlannerRosInterface::multipleManipulatorsObjectTrajectoryCallback,
    this);
  // Service client for executing model trajectory and recording it.
  if (model_uav_namespace.at(0) != '/'){
    model_uav_namespace = "/" + model_uav_namespace;
  }
  execute_trajectory_client_ = 
    nh_.serviceClient<larics_motion_planning::MultiDofTrajectory>(
    model_uav_namespace + "/execute_trajectory"); // /simulate_arducopter
  // Service for planning the cartesian trajectory.
  cartesian_trajectory_server_ = nh_.advertiseService("cartesian_trajectory",
    &GlobalPlannerRosInterface::cartesianTrajectoryCallback, this);
  // Service for planning the multi dof trajectory.
  multi_dof_trajectory_server_ = nh_.advertiseService("multi_dof_trajectory",
    &GlobalPlannerRosInterface::multiDofTrajectoryCallback, this);
  // Service for visualizing arbitrary robot state
  visualize_state_server_ = nh_.advertiseService("visualize_state", 
    &GlobalPlannerRosInterface::visualizeStateCallback, this);
  validity_checker_server_ = nh_.advertiseService("validity_checker", 
    &GlobalPlannerRosInterface::validityCheckerCallback, this);
  // Service for parabolic airdrop trajectory planning.
  parabolic_airdrop_trajectory_server_ = nh_.advertiseService(
    "parabolic_airdrop_trajectory",
    &GlobalPlannerRosInterface::parabolicAirdropTrajectoryCallback, this);
  // Service for saving octomap to file.
  save_octomap_server_ = nh_.advertiseService(
    "save_octomap", &GlobalPlannerRosInterface::saveOctomapCallback, this);
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

bool GlobalPlannerRosInterface::modelCorrectedTrajectoryCallback(
  larics_motion_planning::MultiDofTrajectory::Request &req, 
  larics_motion_planning::MultiDofTrajectory::Response &res)
{
  cout << "Starting the model based trajectory generation." << endl;

  // First plan the initial trajectory by calling the multiDofTrajectoryCallback
  // This is probably not the best way to do it, but it will work.
  bool success = false;
  // Also, don't publish the trajectory, just plan it.
  bool publish_trajectory_temp = req.publish_trajectory;
  req.publish_trajectory = false;
  success = this->multiDofTrajectoryCallback(req, res);
  req.publish_trajectory = publish_trajectory_temp;
  int initial_trajectory_size = res.trajectory.points.size()-1;
  int man_dof = manipulator_dof_;

  Trajectory trajectory;
  if (success == true){

    trajectory = global_planner_->getTrajectory();
    // This is the starting
    Eigen::VectorXd ik_starting_configuration;
    ik_starting_configuration = trajectory.position.block(0, 6, 1, man_dof).transpose();
    cout << ik_starting_configuration << endl;

    cout << "Cols: " << trajectory.position.cols() << " Rows: " << trajectory.position.rows() << endl;
    cout << "Animating initial trajectory" << endl;
    for (int i=0; i<trajectory.position.rows(); i++){
      trajectory.position(i, 3) = -0*trajectory.acceleration(i, 1)/9.81;
      trajectory.position(i, 4) = 0*trajectory.acceleration(i, 0)/9.81;
      visualization_.setStatePoints(
        global_planner_->getRobotStatePoints((trajectory.position.row(i)).transpose()));
      visualization_.publishStatePoints();
      usleep(model_animation_dt_);
    }
    string tempstr;
    cout << "Animated uncompensated trajectory with roll and pitch estimated from compensation." << endl;
    //getline(cin, tempstr);
    //joint_trajectory_pub_.publish(trajectoryToJointTrajectory(trajectory));
    usleep(1000000);

    // Send this trajectory to Gazebo simulation and collect information about
    // roll and pitch
    larics_motion_planning::MultiDofTrajectory service;
    service.request.waypoints = trajectoryToJointTrajectory(trajectory);
    bool execute_trajectory_success;
    execute_trajectory_success = execute_trajectory_client_.call(service);
    cout << "Service call was: " << execute_trajectory_success << endl;
    //cout << service.response.pitch << endl;
    //exit(0);
    trajectory_msgs::JointTrajectory rp_trajectory = service.request.waypoints;
    for (int i=0; i<service.response.pitch.size(); i++){
      if (i >= service.request.waypoints.points.size()){
        rp_trajectory.points.push_back(service.request.waypoints.points[
          service.request.waypoints.points.size()-1]);
      }
      rp_trajectory.points[i].positions[3] = service.response.roll[i];
      rp_trajectory.points[i].positions[4] = service.response.pitch[i];
      //cout << service.response.pitch[i] << endl;
    }
    //trajectory = jointTrajectoryToTrajectory(rp_trajectory);
    cout << "Trajectory rows before service call: " << trajectory.position.rows() << endl;
    trajectory = jointTrajectoryToTrajectory(service.response.trajectory);
    cout << "Trajectory rows after service call: " << trajectory.position.rows() << endl;
    cout << "Points in response: " << service.response.trajectory.points.size() << endl;
    // Compensation part
    // First get fixed transform between uav and manipulator
    Eigen::Affine3d t_b_l0;
    t_b_l0 = Eigen::Affine3d::Identity();
    Eigen::Matrix3d rot_uav_manipulator;
    // This is for arducopter with wp manipulator 3rx - Identity rotation!
    rot_uav_manipulator = Eigen::AngleAxisd(transform_uav_manipulator_[5], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(transform_uav_manipulator_[4],  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(transform_uav_manipulator_[3], Eigen::Vector3d::UnitX());
    // This is for arducopter in simulation
    //rot_uav_manipulator = Eigen::AngleAxisd(3.14159265359, Eigen::Vector3d::UnitZ())
    //  * Eigen::AngleAxisd(0,  Eigen::Vector3d::UnitY())
    //  * Eigen::AngleAxisd(1.57079632679, Eigen::Vector3d::UnitX());
    // This is for NEO
    /*rot_uav_manipulator = Eigen::AngleAxisd(-1.57079632679, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(1.57079632679,  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());*/
    // 0.075 z displacement for arducopter simulation
    // 0.125 z displacement for neo
    // 0.2 z displacement for arducopter with wp manipulator 3rx
    // -0.2 z displacement for arducopter with asap manipulator
    t_b_l0.translate(Eigen::Vector3d(transform_uav_manipulator_[0],
      transform_uav_manipulator_[1], transform_uav_manipulator_[2]));
    t_b_l0.rotate(rot_uav_manipulator);
    shared_ptr<KinematicsInterface> kinematics = global_planner_->getKinematicsInterface();
    // Go through all trajectory points.
    cout << "Applying model corrections to the end-effector." << endl;
    for (int i=0; i<trajectory.position.rows(); i++){
      // Get transform of uav in world frame
      Eigen::Affine3d t_w_b = Eigen::Affine3d::Identity();
      t_w_b.translate(Eigen::Vector3d(trajectory.position(i, 0), 
        trajectory.position(i, 1), trajectory.position(i, 2)));
      Eigen::Matrix3d r_w_b;
      // At this point roll and pitch are 0 since we don't plan for them
      r_w_b = Eigen::AngleAxisd(trajectory.position(i, 5), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(trajectory.position(i, 4)*0.0,  Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(trajectory.position(i, 3)*0.0,  Eigen::Vector3d::UnitX());
      t_w_b.rotate(r_w_b);

      // Transform from l0 to end effector.
      // TODO: Provjeriti ovaj dio ako ne radi.
      Eigen::Affine3d t_l0_ee = kinematics->getEndEffectorTransform(
        (trajectory.position.block(i, 6, 1, man_dof)).transpose());
      // Calculate end effector pose in global coordinate system.
      Eigen::Affine3d t_w_ee = t_w_b*t_b_l0*t_l0_ee;
      // Now we have pose of the end effector that we desire, and it was planned
      // without any knowledge of roll and pitch. The idea is to include roll and
      // pitch now.
      t_w_b = Eigen::Affine3d::Identity();
      t_w_b.translate(Eigen::Vector3d(trajectory.position(i, 0), 
        trajectory.position(i, 1), trajectory.position(i, 2)));

      // Override t_w_b with a point on the planned trajectory that is 
      // closest to the executed point.
      /*t_w_b = Eigen::Affine3d::Identity();
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double hausdorff = 100000.0;
      Trajectory executed_trajectory = jointTrajectoryToTrajectory(
        service.response.executed_trajectory);
      // Search for the closest point
      for (int j=i; j>=0; j--){
        double dx = trajectory.position(j, 0) - executed_trajectory.position(i, 0);
        double dy = trajectory.position(j, 1) - executed_trajectory.position(i, 1);
        double dz = trajectory.position(j, 2) - executed_trajectory.position(i, 2);
        double d = sqrt(dx*dx + dy*dy + dz*dz);

        if (d < hausdorff){
          hausdorff = d;
          x = trajectory.position(j, 0);
          y = trajectory.position(j, 1);
          z = trajectory.position(j, 2);
        }
      }
      cout << "hausdorff: " << hausdorff << endl;
      t_w_b.translate(Eigen::Vector3d(x, y, z));*/

      // At this point roll and pitch are 0 since we don't plan for them
      //double roll = -trajectory.acceleration(i, 1)/9.81;
      //double pitch = trajectory.acceleration(i, 0)/9.81;
      double roll = trajectory.position(i, 3);
      double pitch = trajectory.position(i, 4);
      double dy = t_w_ee.translation().y() - t_w_b.translation().y();
      double dx = t_w_ee.translation().x() - t_w_b.translation().x();
      double yaw = atan2(dy, dx);
      r_w_b = Eigen::AngleAxisd(trajectory.position(i, 5) /*yaw*/, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
      t_w_b.rotate(r_w_b);

      // With new transform we can calculate true end effector position in
      // manipulator base frame. Inverse kinematics works in l0(base manipulator
      // frame).
      t_l0_ee = t_b_l0.inverse()*t_w_b.inverse()*t_w_ee;

      // Inverzna za svaku točku. Dodati funkciju inverzne da prima Affine3d
      // i da vraća VectorXd.
      bool found_ik;
      Eigen::VectorXd ik_solution;
      kinematics->setJointPositions(ik_starting_configuration);
      ik_solution = kinematics->calculateInverseKinematics(t_l0_ee, found_ik);
      ik_starting_configuration = ik_solution;
      if (found_ik == false) cout << i << " " << yaw << endl;
      else{
        trajectory.position.block(i, 6, 1, man_dof) = ik_solution.transpose();
      }
    }
    cout << "Model corrections applied. Animating with model corrections" << endl;
    for (int i=0; i<trajectory.position.rows(); i++){
      //trajectory.position(i, 3) = -trajectory.acceleration(i, 1)/9.81;
      //trajectory.position(i, 4) = trajectory.acceleration(i, 0)/9.81;
      //cout << "171" << endl;
      //cout << (trajectory.position.row(i)).transpose() << endl;
      visualization_.setStatePoints(
        global_planner_->getRobotStatePoints((trajectory.position.row(i)).transpose()));
      //cout << "175" << endl;
      visualization_.publishStatePoints();
      usleep(model_animation_dt_);
    }
    //cout << "Press enter to publish compensated trajectory" << endl;
    //getline(cin, tempstr);
    //joint_trajectory_pub_.publish(trajectoryToJointTrajectory(trajectory));
  }
  else {
    cout << "Something went wrong, model corrections not applied!" << endl;
  }

  cout << "Model correction trajectory planned!" << endl;
  // Service result. Override response trajectory before we plan a new
  // trajectory.
  res.success = success;
  res.trajectory = trajectoryToJointTrajectory(trajectory);

  /* REVERSE TRAJECTORY - not planning it anymore
  // Returning the UAV to the initial position. Add code to press a key to do so.
  trajectory_msgs::JointTrajectory reverse_waypoints;
  // To get reverse waypoints we should revers planned path or the waypoints
  // in service request.
  if (req.plan_path == false){
    for (int i=req.waypoints.points.size()-1; i >= 0; i--){
      reverse_waypoints.points.push_back(req.waypoints.points[i]);
    }
  }
  else {
    // Do nothing for now. 
    // Reverse the RRT* planned path. global_planner_->getPath();
  }
  // Plan the reverse trajectory
  global_planner_->planTrajectory(
    this->jointTrajectoryToEigenWaypoints(reverse_waypoints));
  // And execute the trajectory through service.
  larics_motion_planning::MultiDofTrajectory reverse_trajectory_service;
  reverse_trajectory_service.request.waypoints = trajectoryToJointTrajectory(
    global_planner_->getTrajectory());
  cout << "Reverse trajectory service created" << endl;
  bool reverse_trajectory_success;
  reverse_trajectory_success = execute_trajectory_client_.call(
    reverse_trajectory_service);
  cout << "Reverse trajectory ervice call was: " << reverse_trajectory_success << endl;
  */

  // Append last planned point to resulting trajectory and set velocities and
  // accelerations to zero.
  res.trajectory.points.push_back(
    req.waypoints.points[req.waypoints.points.size()-1]);
  res.trajectory.points[res.trajectory.points.size()-1].velocities = 
    res.trajectory.points[initial_trajectory_size].positions;
  res.trajectory.points[res.trajectory.points.size()-1].accelerations = 
    res.trajectory.points[initial_trajectory_size].positions;
  for (int i=0; i<res.trajectory.points[initial_trajectory_size].positions.size(); i++){
    res.trajectory.points[res.trajectory.points.size()-1].velocities[i] = 0;
    res.trajectory.points[res.trajectory.points.size()-1].accelerations[i] = 0;
  }

  // Publish path and trajectory if requested.
  if (req.publish_trajectory){
    if (model_planning_user_confirmation_ == true){
      cout << "Press enter to publish model corrected trajectory" << endl;
      string tempstr;
      getline(cin, tempstr);
    }
    cout << "Publishing model corrected trajectory." << endl;
    joint_trajectory_pub_.publish(res.trajectory);
  }
  if (req.publish_path){
    // Don't publish path for now
    //cartesian_path_pub_.publish(res.path);
  }
  

  return true;
}

bool GlobalPlannerRosInterface::multipleManipulatorsModelCorrectedTrajectoryCallback(
  larics_motion_planning::MultiDofTrajectory::Request &req, 
  larics_motion_planning::MultiDofTrajectory::Response &res)
{
  cout << "Starting the multiple manipulators model based trajectory generation." << endl;

  // First plan the initial trajectory by calling the multiDofTrajectoryCallback
  // This is probably not the best way to do it, but it will work.
  bool success = false;

  // Also, don't publish the trajectory, just plan it.
  bool publish_trajectory_temp = req.publish_trajectory;
  req.publish_trajectory = false;
  success = this->multiDofTrajectoryCallback(req, res);
  req.publish_trajectory = publish_trajectory_temp;

  Trajectory initial_trajectory;
  Trajectory corrected_trajectory;
  if (success == true){

    initial_trajectory = global_planner_->getTrajectory();

    cout << "Cols: " << initial_trajectory.position.cols() << " Rows: " << initial_trajectory.position.rows() << endl;
    cout << "Animating initial trajectory" << endl;
    for (int i=0; i<initial_trajectory.position.rows(); i++){
      initial_trajectory.position(i, 3) = -0*initial_trajectory.acceleration(i, 1)/9.81;
      initial_trajectory.position(i, 4) = 0*initial_trajectory.acceleration(i, 0)/9.81;
      visualization_.setStatePoints(
        global_planner_->getRobotStatePoints((initial_trajectory.position.row(i)).transpose()));
      visualization_.publishStatePoints();
      usleep(model_animation_dt_);
    }
    cout << "Animated uncompensated trajectory with roll and pitch estimated from compensation." << endl;
    usleep(100000);

    // Send this trajectory to Gazebo simulation and collect information about
    // the executed trajectory.
    larics_motion_planning::MultiDofTrajectory service;
    service.request.waypoints = trajectoryToJointTrajectory(initial_trajectory);
    bool execute_trajectory_success;
    execute_trajectory_success = execute_trajectory_client_.call(service);
    cout << "Service call was: " << execute_trajectory_success << endl;

    // Fill the extended initial trajectory with the last point of the
    // initial trajectory to have data points that correspond to all points in
    // the executed trajectory.
    trajectory_msgs::JointTrajectory extended_initial_trajectory = service.request.waypoints;
    for (int i=0; i<service.response.executed_trajectory.points.size(); i++){
      if (i >= service.request.waypoints.points.size()){
        extended_initial_trajectory.points.push_back(service.request.waypoints.points[
          service.request.waypoints.points.size()-1]);
      }
      //cout << service.response.pitch[i] << endl;
    }
    //trajectory = jointTrajectoryToTrajectory(rp_trajectory);
    cout << "Initial trajectory rows before service call: " << initial_trajectory.position.rows() << endl;
    //initial_trajectory = jointTrajectoryToTrajectory(service.response.trajectory);
    cout << "Extended trajectory rows: " << extended_initial_trajectory.points.size() << endl;
    cout << "Points in executed trajectory response: " << service.response.executed_trajectory.points.size() << endl;
    
    // Correcting the end-effector trajectory.
    Trajectory executed_trajectory;
    /*
    Trajectory ext_init;
    executed_trajectory = jointTrajectoryToTrajectory(service.response.executed_trajectory);
    ext_init = jointTrajectoryToTrajectory(extended_initial_trajectory);
    for (int j=0; j<ext_init.position.rows(); j++){
      executed_trajectory.position.block(j,6,1,5) = ext_init.position.block(
        j,6,1,5);
      executed_trajectory.position.block(j,6+11,1,5) = ext_init.position.block(
        j,6+11,1,5);
    }
    */
    corrected_trajectory = global_planner_->modelCorrectedTrajectory(
      jointTrajectoryToTrajectory(extended_initial_trajectory),
      jointTrajectoryToTrajectory(service.response.executed_trajectory)
      /*ext_init, executed_trajectory*/);

    // Try to do this iteratively. Start from i=1 because the first iteration
    // has already been done above.
    int max_iter = 1;
    for (int i=1; i<max_iter; i++){
      cout << "Starting iteration: " << i << endl;

      // Try planning a new toppra trajectory through corrected trajectory.
      cout << "Replanning TOPP-RA trajectory with corrected trajectory." << endl;
      publish_trajectory_temp = req.publish_trajectory;
      larics_motion_planning::MultiDofTrajectory::Request& iterative_req = req;
      larics_motion_planning::MultiDofTrajectory::Response& iterative_res = res;
      iterative_req.waypoints = trajectoryToJointTrajectory(corrected_trajectory);
      cout << "Corrected trajectory rows: " << corrected_trajectory.position.rows() << endl;
      cout << "Waypoints size: " << iterative_req.waypoints.points.size() << endl;
      iterative_req.publish_trajectory = false;
      success = this->multiDofTrajectoryCallback(iterative_req, iterative_res);
      cout << "Replanned trajectory success: " << success << endl;
      cout << "Replanned trajectory number of points: ";
      cout << iterative_res.trajectory.points.size() << endl;
      req.publish_trajectory = publish_trajectory_temp;
      

      // Create new service request from the current corrected trajectory
      service.request.waypoints = trajectoryToJointTrajectory(corrected_trajectory);
      execute_trajectory_success = execute_trajectory_client_.call(service);
      cout << "Service call was: " << execute_trajectory_success << endl;

      cout << "Initial trajectory rows before service call: " << initial_trajectory.position.rows() << endl;
      //initial_trajectory = jointTrajectoryToTrajectory(service.response.trajectory);
      cout << "Extended trajectory rows: " << extended_initial_trajectory.points.size() << endl;
      cout << "Points in executed trajectory response: " << service.response.executed_trajectory.points.size() << endl;

      // Correcting the end-effector trajectory. Even though extended and
      // executed trajectories are different in size, the output will be
      // the same size as extended_initial_trajectory. This is because the
      // correction method only takes size of the first trajectory into
      // account.
      executed_trajectory = jointTrajectoryToTrajectory(service.response.executed_trajectory);
      for (int j=0; j<corrected_trajectory.position.rows(); j++){
        executed_trajectory.position.block(j,6,1,5) = corrected_trajectory.position.block(
          j,6,1,5);
        executed_trajectory.position.block(j,6+11,1,5) = corrected_trajectory.position.block(
          j,6+11,1,5);
      }
      corrected_trajectory = global_planner_->modelCorrectedTrajectory(
        jointTrajectoryToTrajectory(extended_initial_trajectory),
        /*jointTrajectoryToTrajectory(service.response.executed_trajectory)*/
        executed_trajectory);

      cout << "Corrected trajectory rows: " << corrected_trajectory.position.rows() << endl;
    }

    cout << "Animating corrected trajectory." << endl;
    for (int i=0; i<corrected_trajectory.position.rows(); i++){
      visualization_.setStatePoints(
        global_planner_->getRobotStatePoints((corrected_trajectory.position.row(i)).transpose()));
      visualization_.publishStatePoints();
      usleep(model_animation_dt_);
    }
    cout << "Animated uncompensated trajectory with roll and pitch estimated from compensation." << endl;

  }
  else {
    cout << "Something went wrong, model corrections not applied!" << endl;
  }

  cout << "Model correction trajectory planned!" << endl;
  // Service result. Override response trajectory before we plan a new
  // trajectory.
  res.success = success;
  res.trajectory = trajectoryToJointTrajectory(corrected_trajectory);

  // TODO: Check this!!
  // Append last planned point to resulting trajectory and set velocities and
  // accelerations to zero.
  res.trajectory.points.push_back(
    req.waypoints.points[req.waypoints.points.size()-1]);
  int dof = res.trajectory.points.size()-1;
  res.trajectory.points[dof].velocities = res.trajectory.points[dof].positions;
  res.trajectory.points[dof].accelerations = res.trajectory.points[dof].positions;
  for (int i=0; i<res.trajectory.points[dof].positions.size(); i++){
    res.trajectory.points[dof].velocities[i] = 0;
    res.trajectory.points[dof].accelerations[i] = 0;
  }

  // Set effort because it is required for plot
  for (int i=0; i<res.trajectory.points.size(); i++){
    for (int j=0; j<res.trajectory.points[i].positions.size(); j++){
      res.trajectory.points[i].effort.push_back(0);
    }
  }

  
  // Publish path and trajectory if requested.
  if (req.publish_trajectory){
    if (model_planning_user_confirmation_ == true){
      cout << "Press enter to publish model corrected trajectory" << endl;
      string tempstr;
      getline(cin, tempstr);
    }
    cout << "Publishing model corrected trajectory." << endl;
    joint_trajectory_pub_.publish(res.trajectory);
  }
  if (req.publish_path){
    // Don't publish path for now
    //cartesian_path_pub_.publish(res.path);
  }

  return true;
}

bool GlobalPlannerRosInterface::multipleManipulatorsObjectTrajectoryCallback(
  larics_motion_planning::MultiDofTrajectory::Request &req, 
  larics_motion_planning::MultiDofTrajectory::Response &res)
{
  // The idea for this part of the planner is to generate a path for the
  // manipulated object. Based on that path and the provided kinematics
  // interface, compute each aerial manipulator state and return the final
  // full state path. This is intended to be used with some external node that
  // sends this path to the uav planner.
  cout << "Starting the multiple manipulators object trajectory generation." << endl;

  // If the planner finds a feasible path, this will become true.
  bool success = false;

  // Check the number of waypoints. Exit if there are less than two
  if (req.waypoints.points.size() < 2){
    cout << "At least two points required for generating trajectory." << endl;
    res.success = success;
    return true;
  }

  // Convert the input to Eigen waypoints
  Eigen::MatrixXd waypoints = this->jointTrajectoryToEigenWaypoints(req.waypoints);
  visualization_.setWaypoints(waypoints);

  // This function only plans path, so no need to check the trajectory flags.
  Eigen::MatrixXd object_path;
  if (req.plan_path == true){
    success = global_planner_->planObjectPath(waypoints);
    res.path = this->eigenPathToJointTrajectory(global_planner_->getPath());
    res.path_length = global_planner_->getPathLength();
    visualization_.setPath(global_planner_->getPath());

    object_path = global_planner_->getPath();
    success = true;
  }
  else{
    object_path = waypoints;
    success = true;
  }

  if (model_animation_flag_ == true){
    Trajectory visualization_trajectory;
    bool temp_success = global_planner_->planTrajectory(object_path);
    visualization_trajectory = global_planner_->getTrajectory();
    cout << "Animating object planner path." << endl;
    cout << "Visualization trajectory rows: " << visualization_trajectory.position.rows() << endl;
    for (int i=0; i<visualization_trajectory.position.rows(); i++){
      visualization_.setStatePoints(
        global_planner_->getRobotStatePoints((visualization_trajectory.position.row(i)).transpose()));
      visualization_.publishStatePoints();
      usleep(model_animation_dt_);
    }
  }

  // This part of the code will call the function to get the full state.
  // If the call is here, then we can return something even if the path
  // planning flag is set to false, in which case only waypoints will be
  // forwarded.
  shared_ptr<MultipleManipulatorsKinematics> kinematics;
  kinematics = dynamic_pointer_cast<MultipleManipulatorsKinematics>(
    global_planner_->getKinematicsInterface());

  Eigen::MatrixXd full_state_path = kinematics->getFullSystemStateFromObjectState(
    object_path);
  res.path = this->eigenPathToJointTrajectory(full_state_path);

  // Publish the object path if requested.
  if (req.publish_path == true){
    joint_trajectory_path_pub_.publish(this->eigenPathToJointTrajectory(object_path));
  }

  // This does not publish anything, just returns the path.
  res.success = success;


  cout << "Done with the the multiple manipulators object trajectory generation." << endl;
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
    res.success = success;
    return true;
  }

  // If path or trajectory are to be published, then publish them.
  if (req.publish_trajectory){
    multi_dof_trajectory_pub_.publish(res.trajectory);
  }
  if (req.publish_path){
    cartesian_path_pub_.publish(res.path);
  }

  res.success = success;
  return true;
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
    res.success = success;
    return true;
  }

  //cout << req << endl;

  // Check if user overrides trajectory dynamic constraints set through
  // config file
  if (req.override_dynamic_constraints == true){
    cout << "Override dynamic constraints requested" << endl;
    if ((req.velocity_constraints.size() != req.acceleration_constraints.size()) || 
      ((req.velocity_constraints.size() + req.acceleration_constraints.size()) == 0)){
      cout << "Velocity and acceleration constraints must be of same dimensions != 0" << endl;
      cout << "Override cancelled" << endl;
    }
    else{
      Eigen::MatrixXd constraints(2, req.velocity_constraints.size());
      for (int i=0; i<req.velocity_constraints.size(); i++){
        constraints(0,i) = req.velocity_constraints[i];
        constraints(1,i) = req.acceleration_constraints[i]; 
      }
      // Set new constraints
      shared_ptr<TrajectoryInterface> trajectory_interface = 
        global_planner_->getTrajectoryInterface();
      trajectory_interface->setDynamicConstraints(constraints);
    }
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
    res.success = success;
    return true;
  }

  // If path or trajectory are to be published, then publish them.
  if (req.publish_trajectory){
    joint_trajectory_pub_.publish(res.trajectory);
  }
  if (req.publish_path){
    cartesian_path_pub_.publish(res.path);
  }

  res.success = success;

  // Return default configuration if dynamic constraints were overridden
  if (req.override_dynamic_constraints == true){
    // Simply reconfigure trajectory from file.
    YAML::Node config = YAML::LoadFile(global_planner_config_file_);
    shared_ptr<TrajectoryInterface> trajectory_interface = 
        global_planner_->getTrajectoryInterface();
    string username = "/home/";
    username = username + getenv("USER") + "/";
    trajectory_interface->configureFromFile(username +
      config["global_planner"]["trajectory_config_file"].as<string>());
  }
  return true;
}

bool GlobalPlannerRosInterface::parabolicAirdropTrajectoryCallback(
  larics_motion_planning::ParabolicAirdropTrajectory::Request &req, 
  larics_motion_planning::ParabolicAirdropTrajectory::Response &res)
{
  cout << endl << "Parabolic trajectory service callback." << endl;
  //ParabolicAirdropPlanner airdrop_planner(
  //  "catkin_ws/src/larics_motion_planning/config/uav_only_config_example.yaml");
  //airdrop_planner.setMapInterface(global_planner_->getMapInterface());
  // Generate vectors from poses
  Eigen::VectorXd uav_pose(7);
  uav_pose << req.uav_pose.position.x, req.uav_pose.position.y, 
    req.uav_pose.position.z, req.uav_pose.orientation.x, 
    req.uav_pose.orientation.y, req.uav_pose.orientation.z, 
    req.uav_pose.orientation.w;
  Eigen::VectorXd target_pose(7);
  target_pose << req.target_pose.position.x, req.target_pose.position.y, 
    req.target_pose.position.z, req.target_pose.orientation.x, 
    req.target_pose.orientation.y, req.target_pose.orientation.z, 
    req.target_pose.orientation.w;

  // Begin measuring trajectory planning time
  ros::Time time_start = ros::Time::now();

  if (req.plan_trajectory == false){
    res.success = false;
    cout << "Plan trajectory must be set to true for parabolic trajectory" << endl;
    return true;
  }
  else if (req.use_custom_parabola_params == false){
    if (req.use_custom_psi_params == false){
      res.success = global_planner_->generateParabolicAirdropTrajectory(
        uav_pose, target_pose, req.plan_path);
    }
    else if (req.use_custom_psi_params == true && 
      req.custom_psi_params.size() >= 3){
      cout << "Using custom psi params." << endl;
      // Use those custom params if user provided 3 or more.
      res.success = global_planner_->generateParabolicAirdropTrajectory(
        uav_pose, target_pose, req.plan_path, req.custom_psi_params[0], 
        req.custom_psi_params[1], req.custom_psi_params[2]);
    }
    else{
      cout << "Custom psi params must contain at least 3 elements." << endl;
    }

    // Set up visualization
    visualization_.setStatePoints(global_planner_->getParabola());
    visualization_.publishStatePoints();
    visualization_.setTrajectory(global_planner_->getAirdropTrajectory());
    visualization_changed_ = true;
    //cout << global_planner_->getAirdropTrajectory().position << endl;

    // Set up response
    res.trajectory = this->trajectoryToJointTrajectory(
      global_planner_->getAirdropTrajectory());
    if (req.plan_path == true){
      res.path = this->eigenPathToJointTrajectory(global_planner_->getPath());
    }
  }
  else if (req.use_custom_parabola_params == true){
    // User specifies parabola params, we have to call different function.
    if (req.custom_parabola_params.size() < 5){
      cout << "At least 5 params must be provided [v0, dz, alpha, dx, psi]" << endl;
      res.success = false;
      return true;
    }

    Eigen::VectorXd parabola_params(5);
    parabola_params << req.custom_parabola_params[0], req.custom_parabola_params[1], 
      req.custom_parabola_params[2], req.custom_parabola_params[3], 
      req.custom_parabola_params[4];

    res.success = global_planner_->generateParabolicAirdropTrajectory(
      uav_pose, target_pose, req.plan_path, parabola_params);

    // Set up visualization
    visualization_.setStatePoints(global_planner_->getParabola());
    visualization_.publishStatePoints();
    visualization_.setTrajectory(global_planner_->getAirdropTrajectory());
    visualization_changed_ = true;
    //cout << global_planner_->getAirdropTrajectory().position << endl;

    // Set up response
    res.trajectory = this->trajectoryToJointTrajectory(
      global_planner_->getAirdropTrajectory());
    if (req.plan_path == true){
      res.path = this->eigenPathToJointTrajectory(global_planner_->getPath());
    }
  }

  // End measuring trajectory planning time
  ros::Time time_end = ros::Time::now();

  // Publish parabola info vector
  Eigen::VectorXd info = global_planner_->getInfoVector();
  std_msgs::Float64MultiArray info_array;
  for (int i=0; i<info.size(); i++){
    info_array.data.push_back(info[i]);
  }
  parabolic_airdrop_info_pub_.publish(info_array);

  // Publish trajectory
  if (req.publish_trajectory){
    joint_trajectory_pub_.publish(trajectoryToJointTrajectory(
      global_planner_->getAirdropTrajectory()));
  }
  if (req.publish_path){

  }

  // Calculate trajectory length
  double d = 0.0;
  for (int i=1; i<res.trajectory.points.size(); i++){
    double dx = res.trajectory.points[i].positions[0] - res.trajectory.points[i-1].positions[0];
    double dy = res.trajectory.points[i].positions[1] - res.trajectory.points[i-1].positions[1];
    double dz = res.trajectory.points[i].positions[2] - res.trajectory.points[i-1].positions[2];

    d += sqrt(dx*dx + dy*dy + dz*dz);
  }

  // Print relevant values
  cout << "Planning time: " << (time_end - time_start).toSec() << endl;
  cout << "Trajectory length: " << d << endl;

  cout << "Parabolic trajectory service callback finished." << endl << endl;

  
  return true;
}

bool GlobalPlannerRosInterface::saveOctomapCallback(
  larics_motion_planning::SaveOctomap::Request &req, 
  larics_motion_planning::SaveOctomap::Response &res)
{
  // Here we save the octomap
  string filename = req.filename;
  string file_path = req.file_path;
  if (req.filename.length() == 0) {
    filename = "octomap.binvox.bt";
  }
  // If path was not provided save to home folder
  if (req.file_path.length() == 0){
    file_path = "/home/" + string(getenv("USER")) + "/";
  }
  
  cout << "Saving octomap to file: " << file_path+filename << endl;

  res.success = octomapmap_.get()->saveOctomap(file_path+filename);

  return true;
}

bool GlobalPlannerRosInterface::validityCheckerCallback(
  larics_motion_planning::CheckStateValidity::Request &req, 
  larics_motion_planning::CheckStateValidity::Response &res)
{
  auto temp_trajectory = jointTrajectoryToTrajectory(req.points);
  auto [success, invalid_indices] = global_planner_->collisionCheckWithIndices(temp_trajectory.position);
  res.valid = success;
  res.invalid_indices = invalid_indices;
  return true;
}

bool GlobalPlannerRosInterface::visualizeStateCallback(
  larics_motion_planning::VisualizeState::Request &req, 
  larics_motion_planning::VisualizeState::Response &res)
{
  Eigen::VectorXd state(req.state.data.size());
  for (int i=0; i<req.state.data.size(); i++){
    state(i) = req.state.data[i];
  }
  visualization_.setStatePoints(global_planner_->getRobotStatePoints(state));
  visualization_.publishStatePoints();

  // Some tryout stuff
  /*
  Eigen::Affine3d t_b_l0;
  t_b_l0 = Eigen::Affine3d::Identity();
  Eigen::Matrix3d rot_uav_manipulator;
  // This is for arducopter with wp manipulator 3rx
  rot_uav_manipulator = Eigen::AngleAxisd(3.14159265359, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(0,  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(1.57079632679, Eigen::Vector3d::UnitX());
  t_b_l0.translate(Eigen::Vector3d(0, 0, 0.075));
  t_b_l0.rotate(rot_uav_manipulator);
  // End-effector transform
  Eigen::Affine3d t_w_b = Eigen::Affine3d::Identity();
  t_w_b.translate(Eigen::Vector3d(req.state.data[0], 
    req.state.data[1], req.state.data[2]));
  Eigen::Matrix3d r_w_b;
  // At this point roll and pitch are 0 since we don't plan for them
  r_w_b = Eigen::AngleAxisd(req.state.data[5], Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(req.state.data[4]*0.0,  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(req.state.data[3]*0.0,  Eigen::Vector3d::UnitX());
  t_w_b.rotate(r_w_b);

  // Transform from l0 to end effector.
  int q_size = req.state.data.size() - 6;
  Eigen::VectorXd q(q_size);
  for (int i=6; i<req.state.data.size(); i++){
    q(i-6) = req.state.data[i];
  }
  // TODO: This might not exist in all kinematics. Maybe add getter for the
  // end-effector and just use that pose instead of doing it like this.
  shared_ptr<KinematicsInterface> kinematics = global_planner_->getKinematicsInterface();
  Eigen::Affine3d t_l0_ee = kinematics->getEndEffectorTransform(q);
  // Calculate end effector pose in global coordinate system.
  Eigen::Affine3d t_ee_tool = Eigen::Affine3d::Identity();
  t_ee_tool.translate(Eigen::Vector3d(0.3, 0, 0));
  Eigen::Affine3d t_w_ee = t_w_b*t_b_l0*t_l0_ee*t_ee_tool;
  //cout << t_w_ee.translation() << endl;
  //cout << t_w_ee.rotation() << endl;
  Eigen::Quaterniond quat(t_w_ee.rotation());


  res.end_effector.position.x = t_w_ee.translation().x();
  res.end_effector.position.y = t_w_ee.translation().y();
  res.end_effector.position.z = t_w_ee.translation().z();
  res.end_effector.orientation.x = quat.x();
  res.end_effector.orientation.y = quat.y();
  res.end_effector.orientation.z = quat.z();
  res.end_effector.orientation.w = quat.w();
  */

  cout << req.state.data.size() << endl;
  return true;
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

  joint_trajectory.header.stamp = ros::Time::now();
  for (int j=0; j<n_dofs; j++){
    stringstream ss;
    ss << j;
    joint_trajectory.joint_names.push_back("joint"+ss.str());
  }

  return joint_trajectory;
}

Trajectory GlobalPlannerRosInterface::jointTrajectoryToTrajectory(
  trajectory_msgs::JointTrajectory joint_trajectory)
{
  Trajectory eigen_trajectory;
  eigen_trajectory.position.resize(joint_trajectory.points.size(), 
    joint_trajectory.points[0].positions.size());
  eigen_trajectory.velocity.resize(joint_trajectory.points.size(), 
    joint_trajectory.points[0].positions.size());
  eigen_trajectory.acceleration.resize(joint_trajectory.points.size(), 
    joint_trajectory.points[0].positions.size());
  eigen_trajectory.time.resize(joint_trajectory.points.size());

  // Go through the whole path and set it to
  for (int i=0; i<joint_trajectory.points.size(); i++){
    const auto& joint_trajectory_point = joint_trajectory.points[i];

    for (int j=0; j<joint_trajectory_point.positions.size(); j++){
      eigen_trajectory.position(i,j) = joint_trajectory.points[i].positions[j];
    }

    // Separate appending velocities in case velocity vector size is different
    for (int j=0; j<joint_trajectory_point.velocities.size(); j++)
    {
      eigen_trajectory.velocity(i,j) = joint_trajectory.points[i].velocities[j];
    }

    // Separate appending accelerations in case velocity vector size is different
    for (int j=0; j<joint_trajectory_point.accelerations.size(); j++)
    {
      eigen_trajectory.acceleration(i,j) = joint_trajectory.points[i].accelerations[j];
    }

    if (joint_trajectory_point.velocities.size() != joint_trajectory_point.positions.size() ||
      joint_trajectory_point.accelerations.size() != joint_trajectory_point.positions.size())
    {
      ROS_WARN_THROTTLE(2.0, "[GlobalPlannerRosInterface::jointTrajectoryToTrajectory] "
                             "fields are not the same size! Pos: %d, Vel: %d, Acc: %d.",
                             joint_trajectory_point.positions.size(), 
                             joint_trajectory_point.velocities.size(),
                             joint_trajectory_point.accelerations.size());
    }
    
    eigen_trajectory.time(i) = joint_trajectory.points[i].time_from_start.toSec();
  }

  return eigen_trajectory;
}