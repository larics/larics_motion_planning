#include <larics_motion_planning/RrtPathPlanner.h>
#include <larics_motion_planning/MotionPlanningUtil.h>
#include <ctime>

void printRrtStarConfig(RrtStarConfig config)
{
  cout << "RRT Star planner configuration: " << endl;

  // Print bounds
  cout << " Bounds |    min    |    max    |" << endl;
  /*for (int i = 0; i<config.bounds.size(); i++){
    cout << " " <<  setfill('0') << setw(6) << i << " | " <<
      boost::format("%9.2f") % config.bounds[i][0] << " | " <<
      boost::format("%9.2f") % config.bounds[i][1] << " | " << endl;
  }*/
  cout << endl;

  cout << "       Parameter      | " << "Is used |  Value" << endl;
  cout << "Longest valid segment |    " << config.longest_valid_segment_is_used << "    | " << boost::format("%7.2f") % config.longest_valid_segment_value << endl;
  cout << "       Goal bias      |    " << config.goal_bias_is_used <<             "    | " << boost::format("%7.2f") % config.goal_bias_value << endl;
  cout << "         Range        |    " << config.range_is_used <<                 "    | " << boost::format("%7.2f") % config.range_value << endl;
  cout << "     Rewire factor    |    " << config.rewire_factor_is_used <<         "    | " << boost::format("%7.2f") % config.rewire_factor_value << endl;
  cout << "Delay collision check |    " << config.delay_cc_is_used <<              "    | B" << boost::format("%4d") % config.delay_cc_value << endl;
  cout << "     Tree pruning     |    " << config.tree_pruning_is_used <<          "    | B" << boost::format("%4d") % config.tree_pruning_value << endl;
  cout << "    Prune threshold   |    " << config.prune_threshold_is_used <<       "    | " << boost::format("%7.2f") % config.prune_threshold_value << endl;
  cout << "    Pruned measure    |    " << config.pruned_measure_is_used <<        "    | B" << boost::format("%4d") % config.pruned_measure_value << endl;
  cout << "      K nearest       |    " << config.k_nearest_is_used <<             "    | B" << boost::format("%4d") % config.k_nearest_value << endl;

  cout  << "Solve time: " << config.solve_time << " Is incremental: " <<
    config.solve_time_is_incremental << " Increment: " <<
    config.solve_time_increment << endl;

  cout << "Path simplification: " << endl;
  cout << "Reduce vertices: " << endl;
  cout << "  Is used: " << config.reduce_vertices_is_used << endl;
  cout << "  Max steps: " << config.reduce_vertices_max_steps << endl;
  cout << "  Max empty steps: " << config.reduce_vertices_max_empty_steps << endl;
  cout << "  Range ratio: " << config.reduce_vertices_range_ratio << endl;
  cout << "  Use as fraction: " << config.reduce_vertices_use_as_fraction << endl;
  cout << "Smooth b spline: " << endl;
  cout << "  Is used: " << config.smooth_bspline_is_used << endl;
  cout << "  Max steps: " << config.smooth_bspline_max_steps << endl;
}

RrtPathPlanner::RrtPathPlanner(string config_filename,
  shared_ptr<StateValidityCheckerInterface> validity_checker)
{
  state_validity_checker_ = validity_checker;
  configureFromFile(motion_util::getUserPrefix() + config_filename);
  path_length_ = 0.0;
}

RrtPathPlanner::~RrtPathPlanner()
{

}

bool RrtPathPlanner::configureFromFile(string config_filename)
{
  cout << "Configuring path planner from file: " << endl;
  cout << "  " << config_filename << endl;
  // Open yaml file with configuration
  YAML::Node config = YAML::LoadFile(config_filename);

  // Load spaces configuration
  planner_configuration_.number_of_spaces =
    config["path_planner"]["spaces"]["number"].as<int>();
  planner_configuration_.spaces_dimensions =
    config["path_planner"]["spaces"]["dimensions"].as< std::vector<int> >();
  planner_configuration_.total_dof_number = accumulate(
    planner_configuration_.spaces_dimensions.begin(),
    planner_configuration_.spaces_dimensions.end(), 0);
  planner_configuration_.spaces_weights =
    config["path_planner"]["spaces"]["weights"].as< std::vector<double> >();
  planner_configuration_.spaces_types =
    config["path_planner"]["spaces"]["types"].as< std::vector<string> >();
  planner_configuration_.spaces_bounds =
    config["path_planner"]["spaces"]["bounds"].as< std::vector< std::vector< std::vector<double> > > >();

  planner_configuration_.n_euler = 0;
  planner_configuration_.n_quaternion = 0;
  // Go through all spaces
  for (int i=0; i<planner_configuration_.number_of_spaces; i++){
    if (planner_configuration_.spaces_types[i].compare("RealVector")==0){
      planner_configuration_.n_euler += planner_configuration_.spaces_dimensions[i];
      planner_configuration_.n_quaternion += planner_configuration_.spaces_dimensions[i];
    }
    else if (planner_configuration_.spaces_types[i].compare("SO2")==0){
      planner_configuration_.n_euler += 1;
      planner_configuration_.n_quaternion += 1;
    }
    else if (planner_configuration_.spaces_types[i].compare("SO3")==0){
      planner_configuration_.n_euler += 3; // euler angles
      planner_configuration_.n_quaternion += 4; // quaternion representation
    }
  }

  // Longest valid segment
  planner_configuration_.longest_valid_segment_is_used =
    config["path_planner"]["longest_valid_segment"]["is_used"].as<bool>();
  planner_configuration_.longest_valid_segment_is_metric =
    config["path_planner"]["longest_valid_segment"]["is_metric"].as<bool>();
  planner_configuration_.longest_valid_segment_value =
    config["path_planner"]["longest_valid_segment"]["value"].as<double>();

  // Goal bias
  planner_configuration_.goal_bias_is_used =
    config["path_planner"]["goal_bias"]["is_used"].as<bool>();
  planner_configuration_.goal_bias_value =
    config["path_planner"]["goal_bias"]["value"].as<double>();

  // Range
  planner_configuration_.range_is_used =
    config["path_planner"]["range"]["is_used"].as<bool>();
  planner_configuration_.range_value =
    config["path_planner"]["range"]["value"].as<double>();

  // Rewire factor
  planner_configuration_.rewire_factor_is_used =
    config["path_planner"]["rewire_factor"]["is_used"].as<bool>();
  planner_configuration_.rewire_factor_value =
    config["path_planner"]["rewire_factor"]["value"].as<double>();

  // Delay collision check
  planner_configuration_.delay_cc_is_used =
    config["path_planner"]["delay_cc"]["is_used"].as<bool>();
  planner_configuration_.delay_cc_value =
    config["path_planner"]["delay_cc"]["value"].as<bool>();

  // Tree pruning
  planner_configuration_.tree_pruning_is_used =
    config["path_planner"]["tree_pruning"]["is_used"].as<bool>();
  planner_configuration_.tree_pruning_value =
    config["path_planner"]["tree_pruning"]["value"].as<bool>();

  // Prune threshold
  planner_configuration_.prune_threshold_is_used =
    config["path_planner"]["prune_threshold"]["is_used"].as<bool>();
  planner_configuration_.prune_threshold_value =
    config["path_planner"]["prune_threshold"]["value"].as<double>();

  // Pruned measure
  planner_configuration_.pruned_measure_is_used =
    config["path_planner"]["pruned_measure"]["is_used"].as<bool>();
  planner_configuration_.pruned_measure_value =
    config["path_planner"]["pruned_measure"]["value"].as<bool>();

  // k nearest
  planner_configuration_.k_nearest_is_used =
    config["path_planner"]["k_nearest"]["is_used"].as<bool>();
  planner_configuration_.k_nearest_value =
    config["path_planner"]["k_nearest"]["value"].as<bool>();

  // solve time
  planner_configuration_.solve_time_is_incremental =
    config["path_planner"]["solve_time"]["is_incremental"].as<bool>();
  planner_configuration_.solve_time =
    config["path_planner"]["solve_time"]["time"].as<double>();
  planner_configuration_.solve_time_increment =
    config["path_planner"]["solve_time"]["increment"].as<double>();

  // Reduce vertices
  planner_configuration_.reduce_vertices_is_used =
    config["path_planner"]["path_simplifier"]["reduce_vertices"]["is_used"].as<bool>();
  planner_configuration_.reduce_vertices_max_steps =
    config["path_planner"]["path_simplifier"]["reduce_vertices"]["max_steps"].as<double>();
  planner_configuration_.reduce_vertices_max_empty_steps =
    config["path_planner"]["path_simplifier"]["reduce_vertices"]["max_empty_steps"].as<double>();
  planner_configuration_.reduce_vertices_range_ratio =
    config["path_planner"]["path_simplifier"]["reduce_vertices"]["range_ratio"].as<double>();
  planner_configuration_.reduce_vertices_use_as_fraction =
    config["path_planner"]["path_simplifier"]["reduce_vertices"]["use_as_fraction"].as<bool>();

  // Smooth b-spline
  planner_configuration_.smooth_bspline_is_used =
    config["path_planner"]["path_simplifier"]["smooth_b_spline"]["is_used"].as<bool>();
  planner_configuration_.smooth_bspline_max_steps =
    config["path_planner"]["path_simplifier"]["smooth_b_spline"]["max_steps"].as<double>();
  //printRrtStarConfig(planner_configuration_);

  return true;
}

bool RrtPathPlanner::planPath(Eigen::MatrixXd positions)
{
  // Check if start, goal and size of positions is right.
  bool initial_check = true;
  // If less than two waypoints are provided
  if (positions.rows() < 2) {
    initial_check = false;
    cout << "RrtPathPlanner->planPath: " << endl;
    cout << "  You have to provide at least two waypoints." << endl;
  }
  Eigen::VectorXd temp_state = (positions.block(0, 0, 1, positions.cols())).transpose();
  if (!state_validity_checker_->isStateValid(temp_state)){
    initial_check = false;
    cout << "RrtPathPlanner->planPath: " << endl;
    cout << "  Start point is not valid: " << endl << temp_state << endl;
  }
  int k = 0;
  for (int i=0; i<planner_configuration_.number_of_spaces; i++){
    for (int j=0; j<planner_configuration_.spaces_dimensions[i]; j++){
      if (temp_state(k) < planner_configuration_.spaces_bounds[i][j][0] ||
        temp_state(k) > planner_configuration_.spaces_bounds[i][j][1]){
        initial_check = false;
        cout << "RrtPathPlanner->planPath: " << endl;
        cout << "  Start point is out of bounds: " << endl << temp_state << endl;
      }
      k++;
    }
  }
  temp_state = (positions.block(1, 0, 1, positions.cols())).transpose();
  if (!state_validity_checker_->isStateValid(temp_state)){
    initial_check = false;
    cout << "RrtPathPlanner->planPath: " << endl;
    cout << "  Goal point is not valid: " << endl << temp_state << endl;
  }
  // Check if within bounds.
  k = 0;
  for (int i=0; i<planner_configuration_.number_of_spaces; i++){
    for (int j=0; j<planner_configuration_.spaces_dimensions[i]; j++){
      if (temp_state(k) < planner_configuration_.spaces_bounds[i][j][0] ||
        temp_state(k) > planner_configuration_.spaces_bounds[i][j][1]){
        initial_check = false;
        cout << "RrtPathPlanner->planPath: " << endl;
        cout << "  Goal point is out of bounds: " << endl << temp_state << endl;
      }
      k++;
    }
  }

  if (initial_check == false) return initial_check;

  // First create state space in which we will do planning. The type of this
  // can be auto.
  // TODO Check what exactly happens when this is auto since both types are
  //      shared pointers.
  // ANSWER We can use auto here and it will set type to:
  //        St10shared_ptrIN4ompl4base13SE3StateSpaceEE
  //        If we use the use StateSpacePtr the type is:
  //        St10shared_ptrIN4ompl4base10StateSpaceEE
  //        Both are okay, with latter we will have to use as<ob::Se3StateSpace>
  //        to specify the space in which we are planning.
  //ob::StateSpacePtr state_space(std::make_shared<ob::SE3StateSpace>());
  ob::StateSpacePtr state_space(ob::StateSpacePtr(
      make_shared<ob::CompoundStateSpace>()));
  //cout << typeid(state_space).name() << endl;
  for (int i=0; i<planner_configuration_.number_of_spaces; i++){
    //ob::StateSpacePtr current_space(ob::StateSpacePtr(
    //  make_shared<ob::RealVectorStateSpace>(
    //  planner_configuration_.spaces_dimensions[i])));
    auto current_space = generateSpace(planner_configuration_.spaces_types[i],
      planner_configuration_.spaces_dimensions[i], planner_configuration_.spaces_bounds[i]);
    //cout << typeid(current_space).name() << endl;

    // Set bounds for spatial axes. Our path planner will not go outside these.
    // TODO Check if there can be arbitrary number of bounds and how can they be
    //      passed to validity checker. This might prove useful for checking
    //      validity with manipulator attached to UAV.
    /*ob::RealVectorBounds bounds(planner_configuration_.bounds.size());
    for (int j=0; j<planner_configuration_.bounds.size(); j++){
      bounds.setLow(j, planner_configuration_.bounds[j][0]);
      bounds.setHigh(j, planner_configuration_.bounds[j][1]);
    }*/
    //ob::RealVectorBounds bounds(planner_configuration_.spaces_bounds[i].size());
    //for (int j=0; j<planner_configuration_.spaces_bounds[i].size(); j++){
    //  bounds.setLow(j, planner_configuration_.spaces_bounds[i][j][0]);
    //  bounds.setHigh(j, planner_configuration_.spaces_bounds[i][j][1]);
    //}
    // Set bounds
    //current_space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    state_space->as<ob::CompoundStateSpace>()->addSubspace(
      current_space, planner_configuration_.spaces_weights[i]);
    //state_space->setSubspaceWeight(i, 1.0);
  }
  // Set bounds
  //state_space->as<ob::SE3StateSpace>()->setBounds(bounds);

  // This sets longest segment that does not need to be checked for collisions.
  // The trick is that this is actually a fraction of state space maximum
  // extent. To transfer to metric we can divide our resolution with max extent
  // to get the fraction that corresponds to resolution in meters.
  if (planner_configuration_.longest_valid_segment_is_used){
    if (planner_configuration_.longest_valid_segment_is_metric){
      state_space->setLongestValidSegmentFraction(
        planner_configuration_.longest_valid_segment_value/
        state_space->getMaximumExtent());
    }
    else{
      state_space->setLongestValidSegmentFraction(
        planner_configuration_.longest_valid_segment_value);
    }
  }

  // Create simple setup. Here ob::SpaceInformation and ob::ProblemDefinition
  // are both created internally. But we can override them with our code.
  //og::SimpleSetup ss(state_space);
  // Set validity checker
  //ss.setStateValidityChecker(
  //  boost::bind(&RrtPathPlanner::isStateValid, this, _1));

  // Set up space information
  //ob::SpaceInformationPtr si(std::make_shared<ob::SpaceInformation>(
  //  state_space));
  si = ob::SpaceInformationPtr(std::make_shared<ob::SpaceInformation>(
    state_space));
  //ob::SpaceInformationPtr si = ss.getSpaceInformation();
  si->setStateValidityChecker(
    boost::bind(&RrtPathPlanner::isStateValid, this, _1));
  si->setup();

  // Set up planner based on space information. This can be ob::PlannerPtr or
  // auto. auto will set it to RRTstar while ob::PlannerPtr will be some kind
  // of interface class?
  ob::PlannerPtr planner(std::make_shared<og::RRTstar>(si));
  // Probability the planner will choose the goal state. Leave it as it is.
  if (planner_configuration_.goal_bias_is_used){
    planner->as<og::RRTstar>()->setGoalBias(
      planner_configuration_.goal_bias_value);
  }
  // Maximum length of a motion added in the tree of motions.
  if (planner_configuration_.range_is_used){
    planner->as<og::RRTstar>()->setRange(planner_configuration_.range_value);
  }
  // Rewiring scale factor, not sure what it is but default was 1.1
  if (planner_configuration_.rewire_factor_is_used){
    planner->as<og::RRTstar>()->setRewireFactor(
      planner_configuration_.rewire_factor_value);
  }
  // Delays collision checking procedures. If set to false this checks
  // collisions between neighbor nodes and tries to find the nearest. If set
  // to true it stops when it has found the first collision free neighbor which
  // in turn reduces computation time. Default was set to true.
  if (planner_configuration_.delay_cc_is_used){
    planner->as<og::RRTstar>()->setDelayCC(
      planner_configuration_.delay_cc_value);
  }
  // Controls if the tree will be pruned or not. If set to true, pruning(
  // removing a vertex) will occur only if the vertex and all its decendants
  // satisfy the pruning condition. Default is false;
  if (planner_configuration_.tree_pruning_is_used){
    planner->as<og::RRTstar>()->setTreePruning(
      planner_configuration_.tree_pruning_value);
  }
  // Prune only if the new solution is X% better than the old solution. 0 will
  // prune after every new solution, 1.0 will never prune. Default is 0.05.
  if (planner_configuration_.prune_threshold_is_used){
    planner->as<og::RRTstar>()->setPruneThreshold(
      planner_configuration_.prune_threshold_value);
  }
  // Use the measure of the pruned subproblem instead of the measure of the
  // entire problem domain(if it exists). Sounds like it's best to leave that
  // on default value which is false.
  if (planner_configuration_.pruned_measure_is_used){
    planner->as<og::RRTstar>()->setPrunedMeasure(
      planner_configuration_.pruned_measure_value);
  }
  // COMMENTED PARAMETERS WE LEAVE AT DEFAULT
  // Use direct sampling of the heuristic for the generation of random samples
  // (e.g., x_rand). If a direct sampling method is not defined for the
  // objective, rejection sampling will be used by default.
  // Don't know what exactly that is so leaving it at default which is false.
  //planner->as<og::RRTstar>()->setInformedSampling(false);
  // Controls if heuristic above is used on samples. Default is false.
  //planner->as<og::RRTstar>()->setSampleRejection(false);
  // Controls if heuristic above is used on new states before connection.
  // Default is false
  //planner->as<og::RRTstar>()->setNewStateRejection(false);
  // Controls whether pruning and new-state rejection uses an admissible
  // cost-to-come estimate or not. Default is true.
  //planner->as<og::RRTstar>()->setAdmissibleCostToCome(true);
  // Controls whether samples are returned in ordered by the heuristic. This is
  // accomplished by generating a batch at a time. Default is false.
  //planner->as<og::RRTstar>()->setOrderedSampling(false);
  // Set batch size for ordered sampling. I guess it is not used if the above
  // parameter is false. Default is 1.
  //planner->as<og::RRTstar>()->setBatchSize(1);
  // Focus search is accomplished by turning on InformedSampling, TreePruning
  // and NewStateRejection. Default is false.
  //planner->as<og::RRTstar>()->setFocusSearch(false);
  // Use a k-nearest search for rewiring instead of a r-disc search. Default is
  // true so we use k-nearest search.
  if (planner_configuration_.k_nearest_is_used){
    planner->as<og::RRTstar>()->setKNearest(
      planner_configuration_.k_nearest_value);
  }
  // Set the number of attempts to make while performing rejection or informed
  // sampling. Default is 100
  //planner->as<og::RRTstar>()->setNumSamplingAttempts(100);

  // Set up start and goal states.
  // TODO check if start and goal are
  ob::ScopedState<ob::CompoundStateSpace> start(state_space);
  //start[0] = positions(0,0); start[1] = positions(0,1); start[2] = positions(0,2);
  ob::ScopedState<ob::CompoundStateSpace> goal(state_space);
  //goal[0] = positions(1,0); goal[1] = positions(1,1); goal[2] = positions(1,2);
  //for (int i=0; i<positions.cols(); i++){
  //  start[i] = positions(0, i);
  //  goal[i] = positions(1, i);
  //}
  int nq = 0;
  int ne = 0;
  // First go through all spaces.
  // TODO: TREBAJU DVA BROJACA. JEDAN ZA RPY STATE A DRUGI ZA QUATERNION STATE
  for (int k=0; k<planner_configuration_.spaces_dimensions.size(); k++){
    if (planner_configuration_.spaces_types[k].compare("RealVector")==0 || 
      planner_configuration_.spaces_types[k].compare("SO2")==0){

      for (int l=0; l<planner_configuration_.spaces_dimensions[k]; l++){
        start[nq] = positions(0, ne);
        goal[nq] = positions(1, ne);
        ne++;
        nq++;
      }
    }
    else if (planner_configuration_.spaces_types[k].compare("SO3")==0){
      // Manage start
      double roll=positions(0, ne), pitch=positions(0, ne+1), yaw=positions(0, ne+2);
      double qw = cos(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0) + sin(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
      double qx = sin(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0) - cos(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
      double qy = cos(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0) + sin(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0);
      double qz = cos(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0) - sin(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0);
      start[nq] = qx; start[nq+1] = qy; start[nq+2] = qz; start[nq+3] = qw;

      // Manage goal
      roll=positions(1, ne), pitch=positions(1, ne+1), yaw=positions(1, ne+2);
      qw = cos(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0) + sin(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
      qx = sin(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0) - cos(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
      qy = cos(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0) + sin(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0);
      qz = cos(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0) - sin(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0);
      goal[nq] = qx; goal[nq+1] = qy; goal[nq+2] = qz; goal[nq+3] = qw;
      // SO3 state is represented internaly as quaternion so it will have
      // four data values. Our state has only (roll, pitch, yaw so we advance 
      // by 3)
      ne+=3;
      nq+=4;
    }
  }

  // Set up problem definition. That is basically setting start and goal states
  // to one object.
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));
  pdef->setStartAndGoalStates(start, goal);

  // Tell planner which problem definition we use.
  planner->setProblemDefinition(pdef);
  planner->setup();

  // Solve
  ob::PlannerStatus solved;
  if (planner_configuration_.solve_time_is_incremental == false){
    // If we don't use increment simply solve with provided time to get the
    // path
    solved = planner->ob::Planner::solve(planner_configuration_.solve_time);
  }
  else{
    // In this case start with some small time and increment it if planner
    // does not solve. Terminate if valid path is provided.
    bool plan_flag = true;
    double dt = planner_configuration_.solve_time_increment;
    double t = dt;
    while (plan_flag == true){
      solved = planner->ob::Planner::solve(t);
      t += dt;
      plan_flag = !pdef->hasExactSolution();
      //cout << "Time: " << t << " Exact solution: " << !plan_flag << endl;
    }
  }

  ob::PathPtr path = pdef->getSolutionPath();
  if (!path) return false;
  og::PathGeometric path_geom(dynamic_cast<const og::PathGeometric&>(*path));

  // Try to simplify path
  og::PathSimplifier path_simplifier(si);
  if (planner_configuration_.reduce_vertices_is_used){
    //cout << "Path geometric length: " << path_geom.getStateCount() << endl;
    if (planner_configuration_.reduce_vertices_use_as_fraction){
      path_simplifier.reduceVertices(path_geom,
        path_geom.getStateCount()*planner_configuration_.reduce_vertices_max_steps,
        path_geom.getStateCount()*planner_configuration_.reduce_vertices_max_empty_steps,
        planner_configuration_.reduce_vertices_range_ratio);
    }
    else{
      path_simplifier.reduceVertices(path_geom,
        planner_configuration_.reduce_vertices_max_steps,
        planner_configuration_.reduce_vertices_max_empty_steps,
        planner_configuration_.reduce_vertices_range_ratio);
    }
    //path_simplifier.reduceVertices(path_geom, path_geom.getStateCount()/4, 0, 0.33);
    //cout << "Path geometric length after reduce vertices: " << path_geom.getStateCount() << endl;
  }
  //path_simplifier.shortcutPath(path_geom);
  if (planner_configuration_.smooth_bspline_is_used){
    path_simplifier.smoothBSpline(path_geom, planner_configuration_.smooth_bspline_max_steps);
    //cout << "Path geometric length after bspline: " << path_geom.getStateCount() << endl;
  }
  //path_geom.print(cout);
  convertOmplPathToEigenMatrix(path_geom);
  path_length_ = path_geom.length();
  //cout << "Planning successful" << endl;

  return pdef->hasExactSolution();
}

inline void RrtPathPlanner::convertOmplPathToEigenMatrix(og::PathGeometric path)
{
  path_ = Eigen::MatrixXd(path.getStateCount(), planner_configuration_.n_euler);
  for (int i=0; i<path.getStateCount(); i++){
    auto point = path.getState(i);
    ob::ScopedState<ob::CompoundStateSpace> temp(si);
    temp = point;
    //for (int j=0; j<planner_configuration_.total_dof_number; j++) path_(i,j) = temp[j];

    int j = 0;
    int nq = 0; // Index of space with quaternion
    int ne = 0; // Index of space with euler angle
    // First go through all spaces.
    // TODO: TREBAJU DVA BROJACA. JEDAN ZA RPY(nase) STATE A DRUGI ZA QUATERNION(ompl) STATE
    for (int k=0; k<planner_configuration_.spaces_dimensions.size(); k++){
      //cout << "442" << endl;
      if (planner_configuration_.spaces_types[k].compare("RealVector")==0 || 
        planner_configuration_.spaces_types[k].compare("SO2")==0){
        //cout << "445" << endl;
        for (int l=0; l<planner_configuration_.spaces_dimensions[k]; l++){
          path_(i,ne) = temp[nq];
          ne++;
          nq++;
        }
      }
      else if (planner_configuration_.spaces_types[k].compare("SO3")==0){
        double q1=temp[nq], q2=temp[nq+1], q3=temp[nq+2], q0=temp[nq+3]; 
        path_(i,ne) = atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2)); //roll
        path_(i,ne+1) = asin(2.0*(q0*q2-q3*q1)); //pitch
        path_(i,ne+2) = atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2+q3*q3)); //yaw
        // SO3 state is represented internaly as quaternion so it will have
        // four data values.
        ne+=3;
        nq+=4;
      }
    }
    //cout << "Gotovo punjenje matrice" << endl;
    //cout << temp[11] << " " << path.getState(i)->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(2)->value/*temp->as<ob::CompoundState>()->as<ob::SO2StateSpace>()->value*/ << endl;
    //cout << point->as<ob::RealVectorStateSpace::StateType>()->values[0] << endl;
    //auto point1 = point->as<ob::CompoundState>();
    //auto point2 = point1[0];
    //cout << "point " << point1[0][0].as<ob::RealVectorStateSpace::StateType>(0)->values[0] << endl;
    //path.print(std::cout);
    //exit(0);
    //path_(i,0) = point->as<ob::SE3StateSpace::StateType>()->getX();
    //path_(i,1) = point->as<ob::SE3StateSpace::StateType>()->getY();
    //path_(i,2) = point->as<ob::SE3StateSpace::StateType>()->getZ();
    //path_(i,0) = point->as<ob::RealVectorStateSpace::StateType>()->values[0];
    //path_(i,1) = point->as<ob::RealVectorStateSpace::StateType>()->values[1];
    //path_(i,2) = point->as<ob::RealVectorStateSpace::StateType>()->values[2];
  }
}

Eigen::MatrixXd RrtPathPlanner::getPath()
{
  return path_;
}

double RrtPathPlanner::getPathLength()
{
  return path_length_;
}

bool RrtPathPlanner::isStateValid(const ob::State *state)
{
  Eigen::VectorXd state_vector(planner_configuration_.n_quaternion);
  //state_vector(0) = state->as<ob::SE3StateSpace::StateType>()->getX();
  //state_vector(1) = state->as<ob::SE3StateSpace::StateType>()->getY();
  //state_vector(2) = state->as<ob::SE3StateSpace::StateType>()->getZ();
  ob::ScopedState<ob::CompoundStateSpace> temp(si);
  temp = state;
  //cout << temp[5] << endl;
  //cout << state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] << endl;
  //state_vector(0) = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
  //state_vector(1) = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
  //state_vector(2) = state->as<ob::RealVectorStateSpace::StateType>()->values[2];
  for (int i=0; i<planner_configuration_.n_quaternion; i++) state_vector(i) = temp[i];
  //cout << state_vector << endl;
  //cout << state << endl;
  //exit(0);
  return state_validity_checker_->isStateValid(state_vector);
}

ob::StateSpacePtr RrtPathPlanner::generateSpace(string type, int dimension,
  std::vector< std::vector <double> > space_bounds)
{
  ob::StateSpacePtr space;
  if (type.compare("RealVector") == 0){
    //ob::StateSpacePtr space(ob::StateSpacePtr(
    //  make_shared<ob::RealVectorStateSpace>(dimension)));
    space = ob::StateSpacePtr(make_shared<ob::RealVectorStateSpace>(dimension));

    ob::RealVectorBounds bounds(dimension);
    for (int i=0; i<dimension; i++){
      bounds.setLow(i, space_bounds[i][0]);
      bounds.setHigh(i, space_bounds[i][1]);
      //cout << i << ": " << space_bounds[i][0] << " " << space_bounds[i][1] << endl;
    }

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
  }

  else if (type.compare("SO2") == 0){
    space = ob::StateSpacePtr(make_shared<ob::SO2StateSpace>());
  }

  else if (type.compare("SO3") == 0){
    space = ob::StateSpacePtr(make_shared<ob::SO3StateSpace>());
  }

  return space;
}
