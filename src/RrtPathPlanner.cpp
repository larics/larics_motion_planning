#include <RrtPathPlanner.h>
#include <ctime>

void printRrtStarConfig(RrtStarConfig config)
{
  cout << "RRT Star planner configuration: " << endl;

  // Print bounds
  cout << " Bounds |    min    |    max    |" << endl;
  for (int i = 0; i<config.bounds.size(); i++){
    cout << " " <<  setfill('0') << setw(6) << i << " | " << 
      boost::format("%9.2f") % config.bounds[i][0] << " | " <<
      boost::format("%9.2f") % config.bounds[i][1] << " | " << endl;
  }
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
  shared_ptr<MapInterface> map)
{
  map_ = map;
  configFromFile(config_filename);
}

RrtPathPlanner::~RrtPathPlanner()
{

}

bool RrtPathPlanner::configFromFile(string config_filename)
{
  cout << "Configuring from file: " << config_filename << endl;
  // Open yaml file with configuration
  YAML::Node config = YAML::LoadFile(config_filename);

  // Load bounds 
  planner_configuration_.bounds = 
    config["path_planner"]["bounds"].as<std::vector< std::vector<double> > >();
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
  printRrtStarConfig(planner_configuration_);
}

bool RrtPathPlanner::planPath(Eigen::MatrixXd positions)
{
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
  ob::StateSpacePtr state_space(std::make_shared<ob::SE3StateSpace>());

  // Set bounds for spatial axes. Our path planner will not go outside these.
  // TODO Check if there can be arbitrary number of bounds and how can they be
  //      passed to validity checker. This might prove useful for checking 
  //      validity with manipulator attached to UAV.
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, 0.0); // bounds for x axis
  bounds.setHigh(0, 10.0);
  bounds.setLow(1, -10.0); // bounds for y axis
  bounds.setHigh(1, 10.0);
  bounds.setLow(2, 0.0); // bounds for z axis
  bounds.setHigh(2, 2.5);

  // Set bounds
  state_space->as<ob::SE3StateSpace>()->setBounds(bounds);
  // This sets longest segment that does not need to be checked for collisions.
  // The trick is that this is actually a fraction of state space maximum
  // extent. To transfer to metric we can divide our resolution with max extent
  // to get the fraction that corresponds to resolution in meters.
  state_space->as<ob::SE3StateSpace>()->setLongestValidSegmentFraction(
    0.01/state_space->as<ob::SE3StateSpace>()->getMaximumExtent());
  
  // Create simple setup. Here ob::SpaceInformation and ob::ProblemDefinition 
  // are both created internally. But we can override them with our code.
  //og::SimpleSetup ss(state_space);
  // Set validity checker
  //ss.setStateValidityChecker(
  //  boost::bind(&RrtPathPlanner::isStateValid, this, _1));

  // Set up space information
  ob::SpaceInformationPtr si(std::make_shared<ob::SpaceInformation>(
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
  planner->as<og::RRTstar>()->setGoalBias(0.05);
  // Maximum length of a motion added in the tree of motions.
  planner->as<og::RRTstar>()->setRange(0.5);
  // Rewiring scale factor, not sure what it is but default was 1.1
  //planner->as<og::RRTstar>()->setRewireFactor(1.1);
  // Delays collision checking procedures. If set to false this checks
  // collisions between neighbor nodes and tries to find the nearest. If set
  // to true it stops when it has found the first collision free neighbor which
  // in turn reduces computation time. Default was set to true.
  planner->as<og::RRTstar>()->setDelayCC(true);
  // Controls if the tree will be pruned or not. If set to true, pruning(
  // removing a vertex) will occur only if the vertex and all its decendants
  // satisfy the pruning condition. Default is false;
  planner->as<og::RRTstar>()->setTreePruning(false);
  // Prune only if the new solution is X% better than the old solution. 0 will
  // prune after every new solution, 1.0 will never prune. Default is 0.05.
  planner->as<og::RRTstar>()->setPruneThreshold(0.1);
  // Use the measure of the pruned subproblem instead of the measure of the
  // entire problem domain(if it exists). Sounds like it's best to leave that
  // on default value which is false.
  planner->as<og::RRTstar>()->setPrunedMeasure(false);
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
  planner->as<og::RRTstar>()->setKNearest(true);
  // Set the number of attempts to make while performing rejection or informed 
  // sampling. Default is 100
  //planner->as<og::RRTstar>()->setNumSamplingAttempts(100);

  // Set up start and goal states.
  // TODO check if start and goal are 
  ob::ScopedState<ob::SE3StateSpace> start(state_space);
  start->setXYZ(1.57, -8.74, 1.0);
  start->rotation().setAxisAngle(0, 0, 0, 1);
  ob::ScopedState<ob::SE3StateSpace> goal(state_space);
  goal->setXYZ(8.68, 8.24, 1.0);
  goal->rotation().setAxisAngle(0, 0, 0, 1);

  // Set up problem definition. That is basically setting start and goal states
  // to one object.
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));
  pdef->setStartAndGoalStates(start, goal);

  // Tell planner which problem definition we use.
  planner->setProblemDefinition(pdef);
  planner->setup();

  // Solve
  ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);
  ob::PathPtr path = pdef->getSolutionPath();
  og::PathGeometric path_geom(dynamic_cast<const og::PathGeometric&>(
    *pdef->getSolutionPath()));
  convertOmplPathToEigenMatrix(path_geom);
  
  // Try to simplify path
  cout << "Path geometric length: " << path_geom.getStateCount() << endl;
  og::PathSimplifier path_simplifier(si);
  path_simplifier.reduceVertices(path_geom, path_geom.getStateCount()/4, 0);
  cout << "Path geometric length after reduce vertices: " << path_geom.getStateCount() << endl;
  //path_simplifier.shortcutPath(path_geom);
  path_simplifier.smoothBSpline(path_geom, 5);
  cout << "Path geometric length after bspline: " << path_geom.getStateCount() << endl;
  convertOmplPathToEigenMatrix(path_geom);
  // Zajebancija sa stanjima.
  /*
  auto temp_space(std::make_shared<ob::CompoundStateSpace>());
  temp_space->addSubspace(ob::StateSpacePtr(
    make_shared<ob::SE3StateSpace>()),1.0);
  temp_space->addSubspace(ob::StateSpacePtr(
    make_shared<ob::RealVectorStateSpace>(5)), 1.0);
  ob::ScopedState<ob::CompoundStateSpace> temp_state(temp_space);
  //temp_space.printSettings(std::cout);
  */
}

inline void RrtPathPlanner::convertOmplPathToEigenMatrix(og::PathGeometric path)
{
  path_ = Eigen::MatrixXd(path.getStateCount(), 3);
  for (int i=0; i<path.getStateCount(); i++){
    auto point = path.getState(i);
    path_(i,0) = point->as<ob::SE3StateSpace::StateType>()->getX();
    path_(i,1) = point->as<ob::SE3StateSpace::StateType>()->getY();
    path_(i,2) = point->as<ob::SE3StateSpace::StateType>()->getZ();
  }
}

Eigen::MatrixXd RrtPathPlanner::getPath()
{
  return path_;
}

bool RrtPathPlanner::isStateValid(const ob::State *state)
{
  Eigen::VectorXd state_vector(3);
  state_vector(0) = state->as<ob::SE3StateSpace::StateType>()->getX();
  state_vector(1) = state->as<ob::SE3StateSpace::StateType>()->getY();
  state_vector(2) = state->as<ob::SE3StateSpace::StateType>()->getZ();
  return map_->isStateValid(state_vector);
}