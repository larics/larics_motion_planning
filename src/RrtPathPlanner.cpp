#include <RrtPathPlanner.h>
#include <ctime>

RrtPathPlanner::RrtPathPlanner(string config_filename, 
  shared_ptr<MapInterface> map)
{
  map_ = map;
}

RrtPathPlanner::~RrtPathPlanner()
{

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
  bounds.setLow(0, -10.0); // bounds for x axis
  bounds.setHigh(0, 3.0);
  bounds.setLow(1, -14.0); // bounds for y axis
  bounds.setHigh(1, 0.0);
  bounds.setLow(2, 0.0); // bounds for z axis
  bounds.setHigh(2, 2.7);

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

  // Set planner to simple setup
  //ss.setPlanner(planner);
  //cout << ss.getPlanner() << endl;

  // Set up start and goal states.
  // TODO check if start and goal are 
  ob::ScopedState<ob::SE3StateSpace> start(state_space);
  start->setXYZ(-3.62, -1.94, 1.0);
  start->rotation().setAxisAngle(0, 0, 0, 1);
  ob::ScopedState<ob::SE3StateSpace> goal(state_space);
  goal->setXYZ(-3.91, -12.5, 1.0);
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
  //*path_geom(dynamic_cast<og::PathGeometric>(pdef->getSolutionPath()));
  //convertOmplPathToEigenMatrix(path);

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
  cout << path_ << endl;
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