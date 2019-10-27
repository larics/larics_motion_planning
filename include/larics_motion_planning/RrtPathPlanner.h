/// \file RrtPathPlanner.h

#ifndef RRT_PATH_PLANNER
#define RRT_PATH_PLANNER

#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <larics_motion_planning/PathPlanningInterface.h>
#include <larics_motion_planning/StateValidityCheckerInterface.h>

#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <Eigen/Eigen>

#include "yaml-cpp/yaml.h"

using namespace std;

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/OptimizationObjective.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef struct{
  int number_of_spaces;
  std::vector<int> spaces_dimensions;
  int total_dof_number;
  int n_euler;
  int n_quaternion;
  std::vector<double> spaces_weights;
  std::vector<string> spaces_types;
  std::vector< std::vector< std::vector<double> > > spaces_bounds;

  //std::vector< std::vector<double> > bounds;
  bool longest_valid_segment_is_used;
  bool longest_valid_segment_is_metric;
  double longest_valid_segment_value;
  
  bool goal_bias_is_used;
  double goal_bias_value;
  
  bool range_is_used;
  double range_value;
  
  bool rewire_factor_is_used;
  double rewire_factor_value;
  
  bool delay_cc_is_used;
  bool delay_cc_value;

  bool tree_pruning_is_used;
  bool tree_pruning_value;

  bool prune_threshold_is_used;
  double prune_threshold_value;

  bool pruned_measure_is_used;
  bool pruned_measure_value;

  bool k_nearest_is_used;
  bool k_nearest_value;

  bool solve_time_is_incremental;
  double solve_time;
  double solve_time_increment;

  bool reduce_vertices_is_used;
  double reduce_vertices_max_steps;
  double reduce_vertices_max_empty_steps;
  double reduce_vertices_range_ratio;
  bool reduce_vertices_use_as_fraction;

  bool smooth_bspline_is_used;
  double smooth_bspline_max_steps;
} RrtStarConfig;

void printRrtStarConfig(RrtStarConfig config);

/// \brief Contains the RRT path planning interface to the OMPL library.
/// This class plans path in complex environments. It utilizes OMPL library,
/// specifically RRT and RRT* algorithms for searching the obstacle-free
/// paths. 
class RrtPathPlanner : public PathPlanningInterface
{
  public:
    /// \brief Constructor with filename.
    /// \param config_filename This constructor takes .yaml file for
    ///   planner configuration.
    /// \param validity_checker Validity checker for robot state. Validity
    ///   checking is all done in class so different types of robots can be
    ///   used.
    RrtPathPlanner(string config_filename, 
      shared_ptr<StateValidityCheckerInterface> validity_checker);

    /// \brief Destructor.
    ~RrtPathPlanner();

    /// \brief Configures planner from file. This is also called by constructor.
    /// \param config_filename Path to the .yaml file for configuration.
    /// \return True if configuration provided is valid.
    bool configureFromFile(string config_filename);

    /// \brief Generates obstacle free path between specified waypoints.
    /// \param positions The waypoints are passed through a matrix. Each row 
    ///   of matrix is one waypoint. Number of columns represents the degrees 
    ///   of freedom.
    /// \return Success of path planning.
    bool planPath(Eigen::MatrixXd positions);

    /// \brief Returns the generated trajectory.
    ///
    /// \return Sampled trajectory.
    Eigen::MatrixXd getPath();

    /// \brief Returns length of the path.
    ///
    /// \return Path length.
    double getPathLength();

  private:
    Eigen::MatrixXd path_;
    double path_length_;
    ob::SpaceInformationPtr si;
    shared_ptr<StateValidityCheckerInterface> state_validity_checker_;
    RrtStarConfig planner_configuration_;
    bool isStateValid(const ob::State *state);
    inline void convertOmplPathToEigenMatrix(og::PathGeometric path);

    ob::StateSpacePtr generateSpace(string type, int dimension, 
      std::vector< std::vector <double> > space_bounds);
};

#endif // RRT_PATH_PLANNER