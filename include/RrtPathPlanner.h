/// \file RrtPathPlanner.h

#ifndef RRT_PATH_PLANNER
#define RRT_PATH_PLANNER

#include "MotionPlanningDatatypes.h"
#include "PathPlanningInterface.h"

#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <Eigen/Eigen>

#include "yaml-cpp/yaml.h"

using namespace std;

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/OptimizationObjective.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;

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
    RrtPathPlanner(string config_filename);

    /// \brief Destructor.
    ~RrtPathPlanner();

    /// \brief Generates obstacle free path between specified waypoints.
    /// \param positions The waypoints are passed through a matrix. Each row of matrix is one
    ///   one waypoint. Number of columns represents the degrees of freedom.
    /// \return Success of path planning.
    bool planPath(Eigen::MatrixXd positions);

    /// \brief Returns the generated trajectory.
    ///
    /// \return Sampled trajectory.
    Eigen::MatrixXd getPath();

  private:
    Eigen::MatrixXd path_;
    bool isStateValid(const ob::State *state);
};

#endif // RRT_PATH_PLANNER