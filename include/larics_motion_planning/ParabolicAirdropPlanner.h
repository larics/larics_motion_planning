/// \file ParabolicAirdropPlanner.h
/// \brief Implements parabolic airdrop in cluttered environment

#ifndef PARABOLIC_AIRDROP_PLANNER
#define PARABOLIC_AIRDROP_PLANNER

#include <larics_motion_planning/GlobalPlanner.h>

#include <eigen3/Eigen/Eigen>

#include <iostream>
using namespace std;

/// \brief This class inherits GlobalPlanner for it's full functionality and
///   adds polynomial segment at the end for parabolic load airdrop. The
///   returned trajectory will have time instance of the airdrop.
class ParabolicAirdropPlanner : public GlobalPlanner
{
  public:
    /// \brief Constructor basically configures global planner.
    /// \param config_filename Path to global planner configuration.
    ParabolicAirdropPlanner(string config_filename);

    /// \brief Generates collision free parabolic airdrop trajectory.
    /// \param uav_pose Current uav pose from which we plan the trajectory.
    /// \param target_position Position where we have to drop the package.
    /// \return True if trajectory was successfully generated, false otherwise.
    bool generateParabolicAirdropTrajectory(Eigen::VectorXd uav_pose, 
      Eigen::VectorXd target_pose);

  private:
};



#endif //PARABOLIC_AIRDROP_PLANNER