/// \file ParabolicAirdropTrajectory.h
/// \brief Implements parabolic airdrop in cluttered environment

#ifndef PARABOLIC_AIRDROP_TRAJECTORY
#define PARABOLIC_AIRDROP_TRAJECTORY

#include <larics_motion_planning/GlobalPlanner.h>

#include <eigen3/Eigen/Eigen>

#include <iostream>
using namespace std;

class ParabolicAirdropTrajectory : public GlobalPlanner
{
  public:
    ParabolicAirdropTrajectory(string config_filename);

  private:
};



#endif //PARABOLIC_AIRDROP_TRAJECTORY