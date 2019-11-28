#include <larics_motion_planning/ParabolicAirdropTrajectory.h>

ParabolicAirdropTrajectory::ParabolicAirdropTrajectory(
  string config_filename):GlobalPlanner(config_filename)
{
  cout << "Constructor" << endl;
}