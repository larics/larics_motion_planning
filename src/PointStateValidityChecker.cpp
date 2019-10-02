#include <larics_motion_planning/PointStateValidityChecker.h>

PointStateValidityChecker::PointStateValidityChecker(
  shared_ptr<MapInterface> map)
{
  map_ = map;
  points_ = Eigen::MatrixXd(1, 3);
}

bool PointStateValidityChecker::isStateValid(Eigen::VectorXd state)
{
  points_ = state.transpose();
  return map_->isStateValid(state);
}

Eigen::MatrixXd PointStateValidityChecker::generateValidityPoints(Eigen::VectorXd state)
{
  points_ = state.transpose();
  return points_;
}