#include <PointStateValidityChecker.h>

PointStateValidityChecker::PointStateValidityChecker(
  shared_ptr<MapInterface> map)
{
  map_ = map;
}

bool PointStateValidityChecker::isStateValid(Eigen::VectorXd state)
{
  return map_->isStateValid(state);
}