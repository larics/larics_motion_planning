#include <UavWpManipulatorStateValidityChecker.h>

UavWpManipulatorStateValidityChecker::UavWpManipulatorStateValidityChecker(
  string config_filename, shared_ptr<MapInterface> map)
{
  map_ = map;
}

bool UavWpManipulatorStateValidityChecker::isStateValid(Eigen::VectorXd state)
{
  return map_->isStateValid(state);
}