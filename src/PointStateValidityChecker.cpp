#include <PointStateValidityChecker.h>

PointStateValidityChecker::PointStateValidityChecker(
  string config_filename, shared_ptr<MapInterface> map)
{

}

bool PointStateValidityChecker::isStateValid(Eigen::VectorXd state)
{
  return true;
}