#include <UavWpManipulatorStateValidityChecker.h>
#include <aerial_manipulators_control/ManipulatorControl.h>

UavWpManipulatorStateValidityChecker::UavWpManipulatorStateValidityChecker(
  string config_filename, shared_ptr<MapInterface> map)
{
  map_ = map;

  string robot_model_name = "wp_manipulator";
  string joint_group_name = "wp_manipulator_arm";

  wp_control.setManipulatorName(robot_model_name, joint_group_name);
  wp_control.LoadParameters(
    "/home/antun/catkin_ws/src/aerial_manipulators/aerial_manipulators_control/config/wp_manipulator_dh_parameters.yaml");
  wp_control.init();
}

bool UavWpManipulatorStateValidityChecker::isStateValid(Eigen::VectorXd state)
{
  return map_->isStateValid(state);
}