#include <UavWpManipulatorStateValidityChecker.h>

UavWpManipulatorStateValidityChecker::UavWpManipulatorStateValidityChecker(
  string config_filename, shared_ptr<MapInterface> map)
{
  map_ = map;

  string robot_model_name = "wp_manipulator";
  string joint_group_name = "wp_manipulator_arm";

  manipulator_.setManipulatorName(robot_model_name, joint_group_name);
  manipulator_.LoadParameters(
    "/home/antun/catkin_ws/src/aerial_manipulators/aerial_manipulators_control/config/wp_manipulator_dh_parameters.yaml");
  manipulator_.init();
  testDirectKinematics();
}

void UavWpManipulatorStateValidityChecker::testDirectKinematics()
{
  //Eigen::VectorXd state << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // This one is fixed transformation
  Eigen::Affine3d t_uav_manipulator(Eigen::Affine3d::Identity());
  Eigen::Matrix3d rot_uav_manipulator;
  rot_uav_manipulator = Eigen::AngleAxisd(1.0*M_PI, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(0*M_PI,  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX());
  t_uav_manipulator.translate(Eigen::Vector3d(0, 0, 0.075));
  t_uav_manipulator.rotate(rot_uav_manipulator);
  
  // This one will have to be constructed each time
  Eigen::Affine3d t_world_uav(Eigen::Affine3d::Identity());
  t_world_uav.translate(Eigen::Vector3d(0, 0, 1.0));

  Eigen::Affine3d t_world_manipulator = t_world_uav*t_uav_manipulator;
  cout << t_uav_manipulator.rotation() << endl;

  std::vector<double> q{0.787, 0.787, 0.0, 0.0, 0.0};
  Eigen::Affine3d t_manipulator_end_effector;
  t_manipulator_end_effector = manipulator_.getEndEffectorPosition(q);

  Eigen::Affine3d t_world_end_effector = t_world_manipulator*t_manipulator_end_effector;
  cout << t_world_end_effector.translation() << endl;

  Eigen::Quaterniond quat(t_world_end_effector.rotation());
  cout << quat.coeffs() << endl;

  exit(0);
}

bool UavWpManipulatorStateValidityChecker::isStateValid(Eigen::VectorXd state)
{
  return map_->isStateValid(state);
}