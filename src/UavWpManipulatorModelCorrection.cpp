#include <larics_motion_planning/UavWpManipulatorModelCorrection.h>

UavWpManipulatorModelCorrection::UavWpManipulatorModelCorrection(
  string config_filename, shared_ptr<KinematicsInterface> kinematics)
{
  kinematics_ = kinematics;
  id_ = -1;
}

UavWpManipulatorModelCorrection::UavWpManipulatorModelCorrection(
  YAML::Node config, shared_ptr<MultipleManipulatorsKinematics> kinematics,
  int id)
{
  // If kinematics are a part of multiple manipulator kinematics then 
  mm_kinematics_ = kinematics;
  id_ = id;
  cout << "Configuring UavWpManipulatorModelCorrection id: " << id;
  cout << "  from yaml node." << endl;

  // Get the manipulator transform.
  std::vector<double> translation, rotation;
  translation = config["t_body_manipulator"]["translation"].as< std::vector<double> >();
  rotation = config["t_body_manipulator"]["rotation"].as< std::vector<double> >();
  // There should be 3 elements in translation and rotation.
  if ((translation.size() != 3) || (rotation.size() != 3)){
    cout << "ERROR: Translation or rotation size of manipulator " << id_; 
    cout << " is not 3." << endl;
    cout << "  This occured in UavWpManipulatorModelCorrection." << endl;
    cout << "  Translation size: " << translation.size() << endl;
    cout << "  Rotation size: " << rotation.size() << endl;
    exit(0);
  }
  // Rotation of uav to manipulator
  Eigen::Matrix3d rot_b_l0;
  rot_b_l0 = Eigen::AngleAxisd(rotation[2], Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(rotation[1],  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(rotation[0], Eigen::Vector3d::UnitX());
  // Create transform
  t_b_l0_ = Eigen::Affine3d::Identity();
  t_b_l0_.translate(Eigen::Vector3d(translation[0], translation[1],
    translation[2]));
  t_b_l0_.rotate(rot_b_l0);

  // Get manipulator dof
  manipulator_dof_ = config["manipulator_dof"].as<int>();
  uav_dof_ = 6;
}

bool UavWpManipulatorModelCorrection::configureFromFile(string config_filename)
{
  return true;
}

Trajectory UavWpManipulatorModelCorrection::modelCorrectedTrajectory(
  Trajectory planned_trajectory, Trajectory executed_trajectory)
{
  // Initialize corrected trajectory that will be returned.
  Trajectory corrected_trajectory = planned_trajectory;
  cout << "Correcting manipulator " << id_ << endl;
  cout << "  Corrected trajectory rows: " << corrected_trajectory.position.rows() << endl;
  cout << "  Corrected trajectory cols: " << corrected_trajectory.position.cols() << endl;

  // Check if received manipulator dof is the same as in config file.
  if (manipulator_dof_ != (corrected_trajectory.position.cols()-uav_dof_)){
    cout << "ERROR: Received manipulator dof is not the same as in config." << endl; 
    cout << "  This occured in UavWpManipulatorModelCorrection." << endl;
    cout << "  Received manipulator dof: " << corrected_trajectory.position.cols()-uav_dof_ << endl;
    cout << "  Config manipulator dof: " << manipulator_dof_ << endl;
    cout << "  Returning planned trajectory!" << endl;
    return planned_trajectory;
  }

  for (int i=0; i<planned_trajectory.position.rows(); i++){
    // Get transform of uav in world frame
    Eigen::Affine3d t_w_b_planned = Eigen::Affine3d::Identity();
    t_w_b_planned.translate(Eigen::Vector3d(planned_trajectory.position(i, 0), 
      planned_trajectory.position(i, 1), planned_trajectory.position(i, 2)));
    Eigen::Matrix3d r_w_b_planned;
    // At this point roll and pitch are 0 since we don't plan for them.
    // Multiply them with zero just in case something got in there.
    r_w_b_planned = Eigen::AngleAxisd(planned_trajectory.position(i, 5), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(planned_trajectory.position(i, 4)*0.0,  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(planned_trajectory.position(i, 3)*0.0,  Eigen::Vector3d::UnitX());
    t_w_b_planned.rotate(r_w_b_planned);

    // Transform from l0 to end-effector.
    Eigen::Affine3d t_l0_ee;
    if (id_ != -1){
      t_l0_ee = mm_kinematics_->getSingleManipulatorEndEffectorTransform(
        (planned_trajectory.position.block(i, 6, 1, manipulator_dof_)).transpose(), id_);
    }
    else{
      t_l0_ee = kinematics_->getEndEffectorTransform(
        (planned_trajectory.position.block(i, 6, 1, manipulator_dof_)).transpose());
    }

    // Calculate the planned end-effector pose in global coordinate system.
    Eigen::Affine3d t_w_ee = t_w_b_planned*t_b_l0_*t_l0_ee;

    // Now we have pose of the end-effector that we desire, and it was planned
    // without any knowledge of roll and pitch. The idea is to include roll and
    // pitch now.
    Eigen::Affine3d t_w_b;
    t_w_b = Eigen::Affine3d::Identity();
    t_w_b.translate(Eigen::Vector3d(planned_trajectory.position(i, 0), 
      planned_trajectory.position(i, 1), planned_trajectory.position(i, 2)));
    // In the rotation we use planned yaw since it is controllable degree of
    // freedom. Roll and pitch are taken out of executed trajectory since
    // these are the by product of the underactuated nature of the multirotor.
    Eigen::Matrix3d r_w_b;
    r_w_b = Eigen::AngleAxisd(planned_trajectory.position(i, 5), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(executed_trajectory.position(i, 4), Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(executed_trajectory.position(i, 3), Eigen::Vector3d::UnitX());
    t_w_b.rotate(r_w_b);

    // With new transform we can calculate true end-effector position in
    // manipulator base frame. This includes roll and pitch now which were not
    // planned earlier. Inverse kinematics works in l0(base manipulator
    // frame).
    t_l0_ee = t_b_l0_.inverse()*t_w_b.inverse()*t_w_ee;

    // And finally, we can get the manipulator joint values that satisfy the
    // t_l0_ee.
    bool found_ik;
    Eigen::VectorXd ik_solution;
    if (id_ != -1){
      ik_solution = mm_kinematics_->calculateSingleManipulatorInverseKinematics(
        t_l0_ee, id_, found_ik);
    }
    else{
      ik_solution = kinematics_->calculateInverseKinematics(t_l0_ee, found_ik);
    }

    if (found_ik == false){
      cout << "  Inverse solution not found for iteration: " << i << endl;
    }
    else{
      corrected_trajectory.position.block(i, 6, 1, manipulator_dof_) = ik_solution.transpose();
    }
  }

  return corrected_trajectory;
}