#include <larics_motion_planning/MultipleManipulatorsModelCorrection.h>

MultipleManipulatorsModelCorrection::MultipleManipulatorsModelCorrection(
  string config_filename, shared_ptr<KinematicsInterface> kinematics)
{
  // Immediately cast into multiple manipulator kinematics. This is not pretty
  // but it is necessary to get single manipulator joint positions.
  kinematics_ = dynamic_pointer_cast<MultipleManipulatorsKinematics>(kinematics);

  string username = "/home/";
  username = username + getenv("USER") + "/";
  configureFromFile(username + config_filename);
}

bool MultipleManipulatorsModelCorrection::configureFromFile(
  string config_filename)
{
  cout << "Configuring multiple manipulators model correction from file: " << endl;
  cout << "  " << config_filename << endl;

  // Open yaml file with configuration
  YAML::Node config = YAML::LoadFile(config_filename);
  
  // Get the number of manipulators. This should be the same number as in 
  // kinematics interface.
  //n_manipulators_ = config["state_validity_checker"]["multiple_manipulators"].size();
  // Check if there is the same number of manipulators in both state validity
  // checker and kinematics
  /*if (n_manipulators_ != config["kinematics"]["multiple_manipulators"].size()){
    cout << "ERROR: Number of manipulators is different in kinematics." << endl;
    cout << "  This occured in MultipleManipulatorsStateValidityChecker." << endl;
    cout << "  Number of manipulators in state validity checker: " << n_manipulators_ << endl;
    cout << "  Number of manipulators in kinematics: ";
    cout << config["kinematics"]["multiple_manipulators"].size() << endl;
    exit(0);
  }

  // Go through all manipulators and configure
  for (int i=0; i<n_manipulators_; i++){

  }*/
}


Trajectory MultipleManipulatorsModelCorrection::modelCorrectedTrajectory(
  Trajectory planned_trajectory, Trajectory executed_trajectory)
{
  cout << "Hello from multiple manipulators." << endl;
  Trajectory corrected_trajectory = planned_trajectory;
  return corrected_trajectory;
}