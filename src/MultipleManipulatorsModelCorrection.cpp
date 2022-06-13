#include <larics_motion_planning/MultipleManipulatorsModelCorrection.h>
#include <larics_motion_planning/MotionPlanningUtil.h>

MultipleManipulatorsModelCorrection::MultipleManipulatorsModelCorrection(
  string config_filename, shared_ptr<KinematicsInterface> kinematics)
{
  // Immediately cast into multiple manipulator kinematics. This is not pretty
  // but it is necessary to get single manipulator joint positions.
  kinematics_ = dynamic_pointer_cast<MultipleManipulatorsKinematics>(kinematics);
  configureFromFile(motion_util::getUserPrefix() + config_filename);
}

bool MultipleManipulatorsModelCorrection::configureFromFile(
  string config_filename)
{
  cout << "Configuring multiple manipulators model correction from file: " << endl;
  cout << "  " << config_filename << endl;

  // Open yaml file with configuration
  YAML::Node config = YAML::LoadFile(config_filename);

  // Get the number of manipulators. This should be the same as in kinematics
  // interface
  n_manipulators_ = config["model_correction"]["multiple_manipulators"].size();
  
  // Go through all manipulators and configure
  for (int i=0; i<n_manipulators_; i++){
    // Load each model correction as interface shared pointer and
    // add it to the list of manipulators interfaces.
    shared_ptr<ModelCorrectionInterface> man;
    string type = 
      config["model_correction"]["multiple_manipulators"][i]["type"].as<string>();
    if (type == "UavWpManipulator"){
      man = make_shared<UavWpManipulatorModelCorrection>(
        config["model_correction"]["multiple_manipulators"][i], kinematics_, i);
      manipulators_.push_back(man);
    }
    else{
      cout << "ERROR: The manipulator " << i << " type is not supported." << endl;
      cout << "  This occured in MultipleManipulatorsModelCorrection." << endl;
      cout << "  Manipulator type is " << type << endl;
      exit(0);
    }

    // Get the indexes of each manipulator.
    int start, end;
    start = config["model_correction"]["multiple_manipulators"][i]["indexes"]["start"].as<int>();
    end = config["model_correction"]["multiple_manipulators"][i]["indexes"]["end"].as<int>();
    start_indexes_.push_back(start);
    end_indexes_.push_back(end);
  }
  return true;
}


Trajectory MultipleManipulatorsModelCorrection::modelCorrectedTrajectory(
  Trajectory planned_trajectory, Trajectory executed_trajectory)
{
  cout << "Starting multiple manipulators model correction." << endl;
  Trajectory corrected_trajectory = planned_trajectory;

  for (int i=0; i<n_manipulators_; i++){
    // Take chunk of planned trajectory that corresponds to current manipulator.
    Trajectory planned_chunk;
    planned_chunk.position.resize(planned_trajectory.position.rows(),
      end_indexes_[i]-start_indexes_[i]+1);
    planned_chunk.velocity.resize(planned_trajectory.velocity.rows(),
      end_indexes_[i]-start_indexes_[i]+1);
    planned_chunk.acceleration.resize(planned_trajectory.acceleration.rows(),
      end_indexes_[i]-start_indexes_[i]+1);
    planned_chunk.position = planned_trajectory.position.block(0, start_indexes_[i],
      planned_trajectory.position.rows(), end_indexes_[i]-start_indexes_[i]+1);
    planned_chunk.velocity = planned_trajectory.velocity.block(0, start_indexes_[i],
      planned_trajectory.velocity.rows(), end_indexes_[i]-start_indexes_[i]+1);
    planned_chunk.acceleration = planned_trajectory.acceleration.block(0, start_indexes_[i],
      planned_trajectory.acceleration.rows(), end_indexes_[i]-start_indexes_[i]+1);

    // Take chunk of executed trajectory that corresponds to current manipulator.
    Trajectory executed_chunk;
    executed_chunk.position.resize(planned_trajectory.position.rows(),
      end_indexes_[i]-start_indexes_[i]+1);
    executed_chunk.velocity.resize(planned_trajectory.velocity.rows(),
      end_indexes_[i]-start_indexes_[i]+1);
    executed_chunk.acceleration.resize(planned_trajectory.acceleration.rows(),
      end_indexes_[i]-start_indexes_[i]+1);
    executed_chunk.position = executed_trajectory.position.block(0, start_indexes_[i],
      executed_trajectory.position.rows(), end_indexes_[i]-start_indexes_[i]+1);
    executed_chunk.velocity = executed_trajectory.velocity.block(0, start_indexes_[i],
      executed_trajectory.velocity.rows(), end_indexes_[i]-start_indexes_[i]+1);
    executed_chunk.acceleration = executed_trajectory.acceleration.block(0, start_indexes_[i],
      executed_trajectory.acceleration.rows(), end_indexes_[i]-start_indexes_[i]+1);
    //cout << "planned chunk: " << endl << planned_chunk.position << endl;
    //cout << "executed chunk: " << endl << executed_chunk.position << endl;
    //exit(0);

    // Use current manipulator to correct trajectory.
    Trajectory corrected_chunk = manipulators_[i]->modelCorrectedTrajectory(
      planned_chunk, executed_chunk);

    // Fill the corrected trajectory with the corrected chunk.
    corrected_trajectory.position.block(0, start_indexes_[i],
      corrected_trajectory.position.rows(), 
      end_indexes_[i]-start_indexes_[i]+1) = corrected_chunk.position;
    corrected_trajectory.velocity.block(0, start_indexes_[i],
      corrected_trajectory.velocity.rows(), 
      end_indexes_[i]-start_indexes_[i]+1) = corrected_chunk.velocity;
    corrected_trajectory.acceleration.block(0, start_indexes_[i],
      corrected_trajectory.acceleration.rows(), 
      end_indexes_[i]-start_indexes_[i]+1) = corrected_chunk.acceleration;
  }

  cout << "Multiple manipulators model corrections done! Returning trajectory." << endl;

  return corrected_trajectory;
}