/// \file ModelCorrectionInterface.h
/// \brief Interface for model correction of the end-effector

#ifndef MODEL_CORRECTION_INTERFACE
#define MODEL_CORRECTION_INTERFACE

#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <eigen3/Eigen/Eigen>
#include <iostream>

using namespace std;

/// This is an interface class for model correction.
class ModelCorrectionInterface
{
public:
  virtual Trajectory modelCorrectedTrajectory(Trajectory planned_trajectory,
    Trajectory executed_trajectory) = 0;
  virtual bool configureFromFile(string config_filename) {return true;}
};


#endif //MODEL_CORRECTION_INTERFACE