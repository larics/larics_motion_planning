/// \file KinematicsInterface.h
/// \brief Contains the abstract class interface for robot kinematics

#ifndef KINEMATICS_INTERFACE_H
#define KINEMATICS_INTERFACE_H

#include <larics_motion_planning/MotionPlanningDatatypes.h>

#include <Eigen/Eigen>
#include <string>

using namespace std;

/// This is an interface class for trajectories of all types.
class KinematicsInterface
{
  public:
    KinematicsInterface();
    virtual bool configureFromFile(string config_filename) = 0;
    virtual std::vector<Eigen::Affine3d> getJointPositions(
      Eigen::VectorXd q) = 0;
    virtual Eigen::Affine3d getEndEffectorTransform(Eigen::VectorXd q) = 0;
    virtual Eigen::VectorXd calculateInverseKinematics(
      Eigen::Affine3d transform, bool &found_ik) = 0;
    virtual std::vector<Eigen::Affine3d> getMultipleEndEffectorTransforms(
      Eigen::VectorXd q)
    {
      std::vector<Eigen::Affine3d> vec;
      vec.push_back(this->getEndEffectorTransform(q));
      return vec;
    }
    virtual void setJointPositions(Eigen::VectorXd q){}
    virtual Eigen::MatrixXd getJacobian(Eigen::VectorXd q)
    {
      Eigen::MatrixXd jacobian;
      return jacobian;
    }
};

#endif // KINEMATICS_INTERFACE_H