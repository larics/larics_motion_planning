/// \file ParabolicAirdropPlanner.h
/// \brief Implements parabolic airdrop in cluttered environment

#ifndef PARABOLIC_AIRDROP_PLANNER
#define PARABOLIC_AIRDROP_PLANNER

#include <larics_motion_planning/GlobalPlanner.h>
#include <larics_motion_planning/SplineInterpolator.h>

#include <eigen3/Eigen/Eigen>
#include <cmath>

#include <iostream>
using namespace std;

/// \brief This class inherits GlobalPlanner for it's full functionality and
///   adds polynomial segment at the end for parabolic load airdrop. The
///   returned trajectory will have time instance of the airdrop.
class ParabolicAirdropPlanner : public GlobalPlanner
{
  public:
    /// \brief Constructor basically configures global planner.
    /// \param config_filename Path to global planner configuration.
    ParabolicAirdropPlanner(string config_filename);

    ///
    bool configureParabolicAirdropFromFile(string config_filename);

    ///
    void setMapInterface(shared_ptr<MapInterface> map);

    /// \brief Generates collision free parabolic airdrop trajectory.
    /// \param uav_pose Current uav pose from which we plan the trajectory.
    /// \param target_position Position where we have to drop the package.
    /// \return True if trajectory was successfully generated, false otherwise.
    bool generateParabolicAirdropTrajectory(Eigen::VectorXd uav_pose, 
      Eigen::VectorXd target_pose, bool plan_path);

    /// \brief Generates collision free parabolic airdrop trajectory.
    /// \param uav_pose Current uav pose from which we plan the trajectory.
    /// \param target_position Position where we have to drop the package.
    /// \param psi_min Minimum parabola yaw override.
    /// \param psi_increment Parabola yaw increment override.
    /// \param psi_max Parabola yaw max override.
    /// \return True if trajectory was successfully generated, false otherwise.
    bool generateParabolicAirdropTrajectory(Eigen::VectorXd uav_pose, 
      Eigen::VectorXd target_pose, bool plan_path, double psi_min,
      double psi_increment, double psi_max);

    /// \brief Generates collision free parabolic airdrop trajectory with user
    ///         Specified parameters.
    /// \param uav_pose Current uav pose from which we plan the trajectory.
    /// \param target_position Position where we have to drop the package.
    /// \param plan_path True if a path in cluttered environment is planned.
    /// \param parabola_params Vector of parabola parameters. Contains
    ///         [v0, dz, alpha, dx, psi]
    /// \return True if trajectory was successfully generated, false otherwise.
    bool generateParabolicAirdropTrajectory(Eigen::VectorXd uav_pose, 
      Eigen::VectorXd target_pose, bool plan_path, 
      Eigen::VectorXd parabola_params);

    /// 
    Eigen::MatrixXd getParabola() {return parabola_set_points_;}

    ///
    Trajectory getAirdropTrajectory() {return airdrop_trajectory_;}

    ///
    Eigen::VectorXd getInfoVector() {return info_vector_;}

  private:
    Eigen::MatrixXd parabola_set_points_;
    Eigen::VectorXd info_vector_;
    shared_ptr<StateValidityCheckerInterface> point_checker_;
    SplineInterpolator spline_interpolator_;
    Trajectory airdrop_trajectory_;
    std::vector<double> dx_, v_, alpha_;
    double psi_increment_;
    double max_dz_, payload_z_offset_;
    Eigen::MatrixXd stopping_trajectory_constraints_, 
      dropoff_trajectory_constraints_;
    Eigen::VectorXd intermediate_acceleration_;
    double dropoff_max_line_integral_;
    double spline_sampling_time_;
    double psi_min_, psi_max_;
    double horizontal_stopping_acceleration_, horizontal_dropoff_acceleration_,
      horizontal_intermediate_acceleration_;
    bool use_horizontal_stopping_acceleration_,
      use_horizontal_intermediate_acceleration_, use_horizontal_dropoff_acceleration_;

    Eigen::MatrixXd constructParabola(double dx, double dz, double alpha,
      double v, double t, double g);

    Eigen::MatrixXd transformParabola(Eigen::MatrixXd parabola,
      Eigen::VectorXd target, double yaw, double dp);

    bool checkParabolaForCollision(Eigen::MatrixXd parabola);

    bool checkTrajectoryForCollision(Trajectory trajectory);

    Trajectory planDropoffSpline(Trajectory trajectory, double v,
      double alpha, double parabola_yaw, int &dropoff_index);

    int chooseBestTrajectory(std::vector<Trajectory> spline_list, 
      std::vector<int> spline_index, Trajectory initial_trajectory);

    double trajectoryLineIntegral(Trajectory trajectory);
    double trajectoryLineIntegral(Trajectory trajectory, int start_index, 
      int end_index);
    double calculateTrajectoryRms(Trajectory spline, Trajectory trajectory, 
      string field="position");

    Trajectory concatenateTrajectories(Trajectory first, Trajectory second, 
      int index=-1);

    Trajectory addDropPointColumn(Trajectory trajectory, int index);
};

inline double deg2rad(double deg);



#endif //PARABOLIC_AIRDROP_PLANNER
