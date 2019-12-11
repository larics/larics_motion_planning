#include <larics_motion_planning/ParabolicAirdropPlanner.h>

ParabolicAirdropPlanner::ParabolicAirdropPlanner(
  string config_filename):GlobalPlanner(config_filename)
{
  cout << "Constructor" << endl;
  point_checker_ = make_shared<SimpleStateValidityCheckers>(map_interface_,
    "point", Eigen::VectorXd());
}

bool ParabolicAirdropPlanner::generateParabolicAirdropTrajectory(
  Eigen::VectorXd uav_pose, Eigen::VectorXd target_pose)
{
  // Iterate through dx, velocity and alpha to get parabola parameters.
  // TODO: create lists for dx, v and alpha so the search time reduces.

  parabola_set_points_ = Eigen::MatrixXd(0,3);
  double g=9.81;
  for (double dx=1.0; dx<=8.0; dx+=0.5){
    for (double v=0.5; v<=6.0; v+=0.5){
      for (double alpha=0.0; alpha<=deg2rad(45.0); alpha+=deg2rad(5.0)){
        // Calculate displacement in z axis and time of impact
        double dz = -tan(alpha)*(dx) + 0.5*g*dx*dx/(v*v*cos(alpha)*cos(alpha));
        double t = dx/(v*cos(alpha));

        if (dz < 50.0){

          // Construct the parabola. Note that this is not final one, it still
          // has to be transformed to world.
          Eigen::MatrixXd parabola;
          parabola = constructParabola(dx, dz, alpha, v, t, g);

          // Translate and rotate parabola to find suitable one. We do this in
          // a for loop by changing yaw angle.
          for (double parabola_yaw=0.0; parabola_yaw<=deg2rad(360.0);
            parabola_yaw+=deg2rad(22.5)){

            Eigen::MatrixXd transformed_parabola;
            transformed_parabola = transformParabola(parabola, target_pose,
              parabola_yaw, dx);

            // Plan stopping trajectory
            Eigen::MatrixXd conditions(3, 6);
            conditions << transformed_parabola(0,0), transformed_parabola(0,0), 
              v*cos(alpha)*cos(parabola_yaw), 0, 0, 0,
              transformed_parabola(0,1), transformed_parabola(0,1), 
              v*cos(alpha)*sin(parabola_yaw), 0, 0, 0,
              transformed_parabola(0,2), transformed_parabola(0,2), 
              v*sin(alpha), 0, 0, 0;
            Eigen::MatrixXd constraints(3, 2);
            constraints << 8, 5, 8, 5, 5, 4;
            spline_interpolator_.generateTrajectory(conditions, constraints, 
              0.01);
            Trajectory stopping_trajectory = spline_interpolator_.getTrajectory();


            bool valid_flag = true;
            // Check parabola candidate for collision
            valid_flag &= checkParabolaForCollision(transformed_parabola);
            // Check stopping trajectory for collision
            valid_flag &= checkTrajectoryForCollision(stopping_trajectory);

            // Plan collision free trajectory to dropoff point
            // Set up waypoints
            Eigen::MatrixXd waypoints(2, 4);
            double q0 = uav_pose(3);
            double q3 = uav_pose(2);
            double yaw = atan2(2*q0*q3, 1-2*(q3*q3));
            // Fill waypoints
            waypoints << uav_pose(0), uav_pose(1), uav_pose(2), yaw, 
              transformed_parabola(0, 0), transformed_parabola(0, 1), 
              transformed_parabola(0, 2), yaw;
            // Plan trajectory
            this->planPathAndTrajectory(waypoints);
            Trajectory trajectory = trajectory_interface_->getTrajectory();

            // Plan dropoff spline
            Trajectory dropoff_trajectory;
            dropoff_trajectory = planDropoffSpline(trajectory, v, 
              alpha, parabola_yaw);


            /*if (v > 3.0){
              parabola_set_points_.conservativeResize(parabola_set_points_.rows() +
                transformed_parabola.rows(), 3);
              parabola_set_points_.block(
                parabola_set_points_.rows() - transformed_parabola.rows(), 0, 
                transformed_parabola.rows(), 3) = transformed_parabola;
            }*/
          }
        }

        // Check collision of that spline part
        // Mozda da probamo naci najbolju parabolu ili set najboljih po nekom
        //   kriteriju. Zbroj dx i dz mozda da bude najmanji? Zbroj*v da je
        //   najmanji?

      }
    }
  }

  return true;
}

Eigen::MatrixXd ParabolicAirdropPlanner::constructParabola(double dx,
  double dz, double alpha, double v, double duration, double g)
{
  double ts = 0.01;
  int n = round(duration/ts);
  double t = 0.0;
  Eigen::MatrixXd parabola(n+1, 3);

  for (int i=0; i<=n; i++){
    parabola(i, 0) = v*cos(alpha)*t;
    parabola(i, 1) = 0.0;
    parabola(i, 2) = v*sin(alpha)*t - 0.5*g*t*t + dz;
    t+=ts;
  }

  return parabola;
}

Eigen::MatrixXd ParabolicAirdropPlanner::transformParabola(Eigen::MatrixXd parabola,
  Eigen::VectorXd target, double yaw, double dp)
{
  Eigen::MatrixXd transformed_parabola(parabola.rows(), parabola.cols());

  double cy = cos(yaw);
  double sy = sin(yaw);
  for (int i=0; i<parabola.rows(); i++){
    transformed_parabola(i, 0) = target(0) - dp*cy + parabola(i, 0)*cy - parabola(i, 1)*sy;
    transformed_parabola(i, 1) = target(1) - dp*sy + parabola(i, 0)*sy + parabola(i, 1)*cy;
    transformed_parabola(i, 2) = target(2) + parabola(i, 2);
  }

  return transformed_parabola;
}

bool ParabolicAirdropPlanner::checkParabolaForCollision(
  Eigen::MatrixXd parabola)
{
  bool is_valid = true;
  
  for (int i=0; i<parabola.rows(); i++){
    if (parabola(i, 2) > 100) cout << parabola.row(i) << endl;
    is_valid &= point_checker_->isStateValid((parabola.row(i)).transpose());
  }

  return is_valid;
}

bool ParabolicAirdropPlanner::checkTrajectoryForCollision(
  Trajectory trajectory)
{
  // TODO: implement this function, we don't need it know because we will not
  // work in cluttered environments.
  return true;
}

Trajectory ParabolicAirdropPlanner::planDropoffSpline(
  Trajectory trajectory, double v, double alpha, double parabola_yaw)
{
  return trajectory;
}

inline double deg2rad(double deg)
{
  return deg*M_PI/180.0;
}
