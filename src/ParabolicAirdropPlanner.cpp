#include <larics_motion_planning/ParabolicAirdropPlanner.h>

ParabolicAirdropPlanner::ParabolicAirdropPlanner(
  string config_filename):GlobalPlanner(config_filename)
{
  cout << "Constructor" << endl;
}

bool ParabolicAirdropPlanner::generateParabolicAirdropTrajectory(
  Eigen::VectorXd uav_pose, Eigen::VectorXd target_pose)
{
  // Iterate through dx, velocity and alpha to get parabola parameters.
  double g=9.81;
  for (double dx=1.0; dx<=8.0; dx+=0.5){
    for (double v=0.5; v<=6.0; v+=0.5){
      for (double alpha=0.0; alpha<=deg2rad(45.0); alpha+=deg2rad(5.0)){
        // Calculate displacement in z axis and time of impact
        double dz = -tan(alpha)*(dx) + 0.5*g*dx*dx/(v*v*cos(alpha)*cos(alpha));
        double t = dx/(v*cos(alpha));

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
        }


        // Rotate parabola around z axis to find suitable one
        // Check parabola collision
        // Check stopping trajectory collision
        // Plan trajectory to dropoff point
        // Plan last segment with spline
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

inline double deg2rad(double deg)
{
  return deg*M_PI/180.0;
}
