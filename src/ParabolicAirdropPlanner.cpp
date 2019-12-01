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
        double dz = -tan(alpha)*(dx) + 0.5*g*dx*dx/(v*v*cos(alpha)*cos(alpha));
        double t = dx/(v*cos(alpha));

        Eigen::MatrixXd parabola;
        parabola = constructParabola(dx, dz, alpha, v, t, g);
        // Rotate parabola around z axis to find suitable one
        // Check parabola collision
        // Check stopping trajectory collision
        // Plan trajectory to dropoff point
        // Plan last segment with spline
        // Check collision of that spline part
        // Mozda da probamo naci najbolju parabolu ili set najboljih po nekom
        //   kriteriju. Zbroj dx i dz mozda da bude najmanji? Zbroj*v da je
        //   najmanji?
        cout << parabola << endl;
        cout << dz << endl;
        exit(0);

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

inline double deg2rad(double deg)
{
  return deg*M_PI/180.0;
}
