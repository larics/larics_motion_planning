#include "TrajectoryInterface.h"
#include "ToppraTrajectory.h"
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testing_node");

  Eigen::MatrixXd constraints(2, 4);
  constraints << 0.5, 0.5, 0.5, 0.5, 1.2, 1.2, 1.2, 1.2;
  ToppraTrajectory a(constraints);

  Eigen::MatrixXd waypoints(3, 4);
  waypoints << 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0;

  //cout << waypoints << endl;
  //usleep(1000000);
  double t=ros::Time::now().toSec();
  a.generateTrajectory(waypoints);
  Trajectory temp_trajectory = a.getTrajectory();
  cout << ros::Time::now().toSec()-t << endl;
  cout << "Starting testing program." << endl;
  return 0;
}