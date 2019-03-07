#include "TrajectoryInterface.h"
#include "ToppraTrajectory.h"
#include "MapInterface.h"
#include "OctomapMap.h"
using namespace std;

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
  cout << "In octomap callback " << endl;
  OctomapMap map(0.10);
  map.setOctomapFromRosMessage(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testing_node");

  Eigen::MatrixXd constraints(2, 4);
  constraints << 0.5, 0.5, 0.5, 0.5, 1.2, 1.2, 1.2, 1.2;
  ToppraTrajectory a("/home/antun/catkin_ws/src/larics_motion_planning/config/toppra_config_example.yaml");

  Eigen::MatrixXd waypoints(3, 4);
  waypoints << 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0;
  cout << waypoints << endl;

  TrajectoryInterface *interface = &a;

  //cout << waypoints << endl;
  //usleep(1000000);
  double t=ros::Time::now().toSec();
  interface->generateTrajectory(waypoints);
  Trajectory temp_trajectory = interface->getTrajectory();
  cout << ros::Time::now().toSec()-t << endl;

  
  /*cout << "Starting map testing." << endl;
  ros::NodeHandle n;
  auto octomapSub=n.subscribe("/octomap_binary", 1, &octomapCallback);

  ros::spin();

  OctomapMap map("/home/antun/rotirana_mapa.binvox.bt", 16);
  map.configureFromFile("/home/antun/rotirana_mapa.binvox.bt");
  Eigen::VectorXd state(3);
  map.isStateValid(state);*/
  //map.configureFromFile("haha");
  return 0;
}