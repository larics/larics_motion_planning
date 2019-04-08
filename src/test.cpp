#include <TrajectoryInterface.h>
#include <ToppraTrajectory.h>
#include <MapInterface.h>
#include <OctomapMap.h>
#include <RrtPathPlanner.h>
#include <Visualization.h>
using namespace std;

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
  cout << "In octomap callback " << endl;
  OctomapMap map(0.10);
  map.setOctomapFromRosMessage(msg);
  Eigen::VectorXd state(3);
  cout << state << endl;
  cout << map.isStateValid(state) << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testing_node");
  /*Eigen::MatrixXd constraints(2, 4);
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
  */
  
  /*cout << "Starting map testing." << endl;
  ros::NodeHandle n;
  OctomapMap map(0.1);
  map.setDepth(16);
  //auto octomapSub=n.subscribe("/octomap_binary", 1, &octomapCallback);
  auto octomapSub=n.subscribe("/octomap_binary", 1, &OctomapMap::setOctomapFromRosMessage, &map);
  ros::Rate r(10);
  Eigen::VectorXd state(3);
  state(0) = -5;
  state(1) = -11.42;
  state(2) = 2.68;
  usleep(1000000);
  double t, t1;
  while (ros::ok()){
    ros::spinOnce();
    r.sleep();
    t = ros::Time::now().toSec();
    map.isStateValid(state);
    t1 = ros::Time::now().toSec();
    cout << t1-t << map.isStateValid(state) << endl;
  }*/

  //OctomapMap map("/home/antun/catkin_ws/src/larics_gazebo_worlds/models/floor_plan_simple/model_rotated.binvox.bt", 16);
  //map.configureFromFile("/home/antun/rotirana_mapa.binvox.bt");
  //Eigen::VectorXd state(3);
  //state(1) = -2;
  //state(2) = 1.5;
  //cout << state << endl;
  //cout << map.isStateValid(state) << endl;
  //map.configureFromFile("haha");
  shared_ptr<OctomapMap> map;
  map = make_shared<OctomapMap>(
    "/home/antun/catkin_ws/src/larics_motion_planning/config/octomap_config_example.yaml");
  shared_ptr<MapInterface> interface_map = map;
  RrtPathPlanner path_planner("/home/antun/catkin_ws/src/larics_motion_planning/config/path_planner_config_example.yaml", interface_map);
  Eigen::MatrixXd waypoints(2,3);
  waypoints << 1.57, -8.74, 1.0, 8.68, 8.24, 1.0;
  path_planner.planPath(waypoints);
  Visualization viz;
  viz.eigenPathToNavMsgsPath(path_planner.getPath());

  ToppraTrajectory a("/home/antun/catkin_ws/src/larics_motion_planning/config/toppra_config_example.yaml");
  a.generateTrajectory(path_planner.getPath());
  viz.eigenTrajectoryToNavMsgsPath(a.getTrajectory());

  // Setup some ros for publishing the path
  ros::Rate r(10);
  while (ros::ok()){
    ros::spinOnce();
    r.sleep();
    viz.publishPath();
    viz.publishTrajectory();
  }
  //cout << viz.getPath() << endl;
  //cout << path_planner.getPath() << endl;
  return 0;
}