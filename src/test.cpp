#include <larics_motion_planning/TrajectoryInterface.h>
#include <larics_motion_planning/ToppraTrajectory.h>
#include <larics_motion_planning/MapInterface.h>
#include <larics_motion_planning/OctomapMap.h>
#include <larics_motion_planning/RrtPathPlanner.h>
#include <larics_motion_planning/Visualization.h>
#include <larics_motion_planning/GlobalPlanner.h>
#include <larics_motion_planning/SplineInterpolator.h>
using namespace std;

/*void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
  cout << "In octomap callback " << endl;
  OctomapMap map(0.10);
  map.setOctomapFromRosMessage(msg);
  Eigen::VectorXd state(3);
  cout << state << endl;
  cout << map.isStateValid(state) << endl;
}*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testing_node");

  /*SplineInterpolator spline_interpolator;
  Eigen::VectorXd conditions(6);
  conditions << 0.4, 1.5321321, 0.17, 0.3, 0.1, 0.0;
  Eigen::VectorXd constraints(2);
  constraints << 0.5, 1.5;
  cout << spline_interpolator.generateSplineOrder5(conditions, constraints, 0.01) << endl;
  */

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

  cout << "Starting map testing." << endl;
  ros::NodeHandle n;
  shared_ptr<OctomapMap> map = make_shared<OctomapMap>(0.1);
  map->setDepth(16);
  //auto octomapSub=n.subscribe("/octomap_binary", 1, &octomapCallback);
  auto octomapSub=n.subscribe("/octomap_binary", 1, &OctomapMap::setOctomapFromRosMessage, map.get());
  ros::Rate r(10);
  Eigen::VectorXd state(3);
  state(0) = -5;
  state(1) = -11.42;
  state(2) = 2.68;
  usleep(1000000);
  double t, t1;
  /*while (ros::ok()){
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
  /*shared_ptr<OctomapMap> map;
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
  //cout << path_planner.getPath() << endl;*/
  /*string username = "/home/";
  username = username + getenv("USER") + "/";
  GlobalPlanner gp(username + "catkin_ws/src/larics_motion_planning/config/global_planner_config_example.yaml");
  Eigen::MatrixXd waypoints(5,3);
  waypoints << 1.57, -8.74, 1.0,
               8.86, -2.23, 1.0,
               8.68, 8.24, 1.0,
               1.16, 8.01, 1.0,
               1.7, 2.65, 1.0;
  gp.planPathAndTrajectory(waypoints);

  Visualization viz;
  viz.setPath(gp.getPath());
  viz.setTrajectory(gp.getTrajectory().position, true);
  viz.setWaypoints(waypoints);

  // Setup some ros for publishing the path
  ros::Rate r(10);
  while (ros::ok()){
    ros::spinOnce();
    r.sleep();
    viz.publishPath();
    viz.publishTrajectory();
    viz.publishWaypoints();
  }*/
  return 0;
}
