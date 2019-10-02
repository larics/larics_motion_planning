#include <larics_motion_planning/BallStateValidityChecker.h>

BallStateValidityChecker::BallStateValidityChecker(
  string config_filename, shared_ptr<MapInterface> map)
{
  map_ = map;

  string username = "/home/";
  username = username + getenv("USERNAME") + "/";
  configureFromFile(username + config_filename);
  points_ = generateBall();
  //cout << points_.rows() << " " << points_.cols() << endl;
  //cout << points_ << endl;
}

bool BallStateValidityChecker::configureFromFile(string config_filename)
{
  cout << "Configuring global planner from file: " << endl;
  cout << "  " << config_filename << endl;
  // Open yaml file with configuration
  YAML::Node config = YAML::LoadFile(config_filename);

  // Get ball radius and resolution of points.
  radius_ = config["state_validity_checker"]["ball"]["radius"].as<double>();
  resolution_ = config["state_validity_checker"]["ball"]["resolution"].as<double>();

  return true;
}

bool BallStateValidityChecker::isStateValid(Eigen::VectorXd state)
{
  return true;
}

Eigen::MatrixXd BallStateValidityChecker::generateValidityPoints(
  Eigen::VectorXd state)
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  for (int i=0; i<state.size() && i<3; i++) position(i) = state(i);
  Eigen::MatrixXd state_points = points_;
  for (int i=0; i<state_points.rows(); i++){
    state_points(i,0) += position(0);
    state_points(i,1) += position(1);
    state_points(i,2) += position(2);
  }

  return state_points;
}

Eigen::MatrixXd BallStateValidityChecker::generateBall()
{
  //cout << points << endl;

  int points_size = 0;
  for (double radius = resolution_; radius <= (radius_+resolution_/2.0); 
    radius+=resolution_){
    double sphere_area = 4.0*M_PI*radius*radius;
    double a = sqrt(sphere_area);
    int n = 0.5*(a+resolution_)*(a+resolution_)/(resolution_*resolution_);
    //cout << "n: " << n << endl;

    for (int i=0; i<n; i++){
      for (int j=0; j<n; j++){
        //points.conservativeResize(points.rows()+1, 3);
        points_size++;
      }
    }
  }
  Eigen::MatrixXd points(points_size,3);
  //points.conservativeResize(points.rows()+points_size, 3);
  points_size = 0;
  for (double radius = resolution_; radius <= (radius_+resolution_/2.0); 
    radius+=resolution_){
    double sphere_area = 4.0*M_PI*radius*radius;
    double a = sqrt(sphere_area);
    int n = 0.5*(a+resolution_)*(a+resolution_)/(resolution_*resolution_);
    //cout << "n: " << n << endl;

    for (int i=0; i<n; i++){
      for (int j=0; j<n; j++){
        double theta = double(i)/double(n)*2.0*M_PI;
        double phi = double(j)/double(n)*2.0*M_PI;
        points(points_size, 0) = radius*sin(theta)*cos(phi);
        points(points_size, 1) = radius*sin(theta)*sin(phi);
        points(points_size, 2) = radius*cos(theta);

        points_size++;
      }
    }
  }
  
  //cout << points_size << endl;
  //cout << points.rows() << " " << points.cols() << endl;

  return points;
}