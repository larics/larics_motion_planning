#include <larics_motion_planning/SimpleStateValidityCheckers.h>

SimpleStateValidityCheckers::SimpleStateValidityCheckers(
  string config_filename, shared_ptr<MapInterface> map, string type)
{
  map_ = map;
  checker_type_ = string(type);

  string username = "/home/";
  username = username + getenv("USERNAME") + "/";
  configureFromFile(username + config_filename);
  
  if (checker_type_.compare("ball") == 0) points_ = generateBall();
  else if (checker_type_.compare("sphere") == 0) points_ = generateSphere();
  //cout << points_.rows() << " " << points_.cols() << endl;
  //cout << points_ << endl;
}

bool SimpleStateValidityCheckers::configureFromFile(string config_filename)
{
  cout << "Configuring state validity checker from file: " << endl;
  cout << "  " << config_filename << endl;
  // Open yaml file with configuration
  YAML::Node config = YAML::LoadFile(config_filename);

  // Get ball radius and resolution of points.
  if (checker_type_.compare("ball") == 0){
    ball_radius_ = config["state_validity_checker"]["ball"]["radius"].as<double>();
    ball_resolution_ = config["state_validity_checker"]["ball"]["resolution"].as<double>();
  }
  else if (checker_type_.compare("sphere") == 0){
    sphere_radius_ = config["state_validity_checker"]["sphere"]["radius"].as<double>();
    sphere_resolution_ = config["state_validity_checker"]["sphere"]["resolution"].as<double>();
  }
  else{
    cout << "There is no such checker type. Your type is: " << checker_type_ << endl;
  }
  return true;
}

bool SimpleStateValidityCheckers::isStateValid(Eigen::VectorXd state)
{
  return true;
}

Eigen::MatrixXd SimpleStateValidityCheckers::generateValidityPoints(
  Eigen::VectorXd state)
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  for (int i=0; i<state.size() && i<3; i++) position(i) = state(i);
  Eigen::MatrixXd state_points = points_;
  for (int i=0; i<state_points.rows(); i++){
    for (int j=0; j<state.size() && j<3; j++){
      state_points(i,j) += position(j);
    }
  }

  return state_points;
}

Eigen::MatrixXd SimpleStateValidityCheckers::generateBall()
{
  //cout << points << endl;

  int points_size = 0;
  for (double radius = ball_resolution_; radius <= (ball_radius_+ball_resolution_/2.0); 
    radius+=ball_resolution_){
    double sphere_area = 4.0*M_PI*radius*radius;
    double a = sqrt(sphere_area);
    int n = 0.5*(a+ball_resolution_)*(a+ball_resolution_)/(ball_resolution_*ball_resolution_);
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
  for (double radius = ball_resolution_; radius <= (ball_radius_+ball_resolution_/2.0); 
    radius+=ball_resolution_){
    double sphere_area = 4.0*M_PI*radius*radius;
    double a = sqrt(sphere_area);
    int n = 0.5*(a+ball_resolution_)*(a+ball_resolution_)/(ball_resolution_*ball_resolution_);
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

Eigen::MatrixXd SimpleStateValidityCheckers::generateSphere()
{
  //cout << points << endl;

  int points_size = 0;
  double sphere_area = 4.0*M_PI*sphere_radius_*sphere_radius_;
  double a = sqrt(sphere_area);
  int n = 0.5*(a+sphere_resolution_)*(a+sphere_resolution_)/(
    sphere_resolution_*sphere_resolution_);
  //cout << "n: " << n << endl;

  for (int i=0; i<n; i++){
    for (int j=0; j<n; j++){
      //points.conservativeResize(points.rows()+1, 3);
      points_size++;
    }
  }

  Eigen::MatrixXd points(points_size,3);
  //points.conservativeResize(points.rows()+points_size, 3);
  points_size = 0;

  for (int i=0; i<n; i++){
    for (int j=0; j<n; j++){
      double theta = double(i)/double(n)*2.0*M_PI;
      double phi = double(j)/double(n)*2.0*M_PI;
      points(points_size, 0) = sphere_radius_*sin(theta)*cos(phi);
      points(points_size, 1) = sphere_radius_*sin(theta)*sin(phi);
      points(points_size, 2) = sphere_radius_*cos(theta);
      points_size++;
    }
  }
  
  //cout << points_size << endl;
  //cout << points.rows() << " " << points.cols() << endl;

  return points;
}