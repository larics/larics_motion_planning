#include <larics_motion_planning/SimpleStateValidityCheckers.h>
#include <larics_motion_planning/MotionPlanningUtil.h>

SimpleStateValidityCheckers::SimpleStateValidityCheckers(
  string config_filename, shared_ptr<MapInterface> map, string type)
{
  map_ = map;
  checker_type_ = string(type);

  configureFromFile(motion_util::getUserPrefix() + config_filename);

  if (checker_type_.compare("ball") == 0) points_ = generateBall();
  else if (checker_type_.compare("sphere") == 0) points_ = generateSphere();
  else if (checker_type_.compare("point") == 0) points_ = generatePoint();
  else if (checker_type_.compare("circle") == 0) points_ = generateCircle();
  else if (checker_type_.compare("cylinder") == 0) points_ = generateCylinder();
  else if (checker_type_.compare("rectangle") == 0) points_ = generateRectangle();
  else if (checker_type_.compare("prism") == 0) points_ = generatePrism();
  else{
    cout << "The " << type << " state validity checker does not exist" << endl;
    exit(0);
  }
}

SimpleStateValidityCheckers::SimpleStateValidityCheckers(
  shared_ptr<MapInterface> map, string type, Eigen::VectorXd configuration)
{
  map_ = map;
  checker_type_ = string(type);

  if (checker_type_.compare("ball") == 0){
    if (configuration.size() < 2) {
      cout << "The " << type << " state validity checker requires 2 parameters: " << endl;
      cout << "Ball radius" << endl << "Ball resolution" << endl;
      exit(0);
    }
    ball_radius_ = configuration(0);
    ball_resolution_ = configuration(1);
    points_ = generateBall();
  }
  else if (checker_type_.compare("sphere") == 0){
    if (configuration.size() < 2) {
      cout << "The " << type << " state validity checker requires 2 parameters: " << endl;
      cout << "Sphere radius" << endl << "Sphere resolution" << endl;
      exit(0);
    }
    sphere_radius_ = configuration(0);
    sphere_resolution_ = configuration(1);
    points_ = generateSphere();
  }
  else if (checker_type_.compare("point") == 0){
    points_ = generatePoint();
  }
  else if (checker_type_.compare("circle") == 0){
    if (configuration.size() < 2) {
      cout << "The " << type << " state validity checker requires 2 parameters: " << endl;
      cout << "Circle radius" << endl << "Circle resolution" << endl;
      exit(0);
    }
    circle_radius_ = configuration(0);
    circle_resolution_ = configuration(1);
    points_ = generateCircle();
  }
  else if (checker_type_.compare("cylinder") == 0){
    if (configuration.size() < 3) {
      cout << "The " << type << " state validity checker requires 3 parameters: " << endl;
      cout << "Cylinder radius" << endl << "Cylinder height" << endl << "Cylinder resolution" << endl;
      exit(0);
    }
    cylinder_radius_ = configuration(0);
    cylinder_height_ = configuration(1);
    cylinder_resolution_ = configuration(2);
    points_ = generateCylinder();
  }
  else if (checker_type_.compare("rectangle") == 0){
    if (configuration.size() < 3) {
      cout << "The " << type << " state validity checker requires 3 parameters: " << endl;
      cout << "Rectangle x" << endl << "Rectangle y" << endl << "Rectangle resolution" << endl;
      exit(0);
    }
    rectangle_x_ = configuration(0);
    rectangle_y_ = configuration(1);
    rectangle_resolution_ = configuration(2);
    points_ = generateRectangle();
  }
  else if (checker_type_.compare("prism") == 0){
    if (configuration.size() < 4) {
      cout << "The " << type << " state validity checker requires 4 parameters: " << endl;
      cout << "Prism x" << endl << "Prism y" << endl << "Prism z" << endl << "Prism resolution" << endl;
      exit(0);
    }
    prism_x_ = configuration(0);
    prism_y_ = configuration(1);
    prism_z_ = configuration(2);
    prism_resolution_ = configuration(3);
    points_ = generatePrism();
  }
  else{
    cout << "The " << type << " state validity checker does not exist" << endl;
    exit(0);
  }
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
  else if (checker_type_.compare("circle") == 0){
    circle_radius_ = config["state_validity_checker"]["circle"]["radius"].as<double>();
    circle_resolution_ = config["state_validity_checker"]["circle"]["resolution"].as<double>();
  }
  else if (checker_type_.compare("cylinder") == 0){
    cylinder_radius_ = config["state_validity_checker"]["cylinder"]["radius"].as<double>();
    cylinder_resolution_ = config["state_validity_checker"]["cylinder"]["resolution"].as<double>();
    cylinder_height_ = config["state_validity_checker"]["cylinder"]["height"].as<double>();
  }
  else if (checker_type_.compare("rectangle") == 0){
    rectangle_x_ = config["state_validity_checker"]["rectangle"]["x_dimension"].as<double>();
    rectangle_y_ = config["state_validity_checker"]["rectangle"]["y_dimension"].as<double>();
    rectangle_resolution_ = config["state_validity_checker"]["rectangle"]["resolution"].as<double>();
  }
  else if (checker_type_.compare("prism") == 0){
    prism_x_ = config["state_validity_checker"]["prism"]["x_dimension"].as<double>();
    prism_y_ = config["state_validity_checker"]["prism"]["y_dimension"].as<double>();
    prism_z_ = config["state_validity_checker"]["prism"]["z_dimension"].as<double>();
    prism_resolution_ = config["state_validity_checker"]["prism"]["resolution"].as<double>();
  }
  else{
    cout << "There is no such checker type. Your type is: " << checker_type_ << endl;
  }
  return true;
}

bool SimpleStateValidityCheckers::isStateValid(Eigen::VectorXd state)
{
  // Get points to be checked.
  Eigen::MatrixXd state_points = generateValidityPoints(state);

  // Check for validity
  bool valid_flag = true;
  for (int i=0; i<state_points.rows() && valid_flag==true; i++){
    valid_flag &= map_->isStateValid((state_points.row(i)).transpose());
  }

  return valid_flag;
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
  /*
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
  }*/
  Eigen::MatrixXd points(0,3);
  for (double radius = 0.0; radius < (
    ball_radius_ + ball_resolution_/2.0); radius+=ball_resolution_){
    sphere_radius_ = radius;
    sphere_resolution_ = ball_resolution_;
    Eigen::MatrixXd current_sphere_points = generateSphere();

    int rows = points.rows();
    points.conservativeResize(points.rows()+current_sphere_points.rows(), 3);
    points.block(rows, 0, current_sphere_points.rows(),
      current_sphere_points.cols()) << current_sphere_points;
  }

  //cout << points_size << endl;
  //cout << points.rows() << " " << points.cols() << endl;

  return points;
}

Eigen::MatrixXd SimpleStateValidityCheckers::generateSphere()
{
  //cout << points << endl;
  /*
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
  //cout << points.rows() << " " << points.cols() << endl;*/

  int points_size = 0;
  double n = ceil(2.0*sphere_radius_*M_PI/(sphere_resolution_));
  double delta_phi = 2.0*M_PI/n;

  for (double phi = -M_PI/2.0; phi < (M_PI/2.0+delta_phi/2.0); phi+=delta_phi){
    double r = sphere_radius_*cos(phi);
    double z = sphere_radius_*(1.0-sin(phi));
    double nc = ceil(2.0*r*M_PI/sphere_resolution_);
    double delta_theta = 2.0*M_PI/nc;

    for (double theta = 0.0; theta < (2.0*M_PI-delta_theta/2.0); theta+=delta_theta){
      points_size++;
    }
  }

  Eigen::MatrixXd points(points_size+1,3);
  points_size = 0;

  for (double phi = -M_PI/2.0; phi < (M_PI/2.0+delta_phi/2.0); phi+=delta_phi){
    double r = sphere_radius_*cos(phi);
    double z = sphere_radius_*(1.0-sin(phi));
    double nc = ceil(2.0*r*M_PI/sphere_resolution_);
    double delta_theta = 2.0*M_PI/nc;

    for (double theta = 0.0; theta < (2.0*M_PI-delta_theta/2.0); theta+=delta_theta){
      points(points_size, 0) = r*cos(theta);
      points(points_size, 1) = r*sin(theta);
      points(points_size, 2) = z-sphere_radius_;
      points_size++;
    }
  }
  points(points_size, 0) = 0;
  points(points_size, 1) = 0;
  points(points_size, 2) = -sphere_radius_;

  return points;
}

Eigen::MatrixXd SimpleStateValidityCheckers::generatePoint()
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(1,3);
  return points;
}

Eigen::MatrixXd SimpleStateValidityCheckers::generateCircle(double z)
{
  Eigen::MatrixXd points(1,3);
  points(0,0) = 0.0; points(0,1) = 0.0; points(0,2) = z;
  for (double r=0.0; r<(circle_radius_+circle_resolution_/2.0);
    r+=circle_resolution_){
    double n = ceil(2.0*r*M_PI/circle_resolution_);

    int row = points.rows();
    points.conservativeResize(points.rows()+int(n), 3);
    for (int i=0; i<int(n); i++){
      double phi = double(i)*2.0*M_PI/n;
      points(i+row, 0) = r*cos(phi);
      points(i+row, 1) = r*sin(phi);
      points(i+row, 2) = z;
    }
  }

  return points;
}

Eigen::MatrixXd SimpleStateValidityCheckers::generateCylinder()
{

  Eigen::MatrixXd points(0,3);

  for (double z=0.0; z<(cylinder_height_+cylinder_resolution_/2.0);
    z+=cylinder_resolution_){
    circle_radius_ = cylinder_radius_;
    circle_resolution_ = cylinder_resolution_;
    Eigen::MatrixXd circle = generateCircle(z-cylinder_height_/2.0);

    int rows = points.rows();
    points.conservativeResize(points.rows()+circle.rows(), 3);
    points.block(rows, 0, circle.rows(), circle.cols()) << circle;
  }

  return points;
}

Eigen::MatrixXd SimpleStateValidityCheckers::generateRectangle(double z)
{
  int points_size = 0;
  for (double x=0.0; x<(rectangle_x_+rectangle_resolution_/2.0);
    x+=rectangle_resolution_){
    for (double y=0.0; y<(rectangle_y_+rectangle_resolution_/2.0);
    y+=rectangle_resolution_){
      points_size++;
    }
  }

  Eigen::MatrixXd points(points_size,3);
  points_size = 0;
  for (double x=0.0; x<(rectangle_x_+rectangle_resolution_/2.0);
    x+=rectangle_resolution_){
    for (double y=0.0; y<(rectangle_y_+rectangle_resolution_/2.0);
    y+=rectangle_resolution_){
      points(points_size, 0) = x-rectangle_x_/2.0;
      points(points_size, 1) = y-rectangle_y_/2.0;
      points(points_size, 2) = z;
      points_size++;
    }
  }

  return points;
}

Eigen::MatrixXd SimpleStateValidityCheckers::generatePrism()
{

  Eigen::MatrixXd points(0,3);

  rectangle_x_ = prism_x_;
  rectangle_y_ = prism_y_;
  rectangle_resolution_ = prism_resolution_;
  for (double z=0.0; z<(prism_z_+prism_resolution_/2.0);
    z+=prism_resolution_){
    Eigen::MatrixXd rectangle = generateRectangle(z-prism_z_/2.0);

    int rows = points.rows();
    points.conservativeResize(points.rows()+rectangle.rows(), 3);
    points.block(rows, 0, rectangle.rows(), rectangle.cols()) << rectangle;
  }

  return points;
}
