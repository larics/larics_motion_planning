#include <larics_motion_planning/UavWpManipulatorStateValidityChecker.h>

UavWpManipulatorStateValidityChecker::UavWpManipulatorStateValidityChecker(
  string config_filename, shared_ptr<MapInterface> map,
  shared_ptr<KinematicsInterface> kinematics)
{
  map_ = map;
  kinematics_ = kinematics;

  if (getenv("ABSOLUTE_CONFIG")){
    configureFromFile(config_filename);
  }
  else{
    string username = "/home/";
    username = username + getenv("USER") + "/";
    configureFromFile(username + config_filename);
  }

  //testDirectKinematics();
}

bool UavWpManipulatorStateValidityChecker::configureFromFile(
  string config_filename)
{
  cout << "Configuring state validity checker from file: " << endl;
  cout << "  " << config_filename << endl;

  // Open yaml file with configuration
  YAML::Node config = YAML::LoadFile(config_filename);

  // Load link dimensions and directions
  std::vector< std::vector<double> > link_dimensions_vector;
  link_dimensions_vector = config["state_validity_checker"]["uav_wp_manipulator"]["manipulator_link_dimensions"].as< std::vector< std::vector<double> > >();
  link_directions_ = config["state_validity_checker"]["uav_wp_manipulator"]["manipulator_link_directions"].as< std::vector<string> >();
  num_joints_ = link_dimensions_vector.size();
  // Check sizes
  if (link_directions_.size() != link_dimensions_vector.size()){
    cout << "ERROR: Link directions and dimensions must have the same size." << endl;
    cout << "  Size of link dimensions: " << link_dimensions_vector.size() << endl;
    cout << "  Size of link directions: " << link_directions_.size() << endl;
    exit(0);
  }
  link_dimensions_ = Eigen::MatrixXd(link_dimensions_vector.size(), link_dimensions_vector[0].size());
  for (int i=0; i<link_dimensions_vector.size(); i++){
    if (link_dimensions_vector[i].size() != 3){
      cout << "ERROR: All links must have exactly 3 dimensions." << endl;
      cout << "  Link " << i+1 << " has " << link_dimensions_vector[i].size() << " dimensions." << endl;
      exit(0);
    }
    else{
      link_dimensions_(i, 0) = link_dimensions_vector[i][0];
      link_dimensions_(i, 1) = link_dimensions_vector[i][1];
      link_dimensions_(i, 2) = link_dimensions_vector[i][2];
    }
  }

  // Get UAV dimensions
  std::vector<double> uav_dimensions;
  uav_dimensions = config["state_validity_checker"]["uav_wp_manipulator"]["uav_dimensions"].as< std::vector<double> >();
  for (int i=0; i<3; i++) uav_dimensions_(i) = uav_dimensions[i];

  // Get resolutions for uav and manipulator state sampling.
  uav_sampling_resolution_ = config["state_validity_checker"]["uav_wp_manipulator"]["checker_resolution"]["uav"].as<double>();
  manipulator_sampling_resolution_ = config["state_validity_checker"]["uav_wp_manipulator"]["checker_resolution"]["manipulator"].as<double>();

  // Get fixed transform between UAV and manipulator
  std::vector<double> temp_transf;
  temp_transf = config["state_validity_checker"]["uav_wp_manipulator"]["uav_manipulator_transform"].as< std::vector<double> >();
  // Set up a fixed transform between UAV and manipulator
  t_uav_manipulator_ = Eigen::Affine3d::Identity();
  Eigen::Matrix3d rot_uav_manipulator;
  rot_uav_manipulator = Eigen::AngleAxisd(temp_transf[5], Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(temp_transf[4],  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(temp_transf[3], Eigen::Vector3d::UnitX());
  t_uav_manipulator_.translate(Eigen::Vector3d(temp_transf[0], temp_transf[1], temp_transf[2]));
  t_uav_manipulator_.rotate(rot_uav_manipulator);

  // Get tool data
  use_tool_ = config["state_validity_checker"]["uav_wp_manipulator"]["tool"]["use_tool"].as<bool>();
  std::vector<double> tool_dimensions;
  tool_dimensions = config["state_validity_checker"]["uav_wp_manipulator"]["tool"]["dimensions"].as< std::vector<double> >();
  for (int i=0; i<3; i++) tool_dimensions_(i) = tool_dimensions[i];
  tool_direction_ = config["state_validity_checker"]["uav_wp_manipulator"]["tool"]["direction"].as<string>();
}

void UavWpManipulatorStateValidityChecker::testDirectKinematics()
{
  // This one will have to be constructed each time
  Eigen::Affine3d t_world_uav(Eigen::Affine3d::Identity());
  t_world_uav.translate(Eigen::Vector3d(0, 0, 1.0));
  Eigen::Matrix3d rot_uav;
  rot_uav = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(0,  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  t_world_uav.rotate(rot_uav);

  Eigen::Affine3d t_world_manipulator = t_world_uav*t_uav_manipulator_;
  //cout << t_uav_manipulator.rotation() << endl;

  /*std::vector<double> q{0.787, 0.787, 0.0, 0.0, 0.0};
  Eigen::Affine3d t_manipulator_end_effector;
  t_manipulator_end_effector = manipulator_.getEndEffectorPosition(q);

  Eigen::Affine3d t_world_end_effector = t_world_manipulator*t_manipulator_end_effector;
  cout << t_world_end_effector.translation() << endl;

  Eigen::Quaterniond quat(t_world_end_effector.rotation());
  cout << quat.coeffs() << endl;*/

  Eigen::VectorXd q(5);
  q << 0.7, 0.7, 0.2, 0.5, -0.7;
  std::vector<Eigen::Affine3d> link_positions;
  link_positions = kinematics_->getJointPositions(q);
  //cout << "link " << link_positions.size() << endl;
  // Link names are: world, base, link1, link2, link3, link4, link5,
  //  end_effector_base
  // Links coordinate systems are at their base so here is the plan. Construct
  //  a prism that describes the link and sample it spatially. Transform it
  //  with direct kinematics to describe the link and check for collision.
  Eigen::RowVectorXd px(8), py(8), pz(8), qx(8), qy(8), qz(8), qw(8);
  for (int i=0; i<link_positions.size(); i++){
    Eigen::Affine3d t_world_current_link = t_world_manipulator*link_positions[i];
    px(i) = t_world_current_link.translation()[0];
    py(i) = t_world_current_link.translation()[1];
    pz(i) = t_world_current_link.translation()[2];
    qx(i) = Eigen::Quaterniond(t_world_current_link.rotation()).x();
    qy(i) = Eigen::Quaterniond(t_world_current_link.rotation()).y();
    qz(i) = Eigen::Quaterniond(t_world_current_link.rotation()).z();
    qw(i) = Eigen::Quaterniond(t_world_current_link.rotation()).w();
  }
  /*cout << "x: " << px << endl;
  cout << "y: " << py << endl;
  cout << "z: " << pz << endl;
  cout << "qx: " << qx << endl;
  cout << "qy: " << qy << endl;
  cout << "qz: " << qz << endl;
  cout << "qw: " << qw << endl;*/
  //Eigen::Affine3d t_world_link1 = t_world_manipulator*link_positions[2];
  //Eigen::MatrixXd points_link1 = generatePrism(0.0340, 0.1225, 0.0285, 0.01, "y");
  //points_ = Eigen::MatrixXd(points_link1.rows(), points_link1.cols());

  points_ = Eigen::MatrixXd(0,3);
  for (int i=0; i<5; i++){
    Eigen::Affine3d t_world_link = t_world_manipulator*link_positions[i+2];
    Eigen::MatrixXd points_link = generatePrism(link_dimensions_(i,0),
      link_dimensions_(i,1), link_dimensions_(i,2),
      manipulator_sampling_resolution_, link_directions_[i]);
    //points_ = Eigen::MatrixXd(points_link.rows(), points_link.cols());
    double points_size = points_.rows();
    points_.conservativeResize(points_.rows() + points_link.rows(), 3);
    for (int j=0; j<points_link.rows(); j++){
      Eigen::Affine3d current_point(Eigen::Affine3d::Identity());
      //cout << (points_link.row(i)).transpose() << endl;
      //exit(0);
      current_point.translate(Eigen::Vector3d((points_link.row(j)).transpose()));
      points_.row(j+points_size) = ((t_world_link*current_point).translation()).transpose();
    }
  }

  Eigen::Affine3d t_world_link = t_world_uav;
  Eigen::MatrixXd points_link = generatePrism(uav_dimensions_(0),
    uav_dimensions_(1), uav_dimensions_(2), uav_sampling_resolution_, "n");
  //points_ = Eigen::MatrixXd(points_link.rows(), points_link.cols());
  double points_size = points_.rows();
  points_.conservativeResize(points_.rows() + points_link.rows(), 3);
  for (int j=0; j<points_link.rows(); j++){
    Eigen::Affine3d current_point(Eigen::Affine3d::Identity());
    //cout << (points_link.row(i)).transpose() << endl;
    //exit(0);
    current_point.translate(Eigen::Vector3d((points_link.row(j)).transpose()));
    points_.row(j+points_size) = ((t_world_link*current_point).translation()).transpose();
  }
  //exit(0);
}

bool UavWpManipulatorStateValidityChecker::isStateValid(Eigen::VectorXd state)
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

Eigen::MatrixXd UavWpManipulatorStateValidityChecker::generateValidityPoints(
  Eigen::VectorXd state)
{
  // First create transformation of the UAV with respect to the world.
  Eigen::Affine3d t_world_uav(Eigen::Affine3d::Identity());
  t_world_uav.translate(Eigen::Vector3d(state(0), state(1), state(2)));
  Eigen::Matrix3d rot_uav;
  rot_uav = Eigen::AngleAxisd(state(5), Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(state(4),  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitX());
  t_world_uav.rotate(rot_uav);

  // Create transformation from world to manipulator. Here we have UAV in world
  //  frame and we multiply it by fixed transformation of the manipulator in
  //  UAV frame.
  Eigen::Affine3d t_world_manipulator = t_world_uav*t_uav_manipulator_;

  // Extract joint references and get link positions.
  //cout << state << endl;
  //Eigen::VectorXd q(5);
  //q << state(4), state(5), state(6), state(7), state(8);
  
  Eigen::VectorXd q(num_joints_);
  for (int i=0; i<q.size(); i++) q(i) = state(i+6);

  std::vector<Eigen::Affine3d> link_positions;
  link_positions = kinematics_->getJointPositions(q);
  //cout << "Doing links" << endl;
  // Get sampled points to be checked in octomap
  points_ = Eigen::MatrixXd(0,3);
  for (int i=0; i<q.size(); i++){
    Eigen::Affine3d t_world_link = t_world_manipulator*link_positions[i+2];
    Eigen::MatrixXd points_link = generatePrism(link_dimensions_(i,0),
      link_dimensions_(i,1), link_dimensions_(i,2),
      manipulator_sampling_resolution_, link_directions_[i]);
    //points_ = Eigen::MatrixXd(points_link.rows(), points_link.cols());
    double points_size = points_.rows();
    points_.conservativeResize(points_.rows() + points_link.rows(), 3);
    for (int j=0; j<points_link.rows(); j++){
      Eigen::Affine3d current_point(Eigen::Affine3d::Identity());
      //cout << (points_link.row(i)).transpose() << endl;
      //exit(0);
      current_point.translate(Eigen::Vector3d((points_link.row(j)).transpose()));
      points_.row(j+points_size) = ((t_world_link*current_point).translation()).transpose();
    }
  }

  // Sample tool points if requested. Tool is going to be an extension of last
  // link
  //cout << "Doing Tool" << endl;
  if (use_tool_ == true){
    Eigen::Affine3d t_world_link = t_world_manipulator*link_positions[q.size()+1];
    Eigen::MatrixXd points_link = generatePrism(tool_dimensions_(0),
      tool_dimensions_(1), tool_dimensions_(2),
      manipulator_sampling_resolution_, tool_direction_);
    //points_ = Eigen::MatrixXd(points_link.rows(), points_link.cols());
    double points_size = points_.rows();
    points_.conservativeResize(points_.rows() + points_link.rows(), 3);
    for (int j=0; j<points_link.rows(); j++){
      Eigen::Affine3d current_point(Eigen::Affine3d::Identity());
      //cout << (points_link.row(i)).transpose() << endl;
      //exit(0);
      current_point.translate(Eigen::Vector3d((points_link.row(j)).transpose()));
      points_.row(j+points_size) = ((t_world_link*current_point).translation()).transpose();
    }
  }
  //cout << "Tool done" << endl;

  Eigen::Affine3d t_world_link = t_world_uav;
  Eigen::MatrixXd points_link = generatePrism(uav_dimensions_(0),
    uav_dimensions_(1), uav_dimensions_(2), uav_sampling_resolution_, "n");
  //points_ = Eigen::MatrixXd(points_link.rows(), points_link.cols());
  double points_size = points_.rows();
  points_.conservativeResize(points_.rows() + points_link.rows(), 3);
  //cout << "Adding UAV" << endl;
  for (int j=0; j<points_link.rows(); j++){
    Eigen::Affine3d current_point(Eigen::Affine3d::Identity());
    //cout << (points_link.row(i)).transpose() << endl;
    //exit(0);
    current_point.translate(Eigen::Vector3d((points_link.row(j)).transpose()));
    points_.row(j+points_size) = ((t_world_link*current_point).translation()).transpose();
  }
  //cout << "UAV done" << endl;
  //cout << points_.rows() << " " << points_.cols() << endl;

  return points_;
}

Eigen::MatrixXd UavWpManipulatorStateValidityChecker::generatePrism(
  double x, double y, double z, double resolution, string direction)
{
  // First get number of points which define the prism after sampling.
  int px = x/resolution;
  int py = y/resolution;
  int pz = z/resolution;

  // Get link offsets
  double offset_x = x/2.0;
  double offset_y = y/2.0;
  double offset_z = z/2.0;
  // Set offset that coincides with direction of the link to zero. If direction
  //  is not x, y or z then offset all links.
  if (direction == "x") offset_x = 0.0;
  else if (direction == "y") offset_y = 0.0;
  else if (direction == "z") offset_z = 0.0;

  // Get total number of points. Expand this part by one point because
  //  otherwise we won't get edge points.
  int p = (px+1)*(py+1)*(pz+1);
  // Allocate memory for matrix defining prism.
  Eigen::MatrixXd points(p, 3);

  // Go through all combinations and sample prism along all dimensions.
  int index = 0;
  for (int i=0; i<px+1; i++){
    for (int j=0; j<py+1; j++){
      for (int k=0; k<pz+1; k++){
        // Offset x and y axes in order to "center" the prism. z does not need
        //  to be centered since coordinate system describing the link lies
        //  at the base of the link.
        points(index, 0) = double(i)*x/double(px) - offset_x;
        points(index, 1) = double(j)*y/double(py) - offset_y;
        points(index, 2) = double(k)*z/double(pz) - offset_z;
        index++;
      }
    }
  }

  //points.col(1).swap(points.col(2));

  return points;
}
