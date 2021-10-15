#include <larics_motion_planning/MultipleManipulatorsStateValidityChecker.h>

MultipleManipulatorsStateValidityChecker::MultipleManipulatorsStateValidityChecker(
  string config_filename, shared_ptr<MapInterface> map,
  shared_ptr<KinematicsInterface> kinematics)
{
  map_ = map;
  // Immediately cast into multiple manipulator kinematics. This is not pretty
  // but it is necessary to get single manipulator joint positions.
  kinematics_ = dynamic_pointer_cast<MultipleManipulatorsKinematics>(kinematics);

  string username = "/home/";
  username = username + getenv("USER") + "/";
  configureFromFile(username + config_filename);

  //testDirectKinematics();
}

bool MultipleManipulatorsStateValidityChecker::configureFromFile(
  string config_filename)
{
  cout << "Configuring multiple manipulators state validity checker from file: " << endl;
  cout << "  " << config_filename << endl;

  // Open yaml file with configuration
  YAML::Node config = YAML::LoadFile(config_filename);
  
  // Get the number of manipulators. This should be the same number as in 
  // kinematics interface.
  n_manipulators_ = config["state_validity_checker"]["multiple_manipulators"].size();
  // Check if there is the same number of manipulators in both state validity
  // checker and kinematics
  if (n_manipulators_ != config["kinematics"]["multiple_manipulators"].size()){
    cout << "ERROR: Number of manipulators is different in kinematics." << endl;
    cout << "  This occured in MultipleManipulatorsStateValidityChecker." << endl;
    cout << "  Number of manipulators in state validity checker: " << n_manipulators_ << endl;
    cout << "  Number of manipulators in kinematics: ";
    cout << config["kinematics"]["multiple_manipulators"].size() << endl;
    exit(0);
  }

  // Go through all manipulators and configure
  for (int i=0; i<n_manipulators_; i++){
    std::vector< std::vector<double> > current_dimensions_vector;
    current_dimensions_vector = config["state_validity_checker"]["multiple_manipulators"][i]["manipulator_link_dimensions"].as< std::vector< std::vector<double> > >();
    link_directions_ = config["state_validity_checker"]["multiple_manipulators"][i]["manipulator_link_directions"].as< std::vector<string> >();
    // Check sizes
    if (link_directions_.size() != current_dimensions_vector.size()){
      cout << "ERROR: Link directions and dimensions must have the same size." << endl;
      cout << "  This occured in MultipleManipulatorsStateValidityChecker." << endl;
      cout << "  Size of manipulator " << i << " link dimensions: " << current_dimensions_vector.size() << endl;
      cout << "  Size of manipulator " << i << " link directions: " << link_directions_.size() << endl;
      exit(0);
    }

    // If sizes are okay, load data. First directions
    link_directions_vector_.push_back(link_directions_);
    // And next dimensions, which are matrices in vector.
    Eigen::MatrixXd link_dimensions;
    link_dimensions = Eigen::MatrixXd(current_dimensions_vector.size(), current_dimensions_vector[0].size());
    for (int j=0; j<current_dimensions_vector.size(); j++){
      if (current_dimensions_vector[j].size() != 3){
        cout << "ERROR: All links must have exactly 3 dimensions." << endl;
        cout << "  This occured in MultipleManipulatorsStateValidityChecker." << endl;
        cout << "  Link " << j+1 << " of manipulator " << i << " has ";
        cout << current_dimensions_vector[j].size() << " dimensions." << endl;
        exit(0);
      }
      else{
        link_dimensions(j, 0) = current_dimensions_vector[j][0];
        link_dimensions(j, 1) = current_dimensions_vector[j][1];
        link_dimensions(j, 2) = current_dimensions_vector[j][2];
      }
    }
    // Push dimensions into a vector
    link_dimensions_vector_.push_back(link_dimensions);

    // Base dimensions
    std::vector<double> base_vector;
    Eigen::Vector3d current_base_dimensions;
    // Data from yaml is loaded as std vector
    base_vector = config["state_validity_checker"]["multiple_manipulators"][i]["base_dimensions"].as< std::vector<double> >();
    // And then it is converted into Eigen::Vector3d and pushed into vector
    // of base transforms.
    for (int j=0; j<3; j++){
      current_base_dimensions(j) = base_vector[j];
    }
    base_dimensions_.push_back(current_base_dimensions);

    // Get sampling resolutions
    uav_sampling_resolution_ = config["state_validity_checker"]["multiple_manipulators"][i]["checker_resolution"]["base"].as<double>();
    manipulator_sampling_resolution_ = config["state_validity_checker"]["multiple_manipulators"][i]["checker_resolution"]["manipulator"].as<double>();
    base_sampling_resolutions_.push_back(uav_sampling_resolution_);
    manipulator_sampling_resolutions_.push_back(manipulator_sampling_resolution_);

    // Get fixed transform between UAV and manipulator. This is important
    // because the inverse kinematics operates in coordinate system of the
    // manipulator. It means each end-effector configuration in global system
    // has to be transformed to the manipulator system.
    std::vector<double> temp_transf;
    temp_transf = config["state_validity_checker"]["multiple_manipulators"][i]["base_manipulator_transform"].as< std::vector<double> >();
    // Set up a fixed transform between UAV and manipulator
    t_uav_manipulator_ = Eigen::Affine3d::Identity();
    Eigen::Matrix3d rot_uav_manipulator;
    rot_uav_manipulator = Eigen::AngleAxisd(temp_transf[5], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(temp_transf[4],  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(temp_transf[3], Eigen::Vector3d::UnitX());
    t_uav_manipulator_.translate(Eigen::Vector3d(temp_transf[0], temp_transf[1], temp_transf[2]));
    t_uav_manipulator_.rotate(rot_uav_manipulator);
    t_uav_manipulator_vector_.push_back(t_uav_manipulator_);

    // Get tool data
    use_tool_ = config["state_validity_checker"]["multiple_manipulators"][i]["tool"]["use_tool"].as<bool>();
    std::vector<double> tool_dimensions;
    tool_dimensions = config["state_validity_checker"]["multiple_manipulators"][i]["tool"]["dimensions"].as< std::vector<double> >();
    for (int i=0; i<3; i++){
      tool_dimensions_(i) = tool_dimensions[i];
    }
    tool_direction_ = config["state_validity_checker"]["multiple_manipulators"][i]["tool"]["direction"].as<string>();
    use_tool_vector_.push_back(use_tool_);
    tool_dimensions_vector_.push_back(tool_dimensions_);
    tool_direction_vector_.push_back(tool_direction_);

    // Get the indexes of each manipulator.
    int start, end;
    start = config["state_validity_checker"]["multiple_manipulators"][i]["indexes"]["start"].as<int>();
    end = config["state_validity_checker"]["multiple_manipulators"][i]["indexes"]["end"].as<int>();
    start_indexes_.push_back(start);
    end_indexes_.push_back(end);

    // Get number of dof in base
    int base_dof;
    base_dof = config["state_validity_checker"]["multiple_manipulators"][i]["base_dof"].as<int>();
    base_dof_.push_back(base_dof);
  }
}

void MultipleManipulatorsStateValidityChecker::testDirectKinematics()
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

bool MultipleManipulatorsStateValidityChecker::isStateValid(Eigen::VectorXd state)
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

// TODO: Mislim da ce trebati dodati koji se indeksi koriste za koji manipulator.
// Iz toga sloziti state i onda pozvati ovu funkciju. Osim toga mislim da bi
// bilo dobro napomenuti da je baza uvijek 6DoF i manipulator jos n DoF. S time
// da baza ne mora nuzno biti pokretna.
Eigen::MatrixXd MultipleManipulatorsStateValidityChecker::generateSingleManipulatorValidityPoints(
  Eigen::VectorXd state, int id)
{
  Eigen::MatrixXd points(0,3);
  // First create transformation of the base with respect to the world. This
  // assumes 6DoF base and n DoF manipulator.
  Eigen::Affine3d t_world_base(Eigen::Affine3d::Identity());
  t_world_base.translate(Eigen::Vector3d(state(0), state(1), state(2)));
  Eigen::Matrix3d rot_base;
  rot_base = Eigen::AngleAxisd(state(5), Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(state(4),  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitX());
  t_world_base.rotate(rot_base);

  // Create transformation from world to manipulator. Here we have base in world
  // frame and we multiply it by fixed transformation of the manipulator in
  // base frame.
  Eigen::Affine3d t_world_manipulator = t_world_base*t_uav_manipulator_vector_[id];

  // Extract joint references and get link positions.
  //cout << state << endl;
  //Eigen::VectorXd q(5);
  //q << state(4), state(5), state(6), state(7), state(8);
  
  Eigen::VectorXd q(state.size()-base_dof_[id]);
  for (int i=0; i<q.size(); i++) q(i) = state(i+base_dof_[id]);

  std::vector<Eigen::Affine3d> link_positions;
  link_positions = kinematics_->getSingleManipulatorJointPositions(q, id);
  //cout << "Doing links" << endl;
  // Get sampled points to be checked in octomap
  points = Eigen::MatrixXd(0,3);
  for (int i=0; i<q.size(); i++){
    Eigen::Affine3d t_world_link = t_world_manipulator*link_positions[i+2];
    Eigen::MatrixXd points_link = generatePrism(link_dimensions_vector_[id](i,0),
      link_dimensions_vector_[id](i,1), link_dimensions_vector_[id](i,2),
      manipulator_sampling_resolutions_[id], link_directions_vector_[id][i]);
    //points = Eigen::MatrixXd(points_link.rows(), points_link.cols());
    double points_size = points.rows();
    points.conservativeResize(points.rows() + points_link.rows(), 3);
    for (int j=0; j<points_link.rows(); j++){
      Eigen::Affine3d current_point(Eigen::Affine3d::Identity());
      //cout << (points_link.row(i)).transpose() << endl;
      //exit(0);
      current_point.translate(Eigen::Vector3d((points_link.row(j)).transpose()));
      points.row(j+points_size) = ((t_world_link*current_point).translation()).transpose();
    }
  }

  // Sample tool points if requested. Tool is going to be an extension of last
  // link
  //cout << "Doing Tool" << endl;
  if (use_tool_ == true){
    Eigen::Affine3d t_world_link = t_world_manipulator*link_positions[q.size()+1];
    Eigen::MatrixXd points_link = generatePrism(tool_dimensions_vector_[id](0),
      tool_dimensions_vector_[id](1), tool_dimensions_vector_[id](2),
      manipulator_sampling_resolutions_[id], tool_direction_vector_[id]);
    //points = Eigen::MatrixXd(points_link.rows(), points_link.cols());
    double points_size = points.rows();
    points.conservativeResize(points.rows() + points_link.rows(), 3);
    for (int j=0; j<points_link.rows(); j++){
      Eigen::Affine3d current_point(Eigen::Affine3d::Identity());
      //cout << (points_link.row(i)).transpose() << endl;
      //exit(0);
      current_point.translate(Eigen::Vector3d((points_link.row(j)).transpose()));
      points.row(j+points_size) = ((t_world_link*current_point).translation()).transpose();
    }
  }
  //cout << "Tool done" << endl;

  Eigen::Affine3d t_world_link = t_world_base;
  Eigen::MatrixXd points_link = generatePrism(base_dimensions_[id](0),
    base_dimensions_[id](1), base_dimensions_[id](2), base_sampling_resolutions_[id], "n");
  //points = Eigen::MatrixXd(points_link.rows(), points_link.cols());
  double points_size = points.rows();
  points.conservativeResize(points.rows() + points_link.rows(), 3);
  //cout << "Adding UAV" << endl;
  for (int j=0; j<points_link.rows(); j++){
    Eigen::Affine3d current_point(Eigen::Affine3d::Identity());
    //cout << (points_link.row(i)).transpose() << endl;
    //exit(0);
    current_point.translate(Eigen::Vector3d((points_link.row(j)).transpose()));
    points.row(j+points_size) = ((t_world_link*current_point).translation()).transpose();
  }
  //cout << "UAV done" << endl;
  //cout << points.rows() << " " << points.cols() << endl;

  return points;
}

Eigen::MatrixXd MultipleManipulatorsStateValidityChecker::generateValidityPoints(
  Eigen::VectorXd state)
{
  points_ = Eigen::MatrixXd(0,3);
  cout << "init: " << points_.size() << endl;
  for (int i=0; i<n_manipulators_; i++){
    // Extract current manipulator state based on indexes provided in the
    // config file.
    Eigen::VectorXd current_state(end_indexes_[i]-start_indexes_[i]+1);
    current_state = state.block(
      start_indexes_[i], 0, end_indexes_[i]-start_indexes_[i]+1, 1);//end_indexes_[i]-start_indexes_[i]);

    // Generate points based on the current manipulator state and concatenate
    // them into all points.
    Eigen::MatrixXd current_manipulator_points;
    current_manipulator_points = 
      this->generateSingleManipulatorValidityPoints(current_state, i);
    cout << "cmp: " << current_manipulator_points.size() << endl;
    points_.conservativeResize(
      points_.rows() + current_manipulator_points.rows(), 3);
    points_.block(points_.rows()-current_manipulator_points.rows(), 0, 
      current_manipulator_points.rows() , 3) = current_manipulator_points;
    cout << "man: " << points_.size() << endl;
  }
  return points_;
}

Eigen::MatrixXd MultipleManipulatorsStateValidityChecker::generatePrism(
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
