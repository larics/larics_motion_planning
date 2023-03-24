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
  string username = "/home/";
  username = username + getenv("USER") + "/";
  string kinematics_config_filename = username + config["global_planner"]["kinematics_config_file"].as<string>();
  YAML::Node kinematics_config = YAML::LoadFile(kinematics_config_filename);
  
  // Get the number of manipulators. This should be the same number as in 
  // kinematics interface.
  n_manipulators_ = config["state_validity_checker"]["multiple_manipulators"].size();
  // Check if there is the same number of manipulators in both state validity
  // checker and kinematics
  if (n_manipulators_ != kinematics_config["kinematics"]["multiple_manipulators"].size()){
    cout << "ERROR: Number of manipulators is different in kinematics." << endl;
    cout << "  This occured in MultipleManipulatorsStateValidityChecker." << endl;
    cout << "  Number of manipulators in state validity checker: " << n_manipulators_ << endl;
    cout << "  Number of manipulators in kinematics: ";
    cout << kinematics_config["kinematics"]["multiple_manipulators"].size() << endl;
    exit(0);
  }

  // Go through all manipulators and configure
  for (int i=0; i<n_manipulators_; i++){
    std::vector< std::vector<double> > current_dimensions_vector;
    current_dimensions_vector = config["state_validity_checker"]["multiple_manipulators"][i]["manipulator_link_dimensions"].as< std::vector< std::vector<double> > >();
    std::vector<string> link_directions;
    link_directions = config["state_validity_checker"]["multiple_manipulators"][i]["manipulator_link_directions"].as< std::vector<string> >();
    // Check sizes
    if (link_directions.size() != current_dimensions_vector.size()){
      cout << "ERROR: Link directions and dimensions must have the same size." << endl;
      cout << "  This occured in MultipleManipulatorsStateValidityChecker." << endl;
      cout << "  Size of manipulator " << i << " link dimensions: " << current_dimensions_vector.size() << endl;
      cout << "  Size of manipulator " << i << " link directions: " << link_directions.size() << endl;
      exit(0);
    }

    // If sizes are okay, load data. First directions
    link_directions_vector_.push_back(link_directions);
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
    double base_sampling_resolution, manipulator_sampling_resolution;
    base_sampling_resolution = config["state_validity_checker"]["multiple_manipulators"][i]["checker_resolution"]["base"].as<double>();
    manipulator_sampling_resolution = config["state_validity_checker"]["multiple_manipulators"][i]["checker_resolution"]["manipulator"].as<double>();
    base_sampling_resolutions_.push_back(base_sampling_resolution);
    manipulator_sampling_resolutions_.push_back(manipulator_sampling_resolution);

    // Get fixed transform between UAV and manipulator. This is important
    // because the inverse kinematics operates in coordinate system of the
    // manipulator. It means each end-effector configuration in global system
    // has to be transformed to the manipulator system.
    std::vector<double> temp_transf;
    temp_transf = config["state_validity_checker"]["multiple_manipulators"][i]["base_manipulator_transform"].as< std::vector<double> >();
    // Set up a fixed transform between UAV and manipulator
    Eigen::Affine3d t_base_manipulator;
    t_base_manipulator = Eigen::Affine3d::Identity();
    Eigen::Matrix3d rot_uav_manipulator;
    rot_uav_manipulator = Eigen::AngleAxisd(temp_transf[5], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(temp_transf[4],  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(temp_transf[3], Eigen::Vector3d::UnitX());
    t_base_manipulator.translate(Eigen::Vector3d(temp_transf[0], temp_transf[1], temp_transf[2]));
    t_base_manipulator.rotate(rot_uav_manipulator);
    t_base_manipulator_vector_.push_back(t_base_manipulator);

    // Get tool data
    bool use_tool;
    use_tool = config["state_validity_checker"]["multiple_manipulators"][i]["tool"]["use_tool"].as<bool>();
    std::vector<double> tool_dimensions_config;
    Eigen::Vector3d tool_dimensions;
    tool_dimensions_config = config["state_validity_checker"]["multiple_manipulators"][i]["tool"]["dimensions"].as< std::vector<double> >();
    for (int i=0; i<3; i++){
      tool_dimensions(i) = tool_dimensions_config[i];
    }
    string tool_direction;
    tool_direction = config["state_validity_checker"]["multiple_manipulators"][i]["tool"]["direction"].as<string>();
    use_tool_vector_.push_back(use_tool);
    tool_dimensions_vector_.push_back(tool_dimensions);
    tool_direction_vector_.push_back(tool_direction);

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

  // If the object planner is selected, load it.
  try{
    object_planner_is_used_ = config["state_validity_checker"]["object_checker"]["is_used"].as<bool>();
    // Object will be a prism with resolution from config
    object_planner_resolution_ = config["state_validity_checker"]["object_checker"]["resolution"].as<double>();
    // Object dimensions
    std::vector<double> object_dimensions_config;
    object_dimensions_config = config["state_validity_checker"]["object_checker"]["dimensions"].as< std::vector<double> >();
    for (int i=0; i<3; i++){
      object_dimensions_(i) = object_dimensions_config[i];
    }
  }
  catch (...){
    string ys = "\033[0;33m";
    string ye = "\033[0m";
    cout << ys << "[MultipleManipulatorsStateValidityChecker] Object config." << ye << endl;
    cout << ys << "  Something wrong or missing in the state_validity_checker:object_checker config." << ye << endl;
    cout << ys << "  Not using object planner." << ye << endl;
    cout << ys << "  Ignore this message if you are not using multiple manipulators." << ye << endl;
    object_planner_is_used_ = false;
  }
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
  Eigen::Affine3d t_world_manipulator = t_world_base*t_base_manipulator_vector_[id];

  // Extract joint references and get link positions.
  //cout << state << endl;
  //Eigen::VectorXd q(5);
  //q << state(4), state(5), state(6), state(7), state(8);
  
  // Set up the manipulator q.
  Eigen::VectorXd q(state.size()-base_dof_[id]);
  for (int i=0; i<q.size(); i++) q(i) = state(i+base_dof_[id]);

  // Get positions of each link. If I remember correctly, it actually returns
  // joint positions, and link is directed along link_directions_[id] axis.
  std::vector<Eigen::Affine3d> link_positions;
  link_positions = kinematics_->getSingleManipulatorJointPositions(q, id);

  // Get sampled points to be checked in octomap
  points = Eigen::MatrixXd(0,3);
  for (int i=0; i<q.size(); i++){
    // This is property of the MoveIt kinematics which returns two "fake" joints.
    // That's why everything is offset by 2. This will also transform the link
    // to the world coordinate system.
    Eigen::Affine3d t_world_link = t_world_manipulator*link_positions[i+2];

    // Generate prism based on dimensions from the config file, spatial
    // sampling resolution and direction of the link.
    Eigen::MatrixXd points_link = generatePrism(link_dimensions_vector_[id](i,0),
      link_dimensions_vector_[id](i,1), link_dimensions_vector_[id](i,2),
      manipulator_sampling_resolutions_[id], link_directions_vector_[id][i]);

    // Use conservative resize to ensure all previous rows stay in the matrix.
    double points_size = points.rows();
    points.conservativeResize(points.rows() + points_link.rows(), 3);
    // Translate each validity point in local coordinate system of a joint, and
    // then transform it to the world coordinate system and add it into the
    // points container.
    for (int j=0; j<points_link.rows(); j++){
      Eigen::Affine3d current_point(Eigen::Affine3d::Identity());
      current_point.translate(Eigen::Vector3d((points_link.row(j)).transpose()));
      points.row(j+points_size) = ((t_world_link*current_point).translation()).transpose();
    }
  }

  // Sample tool points if requested. Tool is going to be an extension of last
  // link
  if (use_tool_vector_[id] == true){
    // Get link position in the world frame.
    Eigen::Affine3d t_world_link = t_world_manipulator*link_positions[q.size()+1];
    // Same as before, generate a prism of certain dimensions, with predefined
    // resolution and direction.
    Eigen::MatrixXd points_link = generatePrism(tool_dimensions_vector_[id](0),
      tool_dimensions_vector_[id](1), tool_dimensions_vector_[id](2),
      manipulator_sampling_resolutions_[id], tool_direction_vector_[id]);

    // Again, conservative resize to keep all previous points in the container.
    // Standard resize does not guarantee that.
    double points_size = points.rows();
    points.conservativeResize(points.rows() + points_link.rows(), 3);
    // Translate each validity point in local coordinate system of a joint, and
    // then transform it to the world coordinate system and add it into the
    // points container.
    for (int j=0; j<points_link.rows(); j++){
      Eigen::Affine3d current_point(Eigen::Affine3d::Identity());
      current_point.translate(Eigen::Vector3d((points_link.row(j)).transpose()));
      points.row(j+points_size) = ((t_world_link*current_point).translation()).transpose();
    }
  }

  // Also sample the base of the manipulator.
  Eigen::Affine3d t_world_link = t_world_base;
  Eigen::MatrixXd points_link = generatePrism(base_dimensions_[id](0),
    base_dimensions_[id](1), base_dimensions_[id](2), base_sampling_resolutions_[id], "n");
  double points_size = points.rows();
  points.conservativeResize(points.rows() + points_link.rows(), 3);
  for (int j=0; j<points_link.rows(); j++){
    Eigen::Affine3d current_point(Eigen::Affine3d::Identity());
    current_point.translate(Eigen::Vector3d((points_link.row(j)).transpose()));
    points.row(j+points_size) = ((t_world_link*current_point).translation()).transpose();
  }

  return points;
}

Eigen::MatrixXd MultipleManipulatorsStateValidityChecker::generateValidityPoints(
  Eigen::VectorXd state)
{
  points_ = Eigen::MatrixXd(0,3);

  Eigen::VectorXd full_state;

  // If object is used, first generate its points.
  if (object_planner_is_used_ == true){
    points_ = generateObjectValidityPoints(state);

    // IMPORTANT!!!
    // After generating object points, full state of the manipulator is required
    // and then the rest of this function can be called. The remainder of this function can add 
    // the additional points, if the full state is properly computed.
    for (int i=0; i<n_manipulators_; i++){
      Eigen::VectorXd current_q;
      current_q = kinematics_->getSingleManipulatorStateFromObjectState(state, i);
      full_state.conservativeResize(full_state.rows() + current_q.rows(), 1);
      full_state.block(full_state.rows() - current_q.rows(), 0, 
        current_q.rows(), 1) = current_q;
    }
  }
  else{
    full_state = state;
  }
  //cout << full_state << endl;
  //full_state << -0.55,0.018,-0.166,0,0,0,0.785398, 0.785398, 0.557, -0.861, 0.304, 0.553,-0.018,-0.166,0,0,3.141592654,0.785398, 0.785398, 0.557, -0.861, 0.304;
  //cout << full_state << endl;

  for (int i=0; i<n_manipulators_; i++){
    // Extract current manipulator state based on indexes provided in the
    // config file.
    Eigen::VectorXd current_state(end_indexes_[i]-start_indexes_[i]+1);
    current_state = full_state.block(
      start_indexes_[i], 0, end_indexes_[i]-start_indexes_[i]+1, 1);//end_indexes_[i]-start_indexes_[i]);

    // Generate points based on the current manipulator state and concatenate
    // them into all points.
    Eigen::MatrixXd current_manipulator_points;
    current_manipulator_points = 
      this->generateSingleManipulatorValidityPoints(current_state, i);
    points_.conservativeResize(
      points_.rows() + current_manipulator_points.rows(), 3);
    points_.block(points_.rows()-current_manipulator_points.rows(), 0, 
      current_manipulator_points.rows() , 3) = current_manipulator_points;
  }
  return points_;
}

Eigen::MatrixXd MultipleManipulatorsStateValidityChecker::generateObjectValidityPoints(
  Eigen::VectorXd state)
{
  // First generate object points in a coordinate system at its center.
  points_ = Eigen::MatrixXd(0,3);
  points_ = generateObject(object_dimensions_, object_planner_resolution_);

  // Next, transform points to state, which is in world frame.
  Eigen::Affine3d t_world_object(Eigen::Affine3d::Identity());
  t_world_object.translate(Eigen::Vector3d(state(0), state(1), state(2)));
  Eigen::Matrix3d rot_object;
  rot_object = Eigen::AngleAxisd(state(5), Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(state(4),  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitX());
  t_world_object.rotate(rot_object);

  for (int i=0; i<points_.rows(); i++){
    Eigen::Affine3d current_point(Eigen::Affine3d::Identity());
    current_point.translate(Eigen::Vector3d((points_.row(i)).transpose()));
    points_.row(i) = ((t_world_object*current_point).translation()).transpose();
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

Eigen::MatrixXd MultipleManipulatorsStateValidityChecker::generateObject(
  Eigen::Vector3d dimensions, double resolution)
{
  double x = dimensions(0);
  double y = dimensions(1);
  double z = dimensions(2);

  // First get number of points which define the prism after sampling.
  int px = x/resolution;
  int py = y/resolution;
  int pz = z/resolution;

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
        // Offset x and y axes in order to "center" the prism.
        points(index, 0) = double(i)*x/double(px) - x/2.0;
        points(index, 1) = double(j)*y/double(py) - y/2.0;
        points(index, 2) = double(k)*z/double(pz) - z/2.0;
        index++;
      }
    }
  }

  return points;
}
