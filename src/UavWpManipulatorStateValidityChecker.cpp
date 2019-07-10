#include <UavWpManipulatorStateValidityChecker.h>

UavWpManipulatorStateValidityChecker::UavWpManipulatorStateValidityChecker(
  string config_filename, shared_ptr<MapInterface> map)
{
  map_ = map;

  string robot_model_name = "wp_manipulator";
  string joint_group_name = "wp_manipulator_arm";

  manipulator_.setManipulatorName(robot_model_name, joint_group_name);
  manipulator_.LoadParameters(
    "/home/antun/catkin_ws/src/aerial_manipulators/aerial_manipulators_control/config/wp_manipulator_dh_parameters.yaml");
  manipulator_.init();

  link_dimensions_ = Eigen::MatrixXd(5, 3);
  link_dimensions_ << 0.0340, 0.1225, 0.0285,
                      0.1365, 0.0340, 0.0285,
                      0.0755, 0.0285, 0.0340,
                      0.0725, 0.0285, 0.0340,
                      0.0453, 0.0285, 0.0340;
  link_directions_ = {"y", "x", "x", "x", "x"};
  cout << "link dimensions:" << endl << link_dimensions_ << endl;
  cout << "link directions: " << link_directions_[0] << endl;

  testDirectKinematics();
}

void UavWpManipulatorStateValidityChecker::testDirectKinematics()
{
  //Eigen::VectorXd state << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // This one is fixed transformation
  Eigen::Affine3d t_uav_manipulator(Eigen::Affine3d::Identity());
  Eigen::Matrix3d rot_uav_manipulator;
  rot_uav_manipulator = Eigen::AngleAxisd(1.0*M_PI, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(0*M_PI,  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX());
  t_uav_manipulator.translate(Eigen::Vector3d(0, 0, 0.075));
  t_uav_manipulator.rotate(rot_uav_manipulator);
  
  // This one will have to be constructed each time
  Eigen::Affine3d t_world_uav(Eigen::Affine3d::Identity());
  t_world_uav.translate(Eigen::Vector3d(0, 0, 1.0));

  Eigen::Affine3d t_world_manipulator = t_world_uav*t_uav_manipulator;
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
  link_positions = manipulator_.getLinkPositions(q);
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
      link_dimensions_(i,1), link_dimensions_(i,2), 0.01, link_directions_[i]);
    //points_ = Eigen::MatrixXd(points_link.rows(), points_link.cols());
    double points_size = points_.rows();
    cout << "Size before: " << points_size << endl;
    points_.conservativeResize(points_.rows() + points_link.rows(), 3);
    cout << "Size after: " << points_.rows() << endl;
    for (int j=0; j<points_link.rows(); j++){
      Eigen::Affine3d current_point(Eigen::Affine3d::Identity());
      //cout << (points_link.row(i)).transpose() << endl;
      //exit(0);
      current_point.translate(Eigen::Vector3d((points_link.row(j)).transpose()));
      points_.row(j+points_size) = ((t_world_link*current_point).translation()).transpose();
    }
  }

  Eigen::Affine3d t_world_link = t_world_uav;
  Eigen::MatrixXd points_link = generatePrism(0.4, 0.4, 0.17, 0.03, "n");
  //points_ = Eigen::MatrixXd(points_link.rows(), points_link.cols());
  double points_size = points_.rows();
  cout << "Size before: " << points_size << endl;
  points_.conservativeResize(points_.rows() + points_link.rows(), 3);
  cout << "Size after: " << points_.rows() << endl;
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
  return map_->isStateValid(state);
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