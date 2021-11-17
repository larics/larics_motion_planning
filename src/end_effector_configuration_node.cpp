#include <iostream>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <larics_motion_planning/KinematicsInterface.h>
#include <larics_motion_planning/WpManipulatorKinematics.h>
#include <larics_motion_planning/MultipleManipulatorsKinematics.h>

#include <Eigen/Eigen>

using namespace std;

class EndEffectorConfiguration
{
public:
  EndEffectorConfiguration()
  {
    // Strings for loading files
    string username = "/home/";
    username = username + getenv("USER") + "/";

    // Init node and get params
    ros::NodeHandle nh_private = ros::NodeHandle("~");
    nh_private.param("rate", rate_, int(100));

    // Get namespaces from the trajectory handler files
    string trajectory_handler_filename;
    nh_private.param("trajectory_handler_config_file", trajectory_handler_filename, 
      string("catkin_ws/src/larics_motion_planning/config/multiple_manipulators/two_uavs_and_wp_manipulators.yaml"));
    size_t found = trajectory_handler_filename.find(username);
    if (found == string::npos){
      trajectory_handler_filename = username + trajectory_handler_filename;
    }
    YAML::Node trajectory_handler_config = YAML::LoadFile(trajectory_handler_filename);
    n_manipulators_ = trajectory_handler_config["trajectory_handler"].size();
    // Get namespaces and create topics
    for (int i=0; i<n_manipulators_; i++){
      string ns = trajectory_handler_config["trajectory_handler"][i]["namespace"].as<string>();
      string state_topic_name = "/" + ns + "/end_effector_configuration/end_effector/state";
      string reference_topic_name = "/" + ns + "/end_effector_configuration/end_effector/reference";
      ros::Publisher temp_state_pub, temp_reference_pub;
      temp_state_pub = nh_.advertise<geometry_msgs::PoseStamped>(
        state_topic_name, 1);
      temp_reference_pub = nh_.advertise<geometry_msgs::PoseStamped>(
        reference_topic_name, 1);
      ee_state_pubs_.push_back(temp_state_pub);
      ee_reference_pubs_.push_back(temp_reference_pub);
    }


    //ee_state_pub_ = nh_.advertise<nav_msgs::Path>(
    //  "end_effector_configuration/end_effector/states", 1);
    //ee_reference_pub_ = nh_.advertise<nav_msgs::Path>(
    //  "end_effector_configuration/end_effector/references", 1);

    // Load kinematics interface from yaml file
    string config_filename;
    nh_private.param("global_planner_config_file", config_filename, 
      string("catkin_ws/src/larics_motion_planning/config/cooperative_transport_uav_wp_manipulator.yaml"));
    cout << "Configuring end effector node from file:" << endl;
    cout << "  " << config_filename << endl;
    // Open yaml file with configuration
    YAML::Node config = YAML::LoadFile(config_filename);
    kinematics_ = make_shared<MultipleManipulatorsKinematics>(
      config["global_planner"]["kinematics_config_file"].as<string>());

    // Open the config file that contains kinematics configuration
    YAML::Node state_validity_config = YAML::LoadFile(username +
      config["global_planner"]["state_validity_checker_config_file"].as<string>());
    if (n_manipulators_ != state_validity_config["state_validity_checker"]["multiple_manipulators"].size()){
      cout << "ERROR: Number of manipulators in trajectory_handler not same as in state validity checker!" << endl;
      cout << "  This occured in end_effector_configuration_node.cpp" << endl;
      cout << "  Num manipulators in trajectory handler: " << n_manipulators_ << endl;
      cout << "  Num manipulators in validity checker: " << state_validity_config["state_validity_checker"]["multiple_manipulators"].size() << endl;
      exit(0);
    }
    n_manipulators_ = state_validity_config["state_validity_checker"]["multiple_manipulators"].size();
    // Get the total dof of the system
    total_dof_ = 0;
    for (int i=0; i<n_manipulators_; i++){
      int start, end;
      start = state_validity_config["state_validity_checker"]["multiple_manipulators"][i]["indexes"]["start"].as<int>();
      end = state_validity_config["state_validity_checker"]["multiple_manipulators"][i]["indexes"]["end"].as<int>();
      total_dof_ += (end-start+1);
      start_indexes_.push_back(start);
      end_indexes_.push_back(end);
      // Load transform between base and manipulator
      std::vector<double> temp_transf;
      temp_transf = state_validity_config["state_validity_checker"]["multiple_manipulators"][i]["base_manipulator_transform"].as< std::vector<double> >();
      Eigen::Affine3d t_base_manipulator;
      t_base_manipulator = Eigen::Affine3d::Identity();
      Eigen::Matrix3d rot_base_manipulator;
      rot_base_manipulator = Eigen::AngleAxisd(temp_transf[5], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(temp_transf[4],  Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(temp_transf[3], Eigen::Vector3d::UnitX());
      t_base_manipulator.translate(Eigen::Vector3d(temp_transf[0], temp_transf[1], temp_transf[2]));
      t_base_manipulator.rotate(rot_base_manipulator);
      t_b_l0_.push_back(t_base_manipulator);
    }

    // Total dof is useful to initialize the state and reference vectors
    state_ = Eigen::VectorXd::Zero(total_dof_);
    reference_ = Eigen::VectorXd::Zero(total_dof_);

    // Initialize end-effector states and references to be published.
    for (int i=0; i<n_manipulators_; i++){
      geometry_msgs::PoseStamped temp_pose;
      temp_pose.header.frame_id = "world";
      ee_states_.poses.push_back(temp_pose);
      ee_references_.poses.push_back(temp_pose);
    }
    ee_states_.header.frame_id = "world";
    ee_references_.header.frame_id = "world";

    // Subscribers for state and reference 
    state_sub_ = nh_.subscribe("end_effector_configuration/state", 
      1, &EndEffectorConfiguration::stateCallback, this);
    reference_sub_ = nh_.subscribe("end_effector_configuration/reference", 
      1, &EndEffectorConfiguration::referenceCallback, this);
  }

  void run()
  {
    ros::Rate loop_rate(rate_);

    std::vector<Eigen::Affine3d> t_l0_ee_state, t_l0_ee_reference;
    std::vector<Eigen::Affine3d> t_w_b_state, t_w_b_reference;
    Eigen::Affine3d t_w_ee_state, t_w_ee_reference;

    while (ros::ok()){
      ros::spinOnce();

      // Get the direct kinematics of each manipulator
      t_l0_ee_state = kinematics_->getMultipleEndEffectorTransforms(state_);
      t_l0_ee_reference = kinematics_->getMultipleEndEffectorTransforms(reference_);

      // Get the transform between world and base
      t_w_b_state = this->getWorldBaseTransforms(state_);
      t_w_b_reference = this->getWorldBaseTransforms(reference_);

      for (int i=0; i<n_manipulators_; i++){
        t_w_ee_state = t_w_b_state[i]*t_b_l0_[i]*t_l0_ee_state[i];
        t_w_ee_reference = t_w_b_reference[i]*t_b_l0_[i]*t_l0_ee_reference[i];

        ee_states_.poses[i] = this->eigenAffine3dToGeometryMsgsPoseStamped(
          t_w_ee_state);
        ee_references_.poses[i] = this->eigenAffine3dToGeometryMsgsPoseStamped(
          t_w_ee_reference);

        ee_state_pubs_[i].publish(ee_states_.poses[i]);
        ee_reference_pubs_[i].publish(ee_references_.poses[i]);
      }

      //ee_states_.header.stamp = ros::Time::now();
      //ee_references_.header.stamp = ros::Time::now();
      //ee_state_pub_.publish(ee_states_);
      //ee_reference_pub_.publish(ee_references_);
      loop_rate.sleep();
    }
  }

  Eigen::VectorXd jointTrajectoryPointToEigenVectorXd(
    trajectory_msgs::JointTrajectoryPoint trajectory_point)
  {
    Eigen::VectorXd eigen_point;
    eigen_point.resize(trajectory_point.positions.size());
    for (int i=0; i<trajectory_point.positions.size(); i++){
      eigen_point(i) = trajectory_point.positions[i];
    }
    return eigen_point;
  }

  geometry_msgs::PoseStamped eigenAffine3dToGeometryMsgsPoseStamped(
    Eigen::Affine3d transform)
  {
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";

    pose.pose.position.x = transform.translation().x();
    pose.pose.position.y = transform.translation().y();
    pose.pose.position.z = transform.translation().z();

    Eigen::Quaterniond quat(transform.rotation());
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();

    return pose;
  }

  std::vector<Eigen::Affine3d> getWorldBaseTransforms(Eigen::VectorXd q)
  {
    std::vector<Eigen::Affine3d> transforms;
    for (int i=0; i<n_manipulators_; i++){
      transforms.push_back(this->getWorldBaseTransform(q.block(
        start_indexes_[i], 0, end_indexes_[i]-start_indexes_[i]+1, 1)));
    }
    return transforms;
  }

  Eigen::Affine3d getWorldBaseTransform(Eigen::VectorXd q)
  {
    Eigen::Affine3d transform;
    transform = Eigen::Affine3d::Identity();
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(q(5), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(q(4),  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(q(3), Eigen::Vector3d::UnitX());
    transform.translate(Eigen::Vector3d(q(0), q(1), q(2)));
    transform.rotate(rot);

    return transform;
  }

private:
  //ros::Publisher ee_state_pub_, ee_reference_pub_;
  ros::Subscriber state_sub_, reference_sub_;
  ros::NodeHandle nh_;
  std::vector<ros::Publisher> ee_state_pubs_, ee_reference_pubs_;

  int rate_, total_dof_, n_manipulators_;
  std::vector<int> start_indexes_, end_indexes_;

  Eigen::VectorXd state_, reference_;
  geometry_msgs::PoseStamped ee_state_, ee_reference_;
  nav_msgs::Path ee_states_, ee_references_;

  std::vector<Eigen::Affine3d> t_b_l0_;

  shared_ptr<MultipleManipulatorsKinematics> kinematics_;

  void stateCallback(const
    trajectory_msgs::JointTrajectoryPoint msg)
  {
    if (msg.positions.size() >= total_dof_){
      state_ = jointTrajectoryPointToEigenVectorXd(msg);
    }
    else{
      cout << "State not full! State size: ";
      cout << msg.positions.size() << " Required size: " << total_dof_ << endl;
    }
  }

  void referenceCallback(const
    trajectory_msgs::JointTrajectoryPoint msg)
  {
    if (msg.positions.size() >= total_dof_){
      reference_ = jointTrajectoryPointToEigenVectorXd(msg);
    }
    else{
      ROS_DEBUG_STREAM_DELAYED_THROTTLE(10, "Reference not full! Reference size: ");
      ROS_DEBUG_STREAM_DELAYED_THROTTLE(
        10, msg.positions.size() << " Required size: " << total_dof_ << endl);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "end_effector_configuration");

  EndEffectorConfiguration ee;
  ee.run();

  return 0;
}