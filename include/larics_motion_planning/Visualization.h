/// \file Visualization.h
/// \brief Converts Eigen path and trajectory to ROS topic types.

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <larics_motion_planning/MotionPlanningDatatypes.h>
#include <iostream>

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

using namespace std;

/// \brief This class converts path and trajectory to ROS visualization types
///   suitable for displaying in RVIZ.
class Visualization
{
  public:
    /// \brief Constructor.
    Visualization();

    /// \brief Generates nav_msgs::Path suitable for displaying in RVIZ
    /// \param eigen_path Path represented as Eigen::MatrixXd
    /// \param projection Projects path to plane z=0 if true
    void setPath(Eigen::MatrixXd eigen_path, bool projection=false);

    /// \brief Set path from ROS message.
    /// \param path Path to be visualized.
    void setPath(nav_msgs::Path path);

    /// \brief Gets path generated with eigenPathToNavMsgsPath function
    /// \returns Path converted to ROS message nav_msgs::Path
    nav_msgs::Path getPath();

    /// \brief Publishes converted path.
    void publishPath();

    /// \brief Generates nav_msgs::Path for trajectory suitable for displaying 
    ///   in RVIZ
    /// \param eigen_path Positions in trajectory represented as Eigen::MatrixXd
    /// \param projection Projects trajectory to plane z=0 if true
    void setTrajectory(Eigen::MatrixXd eigen_path, bool projection=false);

    /// \brief Generates nav_msgs::Path for trajectory suitable for displaying 
    ///   in RVIZ
    /// \param trajectory Trajectory type from MotionPlanningDatatypes.h
    /// \param projection Projects trajectory to plane z=0 if true
    void setTrajectory(Trajectory trajectory, bool projection=false);

    /// \brief Set trajectory from ROS nav_msgs::Path message.
    /// \param path Trajectory to be visualized.
    void setTrajectory(nav_msgs::Path trajectory);

    /// \brief Get trajectory represented as path generated with 
    ///   eigenTrajectoryToNavMsgsPath function.
    /// \returns Trajectory converted to ROS message nav_msgs::Path
    nav_msgs::Path getTrajectory();

    /// \brief Publishes trajectory as path message.
    void publishTrajectory();

    /// \brief Generates visualization_msgs::Marker to visualize waypoints.
    /// \param waypoints Eigen matrix of waypoints to be visualized.
    void setWaypoints(Eigen::MatrixXd waypoints);

    /// \brief Get waypoints represented as visualization_msgs::Marker
    /// \return Waypoints converted to ROS message.
    visualization_msgs::Marker getWaypoints();

    /// \brief Publishes waypoints as marker message
    void publishWaypoints();

    /// \brief Generates visualization_msgs::Marker to visualise a set of 
    ///   points that are checked by state validity checker.
    /// \param points A set of points that define state of the robot.
    void setStatePoints(Eigen::MatrixXd points);

    /// \brief Returns points that are visualized
    /// \return Points converted to ROS message.
    visualization_msgs::Marker getStatePoints();

    /// \brief Publishes state points as marker message
    void publishStatePoints();

    /// \brief Publishes path, trajectory and waypoints.
    void publishAll();

    /// \brief Clears all visualization.
    void clearAll();

  private:
    ros::NodeHandle nh_;
    nav_msgs::Path path_, trajectory_;
    visualization_msgs::Marker waypoints_, state_points_;
    ros::Publisher path_publisher_, trajectory_publisher_, waypoints_publisher_, 
      state_points_publisher_;

    double state_scale_, waypoints_scale_;
    std::string frame_id_;

    nav_msgs::Path eigenMatrixXdToNavMsgsPath(Eigen::MatrixXd eigen_path, 
      bool projection=false);

    visualization_msgs::Marker navMsgsPathToVisualizationMsgsMarker(
      nav_msgs::Path path);
};

#endif // VISUALIZATION_H
