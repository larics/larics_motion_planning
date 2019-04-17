/// \file Visualization.h
/// \brief Converts Eigen path and trajectory to ROS topic types.

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "MotionPlanningDatatypes.h"
#include <iostream>

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

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
  private:
    ros::NodeHandle nh_;
    nav_msgs::Path path_, trajectory_;
    ros::Publisher path_publisher_, trajectory_publisher_, waypoints_publisher_;

    /// \brief Generates nav_msgs::Path suitable for displaying in RVIZ.
    /// \param eigen_path Path represented as Eigen::MatrixXd. Must have more
    ///   than one degree of freedom.
    /// \param z If 2D path is to be converted this provides height. The
    ///   default value of this argument is 0.0.
    /// \returns Success of conversion. If unsuccessful previous path will not
    ///   be overwritten.
    nav_msgs::Path eigenMatrixXdToNavMsgsPath(Eigen::MatrixXd eigen_path, 
      bool projection=false);
};

#endif // VISUALIZATION_H
