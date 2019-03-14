/// \file Visualization.h
/// \brief Converts Eigen path and trajectory to ROS topic types.

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "MotionPlanningDatatypes.h"
#include <iostream>

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

/// \brief This class converts path and trajectory to ROS visualization types
///   suitable for displaying in RVIZ.
class Visualization
{
  public:
    /// \brief Constructor.
    Visualization();

    /// \brief Generates nav_msgs::Path suitable for displaying in RVIZ.
    /// \param eigen_path Path represented as Eigen::MatrixXd. Must have more
    ///   than one degree of freedom.
    /// \param z If 2D path is to be converted this provides height. The
    ///   default value of this argument is 0.0.
    /// \returns Success of conversion. If unsuccessful previous path will not
    ///   be overwritten.
    bool eigenPathToNavMsgsPath(Eigen::MatrixXd eigen_path, double z=0.0);

    /// \brief Gets path generated with eigenPathToNavMsgsPath function
    /// \returns Path converted to ROS message nav_msgs::Path
    nav_msgs::Path getPath();

    /// 
    void publishPath();
  private:
    ros::NodeHandle nh_;
    nav_msgs::Path path_;
    ros::Publisher path_publisher_;
};

#endif // VISUALIZATION_H
