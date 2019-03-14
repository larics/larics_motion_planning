cmake_minimum_required(VERSION 2.8.3)
project(larics_motion_planning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    roslib
    roscpp
    std_msgs
    geometry_msgs
    trajectory_msgs
    nav_msgs
    topp_ros
    cmake_modules
    eigen_conversions
)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(octomap REQUIRED)
find_package(OMPL REQUIRED)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES 
  CATKIN_DEPENDS
  DEPENDS
)

include_directories( 
  include ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}
  )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

add_library(trajectory src/TrajectoryInterface.cpp src/ToppraTrajectory.cpp)
target_link_libraries(trajectory ${catkin_LIBRARIES} yaml-cpp)

add_library(map src/MapInterface.cpp src/OctomapMap.cpp)
target_link_libraries(map ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_library(path_planning src/PathPlanningInterface.cpp src/RrtPathPlanner.cpp)
target_link_libraries(map ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen_LIBRARIES})

add_library(visualization src/Visualization.cpp)
target_link_libraries(map ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(testing_node src/test.cpp)
target_link_libraries(testing_node trajectory map path_planning visualization)