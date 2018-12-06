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
    topp_ros
    cmake_modules 
)

find_package(Eigen3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES 
  CATKIN_DEPENDS
  DEPENDS
)

include_directories( 
  include ${catkin_INCLUDE_DIRS}
  )

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library(trajectory src/TrajectoryInterface.cpp src/ToppraTrajectory.cpp)
target_link_libraries(trajectory ${catkin_LIBRARIES})
add_executable(testing_node src/test.cpp)
target_link_libraries(testing_node trajectory)