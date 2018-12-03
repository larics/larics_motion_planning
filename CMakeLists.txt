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
    cmake_modules 
)


catkin_package(
  INCLUDE_DIRS
  LIBRARIES larics_motion_planning
  CATKIN_DEPENDS
  DEPENDS 
)

include_directories( 

)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")