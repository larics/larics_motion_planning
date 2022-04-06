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
    message_generation
    ompl
    aerial_manipulators_control
)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(octomap REQUIRED)
find_package(OMPL)

#add_message_files(
#  FILES
#)

add_service_files(
  FILES
  CartesianTrajectory.srv
  MultiDofTrajectory.srv
  VisualizeState.srv
  ParabolicAirdropTrajectory.srv
  SaveOctomap.srv
  GetPlantBoxInspectionPoints.srv
  CheckStateValidity.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

include_directories( 
  include ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} 
    ${OMPL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}
  )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(TRAJECTORY_SOURCES src/TrajectoryInterface.cpp 
  src/ToppraTrajectory.cpp src/SplineInterpolator.cpp)
add_library(trajectory ${TRAJECTORY_SOURCES})
target_link_libraries(trajectory ${catkin_LIBRARIES} yaml-cpp)

set(MAP_SOURCES src/MapInterface.cpp src/OctomapMap.cpp)
add_library(map ${MAP_SOURCES})
target_link_libraries(map ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

set(KINEMATICS_SOURCES src/KinematicsInterface.cpp 
  src/WpManipulatorKinematics.cpp 
  src/MultipleManipulatorsKinematics.cpp)
add_library(kinematics ${KINEMATICS_SOURCES})
target_link_libraries(kinematics ${catkin_LIBRARIES})

set(MODEL_CORRECTION_SOURCES src/MultipleManipulatorsModelCorrection.cpp
  src/UavWpManipulatorModelCorrection.cpp)
add_library(model_correction ${MODEL_CORRECTION_SOURCES})
target_link_libraries(model_correction kinematics ${catkin_LIBRARIES})

set(STATE_VALIDITY_CHECKING_SOURCES src/StateValidityCheckerInterface.cpp 
  src/PointStateValidityChecker.cpp
  src/UavWpManipulatorStateValidityChecker.cpp
  src/SimpleStateValidityCheckers.cpp
  src/MultipleManipulatorsStateValidityChecker.cpp)
add_library(state_validity_checking ${STATE_VALIDITY_CHECKING_SOURCES})
target_link_libraries(state_validity_checking map kinematics 
  ${catkin_LIBRARIES})

set(PATH_PLANNING_SOURCES src/PathPlanningInterface.cpp 
  src/RrtPathPlanner.cpp)
add_library(path_planning ${PATH_PLANNING_SOURCES})
target_link_libraries(path_planning state_validity_checking ${catkin_LIBRARIES} 
  ${OMPL_LIBRARIES} ${Eigen_LIBRARIES})

set(VISUALIZATION_SOURCES src/Visualization.cpp)
add_library(visualization ${VISUALIZATION_SOURCES})
target_link_libraries(visualization ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

set(GLOBAL_PLANNER_SOURCES src/GlobalPlanner.cpp 
  src/GlobalPlannerRosInterface.cpp src/ParabolicAirdropPlanner.cpp)
add_library(global_planner ${GLOBAL_PLANNER_SOURCES})
target_link_libraries(global_planner trajectory map path_planning 
  kinematics model_correction ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

set(LOCAL_PLANNER_SOURCES src/LocalPlanner.cpp)
add_library(local_planner ${LOCAL_PLANNER_SOURCES})
target_link_libraries(local_planner trajectory map path_planning 
  kinematics ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_library(larics_motion_planning ${TRAJECTORY_SOURCES} ${MAP_SOURCES}
  ${PATH_PLANNING_SOURCES} ${VISUALIZATION_SOURCES} ${GLOBAL_PLANNER_SOURCES}
  ${LOCAL_PLANNER_SOURCES})
target_link_libraries(larics_motion_planning trajectory map kinematics path_planning 
  visualization global_planner local_planner ${catkin_LIBRARIES} ${Eigen_LIBRARIES} 
  yaml-cpp)

add_executable(testing_node src/test.cpp)
target_link_libraries(testing_node larics_motion_planning)

add_executable(global_planner_node src/global_planner_node)
target_link_libraries(global_planner_node larics_motion_planning)

add_executable(local_planner_node src/local_planner_node)
target_link_libraries(local_planner_node larics_motion_planning)

add_executable(end_effector_configuration_node src/end_effector_configuration_node)
target_link_libraries(end_effector_configuration_node larics_motion_planning)

add_executable(binvox2bt src/binvox2bt.cpp)
target_link_libraries(binvox2bt larics_motion_planning)