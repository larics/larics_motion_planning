#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
import copy, os, time
import numpy as np
from math import sin, cos, tan, atan2
from std_msgs.msg import String, Float64MultiArray, Float64
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
  MultiDOFJointTrajectoryPoint
from larics_motion_planning.srv import GetPlantBoxInspectionPoints, \
  GetPlantBoxInspectionPointsRequest, GetPlantBoxInspectionPointsResponse, \
  MultiDofTrajectory, MultiDofTrajectoryRequest, MultiDofTrajectoryResponse

class PlantInspectionStateMachine:

  def __init__(self, node_name):
    # Get node parameters
    self.rate = rospy.get_param('~rate', 1)

    # This vector contains x,y,z,yaw 
    self.box_config = Float64MultiArray()
    # List of inspection points
    self.T_points = []

    # UAV and manipulator config
    self.dof_uav = rospy.get_param('~uav_dof', int(6))
    self.dof_manipulator = rospy.get_param('~manipulator_dof', (5))
    self.n_dof = self.dof_uav + self.dof_manipulator
    self.current_reference = [0]*self.n_dof

    # Publishers
    self.state = String()
    self.state.data = "Idle"
    self.state_pub = rospy.Publisher(node_name + 
      "/state", String, queue_size=1)
    # Add trajectory publisher
    self.joint_trajectory_pub = rospy.Publisher("joint_trajectory",
      JointTrajectory, queue_size=1)

    # Service clients
    # Getting inspection points for plant box
    self.inspection_points = Path()
    self.get_inspection_points_service = rospy.ServiceProxy(
      "plant_box_inspection_points/get_inspection_points", 
      GetPlantBoxInspectionPoints)
    # Request trajectory planning without model
    self.plan_trajectory_service = rospy.ServiceProxy(
      "multi_dof_trajectory", MultiDofTrajectory)
    # Request trajectory planning with model corrections
    self.plan_trajectory_with_model_service = rospy.ServiceProxy(
      "model_correction_trajectory", MultiDofTrajectory)

    # Subscribers
    # Add subscribers for box config and box ID
    rospy.Subscriber(node_name + "/box_config", Float64MultiArray, 
      self.boxConfigCallback, queue_size=1)
    # Subscriber for the UAV current reference
    self.uav_current_reference = MultiDOFJointTrajectoryPoint()
    rospy.Subscriber("trajectory_point_ref", MultiDOFJointTrajectoryPoint,
      self.uavCurrentReferenceCallback, queue_size=1)
    # Subscribers for manipulator references
    self.manipulator_joint_states = []
    for i in range(self.dof_manipulator):
      joint = JointPositionControllerSubscriber()
      self.manipulator_joint_states.append(copy.deepcopy(joint))
      topic = "joint" + str(i+1) + "_position_controller/command" 
      rospy.Subscriber(topic, Float64, 
        self.manipulator_joint_states[i].jointReferenceCallback, queue_size=1)

  def run(self):
    rate = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      rate.sleep()
      self.state_pub.publish(self.state)
      
      if self.state.data == "SanityCheck":
        self.handleStateSanityCheck()
      elif self.state.data == "GetInspectionPoints":
        self.handleStateGetInspectionPoints()
      elif self.state.data == "GoToFirstPoint":
        self.handleStateGoToFirstPoint()
      elif self.state.data == "InspectForward":
        self.handleStateInspectForward()
      elif self.state.data == "InspectBack":
        self.handleStateInspectBack()
      elif self.state.data == "DequeuePoints":
        self.handleStateDequeuePoints()

  def boxConfigCallback(self, msg):
    if self.state.data == "Idle":
      self.state.data = "SanityCheck"
      self.box_config = msg

  def uavCurrentReferenceCallback(self, msg):
    self.uav_current_reference = msg
    self.updateCurrentReference()

  def updateCurrentReference(self):
    if len(self.uav_current_reference.transforms) > 0:
      # First update x, y and z
      self.current_reference[0] = self.uav_current_reference.transforms[0].translation.x
      self.current_reference[1] = self.uav_current_reference.transforms[0].translation.y
      self.current_reference[2] = self.uav_current_reference.transforms[0].translation.z
      if self.dof_uav == 4:
        self.current_reference[3] = quat2yaw(
          self.uav_current_reference.transforms[0].rotation)
      elif self.dof_uav == 6:
        self.current_reference[5] = quat2yaw(
          self.uav_current_reference.transforms[0].rotation)

    for i in range(self.dof_manipulator):
      self.current_reference[i+self.dof_uav] = self.manipulator_joint_states[i].joint_state

  def handleStateSanityCheck(self):
    print("[PlantInspectionSm]->Entering SanityCheck state!")

    self.state.data = "GetInspectionPoints"

    # If box config vector is invalid than go to idle.
    if len(self.box_config.data) < 4:
      print("SanityCheck: Box config data has less than 4 members")
      self.state.data = "Idle"
    # Add more sanity checks if necessary

  def handleStateGetInspectionPoints(self):
    print("[PlantInspectionSm]-> Entering GetInspectionPoints state!")
    # Call inspection points service
    req = GetPlantBoxInspectionPointsRequest()
    req.box_config_vector = self.box_config.data
    res = self.get_inspection_points_service(req)

    # If service call was success and there was even number of inspection
    # points than use these inspection points. Otherwise go to idle.
    if (res.success == True) and (len(res.inspection_points.poses) % 2 == 0):
      self.inspection_points = res.inspection_points
      self.state.data = "GoToFirstPoint"
    else:
      self.state.data = "Idle"

  def handleStateGoToFirstPoint(self):
    print("[PlantInspection]-> Entering GoToFirstPoint state!")
    # Update current reference just in case
    self.updateCurrentReference()

    # Set up waypoints for getting to the first point.
    endpoint = copy.deepcopy(self.current_reference)
    endpoint[0] = self.inspection_points.poses[0].pose.position.x
    endpoint[1] = self.inspection_points.poses[0].pose.position.y
    endpoint[2] = self.inspection_points.poses[0].pose.position.z
    endpoint[self.dof_uav-1] = quat2yaw(
      self.inspection_points.poses[0].pose.orientation)

    # Create service request
    req = MultiDofTrajectoryRequest()
    temp_trajectory_point = JointTrajectoryPoint()
    temp_trajectory_point.positions = self.current_reference
    req.waypoints.points.append(copy.deepcopy(temp_trajectory_point))
    temp_trajectory_point.positions = endpoint
    req.waypoints.points.append(copy.deepcopy(temp_trajectory_point))
    req.plan_path = False
    req.publish_path = False
    req.plan_trajectory = True
    req.publish_trajectory = False

    # Call the service
    res = self.plan_trajectory_service(req)
    #print(res.trajectory.points[len(res.trajectory.points)-1].time_from_start.to_sec())
    if res.success == False:
      # Return to idle if trajectory was not planned
      self.state.data = "Idle"
    else:
      self.state.data = "InspectForward"
      self.joint_trajectory_pub.publish(res.trajectory)
      # Wait until the UAV reaches the position. This is done in "open loop"
      # by just waiting for trajectory length + some time
      t0 = time.time()
      dt = res.trajectory.points[len(res.trajectory.points)-1].time_from_start.to_sec()
      temp_rate = rospy.Rate(10)
      while (not rospy.is_shutdown()) and ((time.time()-t0) < (dt + 5.0)):
        temp_rate.sleep()
        #print("waiting: ", time.time()-t0)

  def handleStateInspectForward(self):
    # Update current reference just in case
    self.updateCurrentReference()

    # Set up waypoints for inspecting forward. We are going to the second point
    # in the list of inspection points.
    endpoint = copy.deepcopy(self.current_reference)
    endpoint[0] = self.inspection_points.poses[1].pose.position.x
    endpoint[1] = self.inspection_points.poses[1].pose.position.y
    endpoint[2] = self.inspection_points.poses[1].pose.position.z
    endpoint[self.dof_uav-1] = quat2yaw(
      self.inspection_points.poses[1].pose.orientation)

    # Create service request
    req = MultiDofTrajectoryRequest()
    temp_trajectory_point = JointTrajectoryPoint()
    temp_trajectory_point.positions = self.current_reference
    req.waypoints.points.append(copy.deepcopy(temp_trajectory_point))
    temp_trajectory_point.positions = endpoint
    req.waypoints.points.append(copy.deepcopy(temp_trajectory_point))
    req.plan_path = False
    req.publish_path = False
    req.plan_trajectory = True
    req.publish_trajectory = False

    # Call the service for planning with the model
    res = self.plan_trajectory_with_model_service(req)
    #print(res.trajectory.points[len(res.trajectory.points)-1].time_from_start.to_sec())
    if res.success == False:
      # Return to idle if trajectory was not planned
      self.state.data = "Idle"
    else:
      self.state.data = "InspectBack"
      self.joint_trajectory_pub.publish(res.trajectory)
      # Wait until the UAV reaches the position. This is done in "open loop"
      # by just waiting for trajectory length + some time
      t0 = time.time()
      dt = res.trajectory.points[len(res.trajectory.points)-1].time_from_start.to_sec()
      temp_rate = rospy.Rate(10)
      while (not rospy.is_shutdown()) and ((time.time()-t0) < (dt + 5.0)):
        pass
        #print("dt: ", dt, " waiting: ", time.time()-t0)


  def handleStateInspectBack(self):
    # Update current reference just in case
    self.updateCurrentReference()

    # Set up waypoints for inspecting forward. We are going back to the first
    # point so the end-effector is not between plants.
    endpoint = copy.deepcopy(self.current_reference)
    endpoint[0] = self.inspection_points.poses[0].pose.position.x
    endpoint[1] = self.inspection_points.poses[0].pose.position.y
    endpoint[2] = self.inspection_points.poses[0].pose.position.z
    endpoint[self.dof_uav-1] = quat2yaw(
      self.inspection_points.poses[0].pose.orientation)

    # Create service request
    req = MultiDofTrajectoryRequest()
    temp_trajectory_point = JointTrajectoryPoint()
    temp_trajectory_point.positions = self.current_reference
    req.waypoints.points.append(copy.deepcopy(temp_trajectory_point))
    temp_trajectory_point.positions = endpoint
    req.waypoints.points.append(copy.deepcopy(temp_trajectory_point))
    req.plan_path = False
    req.publish_path = False
    req.plan_trajectory = True
    req.publish_trajectory = False

    # Call the service for planning with the model
    res = self.plan_trajectory_with_model_service(req)
    #print(res.trajectory.points[len(res.trajectory.points)-1].time_from_start.to_sec())
    if res.success == False:
      # Return to idle if trajectory was not planned
      self.state.data = "Idle"
    else:
      self.state.data = "DequeuePoints"
      self.joint_trajectory_pub.publish(res.trajectory)
      # Wait until the UAV reaches the position. This is done in "open loop"
      # by just waiting for trajectory length + some time
      t0 = time.time()
      dt = res.trajectory.points[len(res.trajectory.points)-1].time_from_start.to_sec()
      temp_rate = rospy.Rate(10)
      while (not rospy.is_shutdown()) and ((time.time()-t0) < (dt + 5.0)):
        pass
        #print("dt: ", dt, " waiting: ", time.time()-t0)

  def handleStateDequeuePoints(self):
    # Remove first two points from the list of points.
    self.inspection_points.poses.pop(0)
    self.inspection_points.poses.pop(0)

    # Check if the list is empty or has only one point
    if len(self.inspection_points.poses) < 2:
      # If the inspection is done return to idle state
      self.state.data = "Idle"
    else:
      # Otherwise go to first point without model corrections
      self.state.data = "GoToFirstPoint"

# Helper function for transforming quaternion to yaw
def quat2yaw(quat):
  q0 = quat.w
  q1 = quat.x
  q2 = quat.y
  q3 = quat.z
  yaw = atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3))
  return yaw

# Helper class for joint position subscriber
class JointPositionControllerSubscriber:

    def __init__(self):
        self.joint_state = 0.0

    def jointReferenceCallback(self, msg):
        self.joint_state = msg.data
        #print("Joint data is: ", self.joint_state)

if __name__ == '__main__':
  node_name = 'plant_inspection_sm'
  rospy.init_node(node_name)
  inspection_points = PlantInspectionStateMachine(node_name)
  inspection_points.run()
