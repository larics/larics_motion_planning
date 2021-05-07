#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
import copy, os, time
import numpy as np
import math
from math import sin, cos, tan, atan2
from std_msgs.msg import String, Float64MultiArray, Float64, Int32
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
  MultiDOFJointTrajectoryPoint
from larics_motion_planning.srv import GetPlantBoxInspectionPoints, \
  GetPlantBoxInspectionPointsRequest, GetPlantBoxInspectionPointsResponse, \
  MultiDofTrajectory, MultiDofTrajectoryRequest, MultiDofTrajectoryResponse

class EllipseWaypointsHandler:

  def __init__(self, node_name):
    # Get node parameters
    self.rate = rospy.get_param('~rate', 1)

    # This vector contains x,y,z,yaw 
    self.box_config = Float64MultiArray()
    # List of inspection points
    self.T_points = []

    # UAV and manipulator config
    self.dof_uav = rospy.get_param('~uav_dof', int(6))
    self.dof_manipulator = rospy.get_param('~manipulator_dof', int(5))
    self.n_dof = self.dof_uav + self.dof_manipulator
    self.current_reference = [0]*self.n_dof
    # Alpha dof relates to the camera pitch dof.
    self.alpha_dof_id = rospy.get_param('~alpha_dof_id', int(7))
    # Default reference is set to all zeros
    self.default_reference = [0]*self.n_dof

    # Add trajectory publisher
    self.joint_trajectory_pub = rospy.Publisher(
      "reference_tracker/joint_trajectory",
      JointTrajectory, queue_size=1)
    # Go to publisher
    self.go_to_pub = rospy.Publisher("go_to/reference/joint_trajectory_point",
      JointTrajectoryPoint, queue_size=1)

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
    # Subscriber for setting the default reference
    rospy.Subscriber(node_name + "/default_reference", Float64MultiArray,
      self.defaultReferenceCallback, queue_size=1)
    # Subscriber for executing trajectory flag
    self.executing_trajectory = 0
    self.executing_trajectory_previous = 0
    self.executing_trajectory_stop_timestamp = 0.0
    rospy.Subscriber("reference_tracker/executing_trajectory", Int32,
      self.executingTrajectoryCallback, queue_size=1)

  def run(self):
    rospy.spin()

  def executingTrajectoryCallback(self, msg):
    self.executing_trajectory_previous = copy.deepcopy(self.executing_trajectory)
    self.executing_trajectory = msg.data
    if self.executing_trajectory > 0.5:
      self.executing_trajectory_stop_timestamp = rospy.Time.now().to_sec()


  def defaultReferenceCallback(self, msg):
    if len(msg.data) == self.n_dof:
      for i in range(self.n_dof):
        self.default_reference[i] = msg.data[i]
    else:
      print("[EllipseWaypointsHandler]->defaultReferenceCallback")
      print("  Received dimension is " + str(len(msg.data)) + 
        " but should be " + str(self.n_dof))

  def boxConfigCallback(self, msg):
    print("[EllipseWaypointsHandler]->boxConfigCallback starting.")
    # First call the service ang get the waypoints.
    req_wp = GetPlantBoxInspectionPointsRequest()
    req_wp.box_config_vector = msg.data
    res_wp = self.get_inspection_points_service(req_wp)

    # Next, create and fill the request for trajectory
    req_traj = MultiDofTrajectoryRequest()
    temp_trajectory_point = JointTrajectoryPoint()
    for i in range(len(res_wp.inspection_points.poses)):
      temp_trajectory_point.positions = copy.deepcopy(self.default_reference)
      temp_trajectory_point.positions[0] = res_wp.inspection_points.poses[i].pose.position.x
      temp_trajectory_point.positions[1] = res_wp.inspection_points.poses[i].pose.position.y
      temp_trajectory_point.positions[2] = res_wp.inspection_points.poses[i].pose.position.z
      yaw = quat2yaw(res_wp.inspection_points.poses[i].pose.orientation) + \
        res_wp.ellipse_delta_yaw[i] + math.pi
      temp_trajectory_point.positions[5] = atan2(sin(yaw), cos(yaw))
      # Convert alpha to joint value
      temp_trajectory_point.positions[self.alpha_dof_id] = self.alpha2q(res_wp.ellipse_alpha_vector[i])
      req_traj.waypoints.points.append(copy.deepcopy(temp_trajectory_point))
    req_traj.plan_path = False
    req_traj.publish_path = False
    req_traj.plan_trajectory = True
    req_traj.publish_trajectory = False

    # Move the aerial manipulator to the first point. This assumes it is 
    # already somewhere near the initial point so it is possible to just send
    # a straight line trajectory
    self.go_to_pub.publish(req_traj.waypoints.points[0])
    rate = 10
    r = rospy.Rate(rate)
    i = 0
    # Wait to make sure the trajectory started. 1.5s for now.
    while (not rospy.is_shutdown()) and (i < 15):
      r.sleep()
      i = i + 1
    print "1.5s ellapsed"
    while ((not rospy.is_shutdown()) 
      and ((rospy.Time.now().to_sec()-self.executing_trajectory_stop_timestamp) < 3.0)):
      r.sleep()
    print "3s ellapsed"
    # After this the UAV is on the initial point of the trajectory so we can
    # request planning
    res_traj = self.plan_trajectory_service(req_traj)
    # And publish the trajectory.
    self.joint_trajectory_pub.publish(res_traj.trajectory)
    print("[EllipseWaypointsHandler]->boxConfigCallback done.")


  def alpha2q(self, alpha):
    q = math.pi/2 - 0.787 - alpha
    return q
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
  node_name = 'ellipse_waypoints_trajectory_planner'
  rospy.init_node(node_name)
  ellipse_waypoints = EllipseWaypointsHandler(node_name)
  ellipse_waypoints.run()
