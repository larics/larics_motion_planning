#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
import numpy as np
from math import sin, cos
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float32, Float64, Float64MultiArray
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
  MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint
from larics_motion_planning.srv import MultiDofTrajectory, \
  MultiDofTrajectoryRequest, MultiDofTrajectoryResponse
import copy

class GenerateObjectTrajectory:

  def __init__(self):
    # Service to call for planning the full state trajectory from the input
    # configuration.
    self.plan_full_state_trajectory = rospy.ServiceProxy(
      "plan_full_state_trajectory", MultiDofTrajectory)

    self.waypoints = JointTrajectory()

    # Subscriber for object trajectory
    self.trajectory_config = Float64MultiArray()
    rospy.Subscriber('generate_object_trajectory/config', Float64MultiArray,
      self.objectTrajectoryConfigCallback, queue_size=1)

  def run(self):
    rospy.spin()

  def callFullStateTrajectoryPlanning(self):
    # Create model correction service
    print("Calling the full state trajectory service.")
    req = MultiDofTrajectoryRequest()
    req.plan_path = False
    req.publish_path = False
    req.plan_trajectory = False
    req.publish_trajectory = False
    req.waypoints = self.waypoints

    res = self.plan_full_state_trajectory(req)

    print("Trajectory generated and is currently executing.")

  def objectTrajectoryConfigCallback(self, msg):
    self.trajectory_config = msg

    if len(self.trajectory_config.data) == 0:
      print("No data provided in the trajectory config.")
    elif int(self.trajectory_config.data[0]) == 0:
      self.generateCircularTrajectoryWithVaryingRotation()

  def generateCircularTrajectoryWithVaryingRotation(self):
    # Unpack the trajectory config
    if len(self.trajectory_config.data) < 6:
      print("generate_object_trajectory.py")
      print("  Circular trajectory with varying rotation expects 6 elements.")
      print("  Not enough config data provided.")
      return
    x_c = self.trajectory_config.data[1]
    y_c = self.trajectory_config.data[2]
    z_c = self.trajectory_config.data[3]
    r = self.trajectory_config.data[4]
    delta_yaw = self.trajectory_config.data[5]

    # Clear trajectory for storing waypoints
    self.waypoints = JointTrajectory()
    current_waypoint = JointTrajectoryPoint()

    for yaw in np.arange(0, 2*math.pi, delta_yaw):
      x = r*cos(yaw) + x_c
      y = r*sin(yaw) + y_c
      z = 0 + z_c
      current_waypoint.positions = [x,y,z,0,0,wrapToPi(yaw)]
      self.waypoints.points.append(copy.deepcopy(current_waypoint))

    # Add last waypoint
    current_waypoint.positions = [r*cos(0)+x_c, r*sin(0)+y_c, z_c, 0, 0, 0]
    self.waypoints.points.append(copy.deepcopy(current_waypoint))

    # Plan and execute the trajectory
    self.callFullStateTrajectoryPlanning()

def wrapToPi(x):
  x = math.fmod(x + math.pi,2*math.pi)
  if (x < 0):
    x += 2*math.pi
  return x - math.pi

if __name__ == '__main__':

  rospy.init_node('generate_object_trajectory')
  planner = GenerateObjectTrajectory()
  planner.run()

