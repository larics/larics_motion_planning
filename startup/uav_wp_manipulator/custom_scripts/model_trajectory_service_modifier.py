#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math, time, copy
import numpy as np
from math import sin, cos
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, Bool
from larics_motion_planning.srv import MultiDofTrajectory, \
  MultiDofTrajectoryRequest, MultiDofTrajectoryResponse
from trajectory_msgs.msg import JointTrajectory

class ModelTrajectoryServiceModifier:

  def __init__(self):
    self.multi_dof_trajectory_client = rospy.ServiceProxy(
      "multi_dof_trajectory", MultiDofTrajectory)

    self.model_correction_trajectory_client = rospy.ServiceProxy(
      "model_correction_trajectory", MultiDofTrajectory)

    self.trajectory_service_server = rospy.Service(
      'model_trajectory_service_modifier/model_correction_trajectory',
      MultiDofTrajectory, self.trajectoryServiceCallback)

    print("Constructor done.")

  def run(self):
    rospy.spin()

  def trajectoryServiceCallback(self, req):
    res = MultiDofTrajectoryResponse()
    # First handle the request. It needs 3 or more points since we expect a
    # starting point, point before insertion, and point after insertion. The
    # last point is a bit problematic because it is very likely to be in
    # false collision with the environment. Therefore, the last point is
    # removed from the request. The initial request is only here for the
    # waypoints.
    # TODO!!!! IZBACITI ZADNJI WAYPOINT I NAPRAVITI SANITY CHECK
    # Check the number of waypoints. The last one will be removed, so if
    # there are less than 3 return empty response
    if len(req.waypoints.points) < 3:
      print("Less than 3 waypoints provided. Aborting!")
      return res
    # Remove the last waypoint for first path planning
    multi_dof_trajectory_req = copy.deepcopy(req)
    multi_dof_trajectory_req.waypoints.points.pop(-1)
    # Set up flags for planning
    multi_dof_trajectory_req.plan_path = True
    multi_dof_trajectory_req.publish_path = False
    multi_dof_trajectory_req.plan_trajectory = False
    multi_dof_trajectory_req.publish_trajectory = False
    multi_dof_trajectory_res = \
      self.multi_dof_trajectory_client(multi_dof_trajectory_req)

    # Fill in the model correction trajectory, append the last waypoint
    model_correction_trajectory_req = MultiDofTrajectoryRequest()
    model_correction_trajectory_req.waypoints.points = \
      multi_dof_trajectory_res.path.points
    model_correction_trajectory_req.waypoints.points.append(
      req.waypoints.points[-1])
    model_correction_trajectory_req.plan_path = False
    model_correction_trajectory_req.publish_path = False
    model_correction_trajectory_req.plan_trajectory = True
    model_correction_trajectory_req.publish_trajectory = True

    # Call the model correction service
    res = self.model_correction_trajectory_client(
      model_correction_trajectory_req)

    return res

if __name__ == '__main__':

  rospy.init_node('model_trajectory_service_modifier')
  modifier = ModelTrajectoryServiceModifier()
  modifier.run()

