#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float32, Float64, Float64MultiArray
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
  MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint
from larics_motion_planning.srv import MultiDofTrajectory, \
  MultiDofTrajectoryRequest, MultiDofTrajectoryResponse
import copy

class PlanFullStateTrajectoryFromObjectWaypoints:

  def __init__(self):
    self.plan_full_state_path_service = rospy.ServiceProxy(
      "multiple_manipulators_object_trajectory", MultiDofTrajectory)

    # Publisher to planner go_to
    planner_ns = rospy.get_param('~planner_ns', "planner")
    self.plan_full_state_model_trajectory_service = rospy.ServiceProxy(
      "/" + planner_ns + "/multiple_manipulators_model_correction_trajectory",
      MultiDofTrajectory)

    # Input service for requesting the trajectory
    self.plan_trajectory_from_model_waypoints = rospy.Service(
      'plan_full_state_trajectory', 
      MultiDofTrajectory, self.planFullStateTrajectoryCallback)

  def run(self):
    rospy.spin()

  def planFullStateTrajectoryCallback(self, req):
    traj_res = MultiDofTrajectoryResponse()

    # Call the full state path service based on object waypoints. It will
    # return the full state path that can be sent to model trajectory planner.
    # 
    path_res = self.plan_full_state_path_service(req)

    # Create model correction service
    traj_req = MultiDofTrajectoryRequest()
    traj_req.plan_path = False
    traj_req.publish_path = False
    traj_req.plan_trajectory = True
    traj_req.publish_trajectory = True
    # Waypoints are path planned earlier
    traj_req.waypoints = path_res.path

    # Plan the full state trajectory and publish it
    traj_res = self.plan_full_state_model_trajectory_service(traj_req)


    return traj_res

if __name__ == '__main__':

  rospy.init_node('full_state_trajectory_from_object')
  planner = PlanFullStateTrajectoryFromObjectWaypoints()
  planner.run()

