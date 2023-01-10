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

class GoToObject:

  def __init__(self):
    self.plan_full_state_path_service = rospy.ServiceProxy(
      "multiple_manipulators_object_trajectory", MultiDofTrajectory)

    # Publisher to planner go_to
    planner_ns = rospy.get_param('~planner_ns', "planner")
    model_planner_ns = rospy.get_param('~model_planner_ns', "model_planner")
    self.planner_trajectory_point_pub = rospy.Publisher(
      "/" + planner_ns + "/go_to/reference/joint_trajectory_point",
      JointTrajectoryPoint, queue_size=1)
    self.model_planner_trajectory_point_pub = rospy.Publisher(
      "/" + model_planner_ns + "/go_to/reference/joint_trajectory_point",
      JointTrajectoryPoint, queue_size=1)

    # Reference is also trajectory point
    self.reference = JointTrajectoryPoint()

    # Subscriber for the object reference
    rospy.Subscriber('go_to/reference/joint_trajectory_point',
      JointTrajectoryPoint, self.trajectoryPointReferenceCallback, queue_size=1)
    rospy.Subscriber('go_to/reference/multiarray', Float64MultiArray,
      self.multiarrayReferenceCallback, queue_size=1)

  def run(self):
    rospy.spin()

  def trajectoryPointReferenceCallback(self, msg):
    self.reference = msg
    self.generateFullStateWaypoint()

  def multiarrayReferenceCallback(self, msg):
    self.reference.positions = msg.data
    self.generateFullStateWaypoint()

  def generateFullStateWaypoint(self):
    print("Generating trajectory.")
    # Create service request
    req = MultiDofTrajectoryRequest()
    # Add reference twice
    req.waypoints.points.append(self.reference)
    req.waypoints.points.append(self.reference)
    req.plan_path = False
    req.publish_path = False
    req.plan_trajectory = False
    req.publish_trajectory = False

    # Call the service
    res = self.plan_full_state_path_service(req)
    if res.success == False:
      print("Well, something went wrong. Trajectory not planned.")
    else:
      self.planner_trajectory_point_pub.publish(res.path.points[0])
      self.model_planner_trajectory_point_pub.publish(res.path.points[0])

if __name__ == '__main__':

  rospy.init_node('go_to_object_config')
  go_to = GoToObject()
  go_to.run()

