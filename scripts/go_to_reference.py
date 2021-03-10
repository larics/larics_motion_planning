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

class GoToReference:

  def __init__(self):
    self.plan_trajectory_service = rospy.ServiceProxy(
      "go_to/trajectory_request", MultiDofTrajectory)
    self.joint_trajectory_pub = rospy.Publisher('go_to/joint_trajectory', 
      JointTrajectory, queue_size=1)

    # Current state of the system. Initially empty, but will change when
    # tracker starts publishing it.
    self.current_state = JointTrajectoryPoint()
    self.first_state_received = False

    # Reference is also trajectory point
    self.reference = JointTrajectoryPoint()

    # New reference subscriber
    rospy.Subscriber('go_to/current_state', JointTrajectoryPoint,
      self.currentStateCallback, queue_size=1)
    rospy.Subscriber('go_to/reference/joint_trajectory_point',
      JointTrajectoryPoint, self.trajectoryPointReferenceCallback, queue_size=1)
    rospy.Subscriber('go_to/reference/multiarray', Float64MultiArray,
      self.multiarrayReferenceCallback, queue_size=1)

  def run(self):
    rospy.spin()

  def currentStateCallback(self, msg):
    self.current_state = msg
    self.first_state_received = True

  def trajectoryPointReferenceCallback(self, msg):
    self.reference = msg
    self.generateTrajectory()

  def multiarrayReferenceCallback(self, msg):
    self.reference.positions = msg.data
    self.generateTrajectory()

  def generateTrajectory(self):
    if self.first_state_received == False:
      print("First state not yet received!")
    else:
      d = waypointNorm(self.current_state, self.reference)

      if d < 0.05:
        print("Too close to initial point. Not going anywhere.")
      else:
        print("Generating trajectory.")
        # Create service request
        req = MultiDofTrajectoryRequest()
        req.waypoints.points.append(self.current_state)
        req.waypoints.points.append(self.reference)
        req.plan_path = False
        req.publish_path = False
        req.plan_trajectory = True
        req.publish_trajectory = False

        # Call the service
        res = self.plan_trajectory_service(req)
        if res.success == False:
          print("Well, something went wrong. Trajectory not planned.")
        else:
          self.joint_trajectory_pub.publish(res.trajectory)

def waypointNorm(w1, w2):
  d = 0.0
  if (len(w1.positions) != len(w2.positions)):
    return -1.0
  else:
    for i in range(len(w1.positions)):
      d = d + abs(w1.positions[i] - w2.positions[i])
  return d

if __name__ == '__main__':

  rospy.init_node('go_to_reference')
  go_to = GoToReference()
  go_to.run()

