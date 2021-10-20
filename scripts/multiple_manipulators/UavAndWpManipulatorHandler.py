#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from std_msgs.msg import Float64
from geometry_msgs.msg import Transform, Twist
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint
import copy
from JointPositionControllerHandler import *

class UavAndWpManipulatorHandler:

  def __init__(self, config):
    # Publisher for the UAV
    uav_topic = "/" + config["namespace"] + "/" + config["uav_topic"]
    self.trajectory_point_pub = rospy.Publisher(uav_topic, 
      MultiDOFJointTrajectoryPoint, queue_size=1)

    # Manipulator publishers are list because it has multiple DoF.
    self.manipulator_pubs = []
    self.manipulator_dof = len(config["manipulator_topics"])
    for i in range(self.manipulator_dof):
      current_topic = "/" + config["namespace"] + "/" + \
        config["manipulator_topics"][i]
      current_pub = JointPositionControllerHandler(current_topic)
      self.manipulator_pubs.append(copy.copy(current_pub))

  def publish(self, joint_trajectory_point):
    # Create UAV point from the trajectory point. It is assumed that the
    # received point has 6DoF for base and n DoF for manipulator.
    multi_dof_trajectory_point = createMultiDofJointTrajectoryPoint(
      joint_trajectory_point)
    self.trajectory_point_pub.publish(multi_dof_trajectory_point)

    # Publish manipulator points.
    for i in range(self.manipulator_dof):
      self.manipulator_pubs[i].publish(
        joint_trajectory_point.positions[i+6])


def createMultiDofJointTrajectoryPoint(joint):
  multi = MultiDOFJointTrajectoryPoint()

  transform = Transform()
  transform.translation.x = joint.positions[0]
  transform.translation.y = joint.positions[1]
  transform.translation.z = joint.positions[2]
  transform.rotation.z = math.sin(joint.positions[5]/2.0)
  transform.rotation.w = math.cos(joint.positions[5]/2.0)

  vel = Twist()
  vel.linear.x = joint.velocities[0]
  vel.linear.y = joint.velocities[1]
  vel.linear.z = joint.velocities[2]
  vel.angular.z = joint.velocities[5]

  acc = Twist()
  acc.linear.x = joint.accelerations[0]
  acc.linear.y = joint.accelerations[1]
  acc.linear.z = joint.accelerations[2]
  acc.angular.z = joint.accelerations[5]

  multi.transforms.append(transform)
  multi.velocities.append(vel)
  multi.accelerations.append(acc)

  return multi