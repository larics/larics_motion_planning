#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Transform, Twist, Pose
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Odometry
import copy
from JointPositionControllerHandler import *

class UavAndWpManipulatorHandler:

  def __init__(self, config):
    # Publisher for the UAV
    uav_pub_topic = "/" + config["namespace"] + "/" + config["publishers"]["uav_topic"]
    self.trajectory_point_pub = rospy.Publisher(uav_pub_topic, 
      MultiDOFJointTrajectoryPoint, queue_size=1)

    # Manipulator publishers are list because it has multiple DoF.
    self.manipulator_dofs = []
    self.manipulator_dof = len(config["publishers"]["manipulator_topics"])
    for i in range(self.manipulator_dof):
      current_pub_topic = "/" + config["namespace"] + "/" + \
        config["publishers"]["manipulator_topics"][i]
      current_sub_topic = "/" + config["namespace"] + "/" + \
        config["subscribers"]["manipulator_topics"][i]
      current_handler = JointPositionControllerHandler(current_pub_topic,
        current_sub_topic)
      self.manipulator_dofs.append(copy.copy(current_handler))

    # Subscriber for the UAV
    self.state = Float64MultiArray()
    self.current_uav_pose = Pose()
    uav_sub_topic = "/" + config["namespace"] + "/" + config["subscribers"]["uav_topic"]
    rospy.Subscriber(uav_sub_topic, Odometry, self.odometryCallback, queue_size=1)

  def publish(self, joint_trajectory_point):
    # Create UAV point from the trajectory point. It is assumed that the
    # received point has 6DoF for base and n DoF for manipulator.
    multi_dof_trajectory_point = createMultiDofJointTrajectoryPoint(
      joint_trajectory_point)
    self.trajectory_point_pub.publish(multi_dof_trajectory_point)

    # Publish manipulator points.
    for i in range(self.manipulator_dof):
      self.manipulator_dofs[i].publish(
        joint_trajectory_point.positions[i+6])

  def odometryCallback(self, msg):
    self.current_uav_pose.position = msg.pose.pose.position
    self.current_uav_pose.orientation = msg.pose.pose.orientation    

  def getCurrentState(self):
    self.state = JointTrajectoryPoint()

    # Get roll, pitch and yaw from current pose
    q0 = self.current_uav_pose.orientation.w
    q1 = self.current_uav_pose.orientation.x
    q2 = self.current_uav_pose.orientation.y
    q3 = self.current_uav_pose.orientation.z
    roll = math.atan2(2.0*(q0*q1 + q2*q3), 1.0-2.0*(q1*q1 + q2*q2))
    pitch = math.asin(2.0*(q0*q2 - q3*q1))
    yaw = math.atan2(2.0*(q0*q3 + q1*q2), 1.0-2.0*(q2*q2+q3*q3))

    # Add UAV to state
    self.state.positions.append(self.current_uav_pose.position.x)
    self.state.positions.append(self.current_uav_pose.position.y)
    self.state.positions.append(self.current_uav_pose.position.z)
    self.state.positions.append(roll)
    self.state.positions.append(pitch)
    self.state.positions.append(yaw)
    # Add manipulator self.state to
    for i in range(self.manipulator_dof):
      self.state.positions.append(self.manipulator_dofs[i].getJointState().data)

    # Add velocity and acceleration as zeros as they are not measured.
    self.state.velocities.extend([0]*len(self.state.positions))
    self.state.accelerations.extend([0]*len(self.state.positions))

    return self.state

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