#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
  MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
import copy
import yaml


import rospkg
rospack = rospkg.RosPack()
lmp_path = rospack.get_path('larics_motion_planning')
import sys
sys.path.append(lmp_path+"/scripts/multiple_manipulators")
from JointPositionControllerHandler import *


class JointTrajectoryPointToArducopterAndManipulator:

  def __init__(self):
    # Parameters
    self.rate = rospy.get_param('~rate', 100)
    #config_file = rospy.get_param('~config_file', 
    #  'config/multiple_manipulators/two_uavs_and_wp_manipulator.yaml')

    # Set up degrees of freedom
    self.uav_n_dof = rospy.get_param('~uav_n_dof', 6)
    self.manipulator_n_dof = rospy.get_param('~manipulator_n_dof', 5)
    #self.uav_n_dof = 6
    #self.manipulator_n_dof = 5

    # Publishers for each joint of the manipulator. These are in
    # JointPositionControllerHandler class which also subscribes to each
    # joint feedback. If none is sent, it does not subscribe.
    self.manipulator_dofs = []
    for i in range(self.manipulator_n_dof):
      current_pub_topic = "joint" + str(i+1) + "_position_controller/command"
      current_sub_topic = "none"
      current_handler = JointPositionControllerHandler(current_pub_topic,
        current_sub_topic, 64) # 32 is Float32 as message type.
      self.manipulator_dofs.append(copy.copy(current_handler))

    # Publisher for the UAV trajectory point
    self.uav_trajectory_point_pub = rospy.Publisher(
      'joint_trajectory_point_to_arducopter_and_manipulator/multi_dof_trajectory_point_out', 
      MultiDOFJointTrajectoryPoint, queue_size=1)

    # Subscriber for joint trajectory point with all degrees of freedom
    self.current_trajectory_point = JointTrajectoryPoint()
    rospy.Subscriber(
      'joint_trajectory_point_to_arducopter_and_manipulator/joint_trajectory_point_in', 
      JointTrajectoryPoint, self.jointTrajectoryPointCallback, queue_size=1)

  def run(self):
    rospy.spin()
    #rate = rospy.Rate(self.rate)
    #while not rospy.is_shutdown():
    #  rate.sleep()


  def jointTrajectoryPointCallback(self, msg):
    # Get the trajectory point and publish it.
    self.current_trajectory_point = msg
    self.publishAll()

  def publishAll(self):
    # First part of the message relates to the UAV. The whole jointTrajectoryPoint
    # is sent since the first six values are related to the UAV.
    multi_dof_trajectory_point = self.jointTrajectoryPointToMultiDofJointTrajectoryPoint(
      self.current_trajectory_point)
    self.uav_trajectory_point_pub.publish(multi_dof_trajectory_point)

    # Next publish manipulator joint values
    for i in range(self.manipulator_n_dof):
      self.manipulator_dofs[i].publish(
        self.current_trajectory_point.positions[self.uav_n_dof+i])

  def jointTrajectoryPointToMultiDofJointTrajectoryPoint(self, joint):
    multi = MultiDOFJointTrajectoryPoint()

    transform = Transform()
    transform.translation.x = joint.positions[0]
    transform.translation.y = joint.positions[1]
    transform.translation.z = joint.positions[2]
    # Index 5 denotes yaw because we have roll and pitch as placeholders
    transform.rotation.z = math.sin(joint.positions[self.uav_n_dof-1]/2.0)
    transform.rotation.w = math.cos(joint.positions[self.uav_n_dof-1]/2.0)

    vel = Twist()
    vel.linear.x = joint.velocities[0]
    vel.linear.y = joint.velocities[1]
    vel.linear.z = joint.velocities[2]

    acc = Twist()
    acc.linear.x = joint.accelerations[0]
    acc.linear.y = joint.accelerations[1]
    acc.linear.z = joint.accelerations[2]

    multi.transforms.append(transform)
    multi.velocities.append(vel)
    multi.accelerations.append(acc)

    return multi


if __name__ == '__main__':
  rospy.init_node('joint_trajectory_point_to_arducopter_and_manipulator')
  handler = JointTrajectoryPointToArducopterAndManipulator()
  handler.run()