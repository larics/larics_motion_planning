#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import copy
import yaml
from UavAndWpManipulatorHandler import *


class MultipleManipulatorsJointTrajectoryHandler:

  def __init__(self):
    # Parameters
    self.rate = rospy.get_param('~rate', 100)
    config_file = rospy.get_param('~config_file', 
      'config/multiple_manipulators/two_uavs_and_wp_manipulator.yaml')

    # State publisher of all manipulators combined.
    self.full_state = JointTrajectoryPoint()
    self.full_state_pub = rospy.Publisher(
      'multiple_manipulators_joint_trajectory_handler/full_state',
      JointTrajectoryPoint, queue_size=1)
    
    # Open the config file and add all manipulators
    s = open(config_file, "r")
    config = yaml.safe_load(s)
    self.n_manipulators = len(config["trajectory_handler"])
    self.manipulators = []
    self.start_indexes = []
    self.end_indexes = []
    self.total_dof = 0
    for i in range(self.n_manipulators):
      # Get the appropriate configuration and append trajectory handler
      # accordingly
      if config["trajectory_handler"][i]["type"] == "UavAndWpManipulatorHandler":
        current_manipulator = UavAndWpManipulatorHandler(
          config["trajectory_handler"][i])
        self.manipulators.append(copy.copy(current_manipulator))
      else:
        # If there are no trajectory handlers, simply exit.
        print("multiple_manipulators_joint_trajectory_handler.py")
        ss = "  No such trajectory handler type: " + \
          config["trajectory_handler"][i]["type"]
        print(ss)
        quit()
      # Load start and end indexes
      self.start_indexes.append(
        config["trajectory_handler"][i]["indexes"]["start"])
      self.end_indexes.append(
        config["trajectory_handler"][i]["indexes"]["end"])
      self.total_dof = self.total_dof + self.end_indexes[i] \
        - self.start_indexes[i] + 1

    # Based on total dof, create full manipulator state.
    for i in range(self.total_dof):
      self.full_state.positions.append(0.0)
      self.full_state.velocities.append(0.0)
      self.full_state.accelerations.append(0.0)

    # Subscriber for joint trajectory point with all degrees of freedom
    self.current_trajectory_point = JointTrajectoryPoint()
    rospy.Subscriber(
      'multiple_manipulators_joint_trajectory_handler/joint_trajectory_point_in', 
      JointTrajectoryPoint, self.jointTrajectoryPointCallback, queue_size=1)

  def run(self):
    rate = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      rate.sleep()

      # Create full state based on each manipulator state
      for i in range(self.n_manipulators):
        current_state = self.manipulators[i].getCurrentState()
        for j in range(self.start_indexes[i], self.end_indexes[i]+1):
          self.full_state.positions[j] = current_state.positions[j-self.end_indexes[i]-1]
          self.full_state.velocities[j] = current_state.velocities[j-self.end_indexes[i]-1]
          self.full_state.accelerations[j] = current_state.accelerations[j-self.end_indexes[i]-1]
      self.full_state_pub.publish(self.full_state)

    #rospy.spin()


  def jointTrajectoryPointCallback(self, msg):
    # Get the trajectory point and publish it.
    self.current_trajectory_point = msg
    self.publishAll()

  def publishAll(self):
    # Go through all manipulators and based on start and end indexes extract
    # values for each manipulator accordingly. If the number of dofs here is
    # not as the one from config file, throw an error.
    current_dof = len(self.current_trajectory_point.positions)
    if self.total_dof == current_dof:
      for i in range(self.n_manipulators):
        current_point = segmentJointTrajectoryPoint(self.current_trajectory_point, 
          self.start_indexes[i], self.end_indexes[i])
        # After segmentation, publish all 
        self.manipulators[i].publish(current_point)
    else:
      # If the received point's DoF differs from total dof, print error and
      # don't publish
      print("multiple_manipulators_joint_trajectory_handler.py")
      ss = "  Rejecting publish. There are " + str(current_dof) + \
        " degrees of freedom in received point. \n  Required number is: " + \
        str(self.total_dof)  
      print(ss)


def segmentJointTrajectoryPoint(joint, start, end):
  segmented_point = JointTrajectoryPoint()
  # Go from start to end and append values to segmented point.
  for i in range(start, end+1):
    segmented_point.positions.append(joint.positions[i])
    segmented_point.velocities.append(joint.velocities[i])
    segmented_point.accelerations.append(joint.accelerations[i])

  return segmented_point


if __name__ == '__main__':
  rospy.init_node('multiple_manipulators_joint_trajectory_handler')
  handler = MultipleManipulatorsJointTrajectoryHandler()
  handler.run()