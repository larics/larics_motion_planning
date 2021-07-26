#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import copy

class ImpedanceJointTrajectoryHandler:

  def __init__(self):
    # Joint trajectory point is only published while trajectory is non-empty
    self.joint_trajectory_point_pub = rospy.Publisher(
      'impedance_joint_trajectory_handler/joint_trajectory_point_out', 
      JointTrajectoryPoint, queue_size=1)

    # Parameters
    self.rate = rospy.get_param('~rate', 100)
    # Delay timer determines how much will some axes be delayed
    self.delay_time = rospy.get_param('~delay_time', 0.1)
    #print("DELAY: ", self.delay_time)
    self.delay_axes = rospy.get_param('~delay_axes', [9, 10, 11, 12, 13, 14, 15])
    self.delay_buffers = []
    self.buffer_length = int(self.delay_time*float(self.rate))
    self.delay_counter = copy.deepcopy(self.buffer_length)
    for i in range(len(self.delay_axes)):
      buf = DelayAxisBuffer(self.buffer_length, 0)
      self.delay_buffers.append(copy.deepcopy(buf))

    # First point received flag
    self.first_trajectory_point_received_flag = False
    
    self.current_trajectory_point = JointTrajectoryPoint()
    self.publish_trajectory_point = JointTrajectoryPoint()

    # Subscriber for the full joint trajectory
    rospy.Subscriber('impedance_joint_trajectory_handler/joint_trajectory_point_in', 
      JointTrajectoryPoint, self.jointTrajectoryPointCallback, queue_size=1)

  def run(self):
    rate = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      rate.sleep()

      if ((self.first_trajectory_point_received_flag == True) and 
        self.delay_counter >= 0):
        # Set trajectory positions
        lst = list(self.current_trajectory_point.positions)      
        for i in range(len(self.delay_axes)):
          self.delay_buffers[i].update(self.current_trajectory_point.positions[self.delay_axes[i]])
          lst[self.delay_axes[i]] = self.delay_buffers[i].getValue()

        self.publish_trajectory_point = copy.deepcopy(
          self.current_trajectory_point)
        self.publish_trajectory_point.positions = tuple(lst)
        # Publish and continue
        self.joint_trajectory_point_pub.publish(self.publish_trajectory_point)
        self.delay_counter = self.delay_counter - 1

  def jointTrajectoryPointCallback(self, msg):
    # On first pass reset buffer
    if self.first_trajectory_point_received_flag == False:
      for i in range(len(self.delay_axes)):
        self.delay_buffers[i].resetBuffer(msg.positions[self.delay_axes[i]])
    self.first_trajectory_point_received_flag = True
    self.current_trajectory_point = msg
    self.delay_counter = copy.deepcopy(self.buffer_length)


class DelayAxisBuffer:

  def __init__(self, buffer_length, initial_value):
    self.buffer = [initial_value]*buffer_length
    self.index = 0
    self.buffer_length = len(self.buffer)
    print("Buffer length: ", self.buffer_length, buffer_length)

  def resetBuffer(self, value):
    print("Resetting buffer")
    self.index = 0
    for i in range(self.buffer_length):
      self.buffer[i] = value

  def update(self, value):
    self.buffer[self.index] = value
    self.index = self.index + 1
    self.index = self.index % self.buffer_length

  def getValue(self):
    return self.buffer[self.index]

  def printBuffer(self):
    print("Buffer: ", self.buffer)
    print("Index: ", self.index)
    print("Current: ", self.buffer[self.index])

if __name__ == '__main__':

  rospy.init_node('impedance_joint_trajectory_handler')
  handler = ImpedanceJointTrajectoryHandler()
  handler.run()

