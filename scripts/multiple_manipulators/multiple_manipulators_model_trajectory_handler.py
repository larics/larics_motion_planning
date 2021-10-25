#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from larics_motion_planning.srv import MultiDofTrajectory, \
  MultiDofTrajectoryRequest, MultiDofTrajectoryResponse
import copy
import yaml

class MultipleManipulatorsModelTrajectoryHandler:

  def __init__(self):
    # Parameters
    self.rate = rospy.get_param('~rate', 100)

    # Service for executing model trajectory
    self.execute_trajectory_service = rospy.Service('execute_trajectory', 
      MultiDofTrajectory, self.executeTrajectoryCallback)


  def executeTrajectoryCallback(self, req):
    response = MultiDofTrajectoryResponse()

    return response

if __name__ == '__main__':
  rospy.init_node('multiple_manipulators_model_trajectory_handler')
  handler = MultipleManipulatorsModelTrajectoryHandler()
  handler.run()