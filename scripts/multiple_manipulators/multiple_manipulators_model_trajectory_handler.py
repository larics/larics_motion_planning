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
    self.ros_rate = rospy.Rate(self.rate)

    # UAV publishers for trajectory
    self.executing_trajectory_indicator = Int32()
    self.executing_trajectory_indicator.data = 0
    self.executing_trajectory_pub = rospy.Publisher(
      'multiple_manipulators_model_trajectory_handler/executing_model_trajectory', 
      Int32, queue_size=1)
    self.trajectory_go_to_pub = rospy.Publisher(
      'go_to/reference/joint_trajectory_point', 
      JointTrajectoryPoint, queue_size=1)
    # This next publisher will completely bypass the reference tracker.
    # aerial_manipulator_control/trajectory_ref_input
    self.joint_trajectory_point_pub = rospy.Publisher(
      'multiple_manipulators_model_trajectory_handler/joint_trajectory_point', 
      JointTrajectoryPoint, queue_size=1)
    # At the end, we have to set current reference of the reference tracker
    # to the proper value.
    self.set_reference_tracker_point_pub = rospy.Publisher(
      'reference_tracker/set_current_trajectory_point',
      JointTrajectoryPoint, queue_size=1)

    # Since this node will record the trajectory and return the recorded
    # values, this termination timer will record for some time after the last
    # trajectory point has been sent.
    self.termination_timer = rospy.get_param('~record_termination/timer', 2.0)
    
    # Trajectory and current trajectory point to be sent to the multiple
    # manipulators trajectory handler
    self.trajectory = JointTrajectory()
    self.current_trajectory_point = JointTrajectoryPoint()

    # In the service callback, we will request for the UAV to move to the
    # initial position. We have to wait until it gets to the initial position
    # and since it is handled through another node, we simply check its 
    # flag if the trajectory finished executing.
    self.executing_trajectory_in_other_node = 0
    self.executing_trajectory_in_other_node_previous = 0
    rospy.Subscriber('reference_tracker/executing_trajectory', Int32, 
      self.executingTrajectoryInOtherNodeCallback, queue_size=1)

    # Subscriber for the current state of multiple manipulators
    self.current_full_state = JointTrajectoryPoint()
    rospy.Subscriber('multiple_manipulators_joint_trajectory_handler/full_state',
      JointTrajectoryPoint, self.fullStateCallback, queue_size=1)

    # Service for executing model trajectory
    self.execute_trajectory_service = rospy.Service('execute_trajectory', 
      MultiDofTrajectory, self.executeTrajectoryCallback)


  def run(self):
    rate = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      rate.sleep()
      self.executing_trajectory_pub.publish(self.executing_trajectory_indicator)

  def executingTrajectoryInOtherNodeCallback(self, msg):
    self.executing_trajectory_in_other_node_previous = \
      copy.deepcopy(self.executing_trajectory_in_other_node)
    self.executing_trajectory_in_other_node = msg.data

  def fullStateCallback(self, msg):
    self.current_full_state = copy.deepcopy(msg)

  def executeTrajectoryCallback(self, req):
    # Create response, empty for now
    response = MultiDofTrajectoryResponse()

    # If someone sends empty trajectory simply exit.
    if len(req.waypoints.points) <= 1:
      print("multiple_manipulators_model_trajectory_handler.py")
      print("  Something went wrong. 0 points in trajectory.")
      response = MultiDofTrajectoryResponse()
      response.path_length = -1.0
      return response

    # Send the model UAV to the initial point
    self.trajectory_go_to_pub.publish(req.waypoints.points[0])
    rospy.sleep(0.5)
    start_time = rospy.Time.now().to_sec()
    initial_timer = 0.0
    temp_rate = rospy.Rate(self.rate*2)
    # First loop waits for 2s to see if trajectory started. If it has not
    # started that means the model UAV is already at the right spot so we
    # can move directly to executing the trajectory
    trajectory_started_flag = True
    while (not rospy.is_shutdown()) and (initial_timer < 2.0):
      if (self.executing_trajectory_in_other_node_previous == 1) or \
        (self.executing_trajectory_in_other_node == 1):
        trajectory_started_flag = False
      initial_timer = rospy.Time.now().to_sec() - start_time
      temp_rate.sleep()

    # If trajectory has not started, the model UAV is at the right spot.
    # Otherwise wait for the model UAV to get to the right spot.
    if (trajectory_started_flag  == False):
      print("multiple_manipulators_model_trajectory_handler.py")
      print("  Model UAV not at required point. Moving model to first point.")
      # Second loop waits for 2s so we are sure trajectory started
      start_time = rospy.Time.now().to_sec()
      initial_timer = 0.0
      trajectory_end_flag = False
      while (not rospy.is_shutdown()) and ((initial_timer < 2.0) or \
        (trajectory_end_flag == False)):
        if (self.executing_trajectory_in_other_node_previous == 0) and \
          (self.executing_trajectory_in_other_node == 0):
          trajectory_end_flag = True
        initial_timer = rospy.Time.now().to_sec() - start_time
        temp_rate.sleep()
      print("multiple_manipulators_model_trajectory_handler.py")
      print("  Model trajectory executed. Waiting for 5s to start with intended trajectory.")
      # At this point the trajectory should ended so just wait for 5s so
      # that the model UAV can settle down.
      start_time = rospy.Time.now().to_sec()
      while (not rospy.is_shutdown()) and ((rospy.Time.now().to_sec() - start_time) < 5.0):
        temp_rate.sleep()


    # Start executing trajectory
    print("multiple_manipulators_model_trajectory_handler.py")
    print("  Starting to execute trajectory. Length: ", len(req.waypoints.points))
    # Go through all points and execute the trajectory point by point
    rate = rospy.Rate(self.rate)
    self.executing_trajectory_indicator.data = 1
    for i in range(len(req.waypoints.points)):
      self.current_trajectory_point = req.waypoints.points[i]
      self.publishAll()

      # Set the current full state in the response
      response.executed_trajectory.points.append(copy.deepcopy(
        self.current_full_state))

      rate.sleep()
    self.executing_trajectory_indicator.data = 0

    # Since the UAV 'lags' when executing the trajectory we want to
    # prolong the roll and pitch recording until the UAV gets to the steady
    # state.
    tstart = rospy.Time.now().to_sec()
    iteration = 1
    while True: #(((abs(self.roll) > 0.001) or (abs(self.pitch) > 0.001)) or ((rospy.Time.now().to_sec() - tstart) < 0.5)):
      # Simply put current full state in executed trajectory.
      response.executed_trajectory.points.append(copy.deepcopy(
        self.current_full_state))

      # This can be checked through roll and pitch angles.
      # Also minimum time is required.
      if ((rospy.Time.now().to_sec()-tstart) > self.termination_timer):
        print("multiple_manipulators_model_trajectory_handler.py")
        print("  Trajectory recorded.")
        print("  Extra time: ", (rospy.Time.now().to_sec()-tstart))
        break

      rate.sleep()

    print("  Lenghtened trajectory length: ", len(
      response.executed_trajectory.points))
    print("  Trajectory executed!")

    return response

  def publishAll(self):
    self.joint_trajectory_point_pub.publish(self.current_trajectory_point)
    self.set_reference_tracker_point_pub.publish(self.current_trajectory_point)
    self.executing_trajectory_pub.publish(self.executing_trajectory_indicator)

if __name__ == '__main__':
  rospy.init_node('multiple_manipulators_model_trajectory_handler')
  handler = MultipleManipulatorsModelTrajectoryHandler()
  handler.run()