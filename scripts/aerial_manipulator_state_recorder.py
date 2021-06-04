#!/usr/bin/env python

__author__ = 'aivanovic'

import sys, copy, os, getpass
import rospy, tf2_ros, tf_conversions, tf
import cv2
import numpy as np
import math
import time
from std_msgs.msg import Int32, Float64
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import JointControllerState
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

class AerialManipulatorStateRecorder:

  def __init__(self):    
    self.rate = rospy.get_param('~rate', 24)

    self.uav_pose = PoseStamped()
    self.executing_trajectory = 0
    self.executing_trajectory_previous = 0
    self.timer_duration = rospy.get_param('~duration_after_trajectory', 5.0)
    self.timer_start = 0.0

    # Open file
    self.directory = rospy.get_param('~directory', "/home/"+getpass.getuser())
    self.filename = rospy.get_param('~filename', "aerial_manipulator")
    extension = ".csv"
    i = 0
    check_filename = copy.deepcopy(self.filename)
    while os.path.exists(self.directory + "/" + check_filename + extension):
      i = i + 1
      check_filename = copy.deepcopy(self.filename) + "_" + str(i)
    self.aerial_manipulator_state_file = open(self.directory + "/" + check_filename + 
      extension, "w")
    print("[AerialManipulatorStateRecorder] Saving to file")
    print(self.directory + "/" + check_filename + extension)

    self.recording_flag = False
    self.recording_flag_service = rospy.Service('record_trajectory', 
      SetBool, self.recordingFlagCallback)

    # Subscribers
    rospy.Subscriber('pose', PoseStamped,
      self.uavPoseCallback, queue_size=1)
    rospy.Subscriber('executing_trajectory', Int32,
      self.executingTrajectoryCallback, queue_size=1)
    self.ee_pose = PoseStamped()
    rospy.Subscriber('aerial_manipulator_control/pose_output', PoseStamped,
      self.endEffectorPoseCallback, queue_size=1)

    # Subscribers for manipulator references
    self.manipulator_joint_states = []
    self.dof_manipulator = rospy.get_param("~manipulator_dof", int(3))
    print("[AerialManipulatorStateRecorder]: Using " + 
      str(self.dof_manipulator) + " dof manipulator.")
    for i in range(self.dof_manipulator):
      joint = JointPositionControllerSubscriber()
      self.manipulator_joint_states.append(copy.deepcopy(joint))
      topic = "joint" + str(i+1) + "_position_controller/state" 
      rospy.Subscriber(topic, JointControllerState, 
        self.manipulator_joint_states[i].jointStateCallback, queue_size=1)

  def run(self):
    rate = rospy.Rate(self.rate)
    t_start = time.time()
    header_line = "time,uav_x,uav_y,uav_z,uav_qx,uav_qy,uav_qz,uav_qw"
    header_line = header_line + ",ee_x,ee_y,ee_z,ee_qx,ee_qy,ee_qz,ee_qw"
    for i in range(self.dof_manipulator):
      header_line = header_line + ",q" + str(i+1)
    self.aerial_manipulator_state_file.write(header_line)
    while not rospy.is_shutdown():
      rate.sleep()

      # If trajectory is executing start recording
      if (self.executing_trajectory == 1) or \
        ((time.time() - self.timer_start) < self.timer_duration) or \
        (self.recording_flag == True):
        line = "\n" + str(time.time()-t_start) + ',' + \
          str(self.uav_pose.pose.position.x) + ',' + \
          str(self.uav_pose.pose.position.y) + ',' + \
          str(self.uav_pose.pose.position.z) + ',' + \
          str(self.uav_pose.pose.orientation.x) + ',' + \
          str(self.uav_pose.pose.orientation.y) + ',' + \
          str(self.uav_pose.pose.orientation.z) + ',' + \
          str(self.uav_pose.pose.orientation.w) + ',' + \
          str(self.ee_pose.pose.position.x) + ',' + \
          str(self.ee_pose.pose.position.y) + ',' + \
          str(self.ee_pose.pose.position.z) + ',' + \
          str(self.ee_pose.pose.orientation.x) + ',' + \
          str(self.ee_pose.pose.orientation.y) + ',' + \
          str(self.ee_pose.pose.orientation.z) + ',' + \
          str(self.ee_pose.pose.orientation.w)
        for i in range(self.dof_manipulator):
          line = line + "," + str(self.manipulator_joint_states[i].joint_state)
        self.aerial_manipulator_state_file.write(line)

    self.aerial_manipulator_state_file.close()

  def recordingFlagCallback(self, req):
    res = SetBoolResponse()
    if (req.data == self.recording_flag):
      res.message = "Flag is already set to requested value."
      res.success = False
    else:
      self.recording_flag = req.data
      res.success = True
      if self.recording_flag == True:
        res.message = "Start recording."
      else:
        res.message = "Stop recording."

    return res

  def uavPoseCallback(self, msg):
    self.uav_pose = msg

  def endEffectorPoseCallback(self, msg):
    self.ee_pose = msg

  def executingTrajectoryCallback(self, msg):
    self.executing_trajectory = msg.data
    if (self.executing_trajectory == 0 and self.executing_trajectory_previous == 1):
      self.timer_start = time.time()
    self.executing_trajectory_previous = copy.deepcopy(self.executing_trajectory)



# Helper class for joint position subscriber
class JointPositionControllerSubscriber:

    def __init__(self):
        self.joint_state = 0.0

    def jointStateCallback(self, msg):
        self.joint_state = msg.process_value
        #print("Joint data is: ", self.joint_state)

def main(args):
  rospy.init_node('aerial_manipulator_state_recorder', anonymous=True)
  recorder = AerialManipulatorStateRecorder()
  recorder.run()

if __name__ == '__main__':
    main(sys.argv)