#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math, time
from math import sin, cos
from geometry_msgs.msg import Twist, Transform, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float32, String
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
  MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint

from larics_motion_planning.srv import MultiDofTrajectory, \
  MultiDofTrajectoryRequest, MultiDofTrajectoryResponse
import copy

class UavExplorationSm:

  def __init__(self):
    self.state = "start"
    self.state_previous = "none"
    self.target_pose = Pose()
    self.current_pose = Pose()
    self.first_measurement_received = False
    self.executing_trajectory = 0
    self.start_time = time.time()
    self.n_execution_attempts = 0
    self.execution_start = time.time()
    self.current_trajectory_execution_time = 0

    # Initialize ROS params
    # Distance of the UAV from the target pose at which we consider the 
    # trajectory to be executed
    self.r_trajectory = rospy.get_param('~radius_trajectory_executed', 0.3)
    # Distance of the UAV from its mean position at which we consider it to 
    # be stuck in potential field minimum.
    self.r_potential_field_minimum = rospy.get_param(
      '~radius_potential_field_minimum', 0.3)
    # Trajectory execution time varies with its length. Alongside radius, we 
    # also wait for some time after trajectory has been executed to allow for
    # settling before we can conclude that the UAV is indeed stuck.
    self.potential_field_minimum_time_factor = rospy.get_param(
      '~potential_field_minimum_time_factor', 1.5)
    # Pure timeout factor. If trajectory execution lasts x times longer than
    # planned and no executed or stuck states have been detected, simply 
    # terminate this execution attempt.
    self.execution_timeout_factor = rospy.get_param(
      '~execution_timeout_factor', 5.0)
    # Maximum number of execution attempts. Afterwards, we terminate the state
    # machine
    self.max_execution_attempts = rospy.get_param('~max_execution_attempts', int(5))
    # Rate of the state machine.
    self.rate = rospy.get_param('~rate', 10)
    # How long do we collect feedback and store it in array. Checks for execution
    # and stuck rely on this parameter because they check through last x 
    # seconds to determine next state.
    self.feedback_collection_time = rospy.get_param('~feedback_collection_time', 5.0)
    self.dt = 1.0/float(self.rate)
    self.n_feedback_points = int(self.rate*self.feedback_collection_time)

    # Set up array of feedback poses
    self.feedback_array = []
    self.feedback_array_index = 0
    for i in range(self.n_feedback_points):
      temp_pose = Pose()
      self.feedback_array.append(copy.deepcopy(temp_pose))

    # Initialize publishers
    self.state_pub = rospy.Publisher('exploration/current_state', String, 
      queue_size=1, latch=True)
    self.trajectory_pub = rospy.Publisher('joint_trajectory', JointTrajectory, 
      queue_size=1)
    self.executing_pub = rospy.Publisher('exploration/executing', Int32, 
      queue_size=1)

    # Initialize services
    rospy.wait_for_service('multi_dof_trajectory', timeout=30)
    self.plan_trajectory_service = rospy.ServiceProxy(
      "multi_dof_trajectory", MultiDofTrajectory)

    # Initialize subscribers
    rospy.Subscriber('exploration/target_pose', Pose, 
      self.targetPointCallback, queue_size=1)
    rospy.Subscriber('/mavros/global_position/local', Odometry, 
      self.globalPositionCallback, queue_size=1)
    rospy.Subscriber('executing_trajectory', Int32, 
      self.executingTrajectoryCallback, queue_size=1)

    time.sleep(0.2)

  def run(self):
    
    rate = rospy.Rate(self.rate)
    self.state_pub.publish(self.state)

    while not rospy.is_shutdown() and not self.first_measurement_received:
      print "Waiting for first measurement..."
      time.sleep(1)
    print "First measurement received. Starting state machine."

    while not rospy.is_shutdown():

      # Start state only waits for something to happen
      if self.state == "start":
        self.executing_pub.publish(0)
        if self.state_previous != self.state:
          self.printStates()
          self.state_previous = "start"
          self.state_pub.publish(self.state)

      # Planning the obstacle free trajectory in the map
      if self.state == "plan":
        self.executing_pub.publish(1)
        if self.state_previous != self.state:
          self.printStates()
          self.state_previous = "plan"
          self.state_pub.publish(self.state)

        # Call the obstacle free trajectory planning service
        request = MultiDofTrajectoryRequest()
        # Create start point from current position information
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [self.current_pose.position.x, \
          self.current_pose.position.y, self.current_pose.position.z, \
          self.quaternion2Yaw(self.current_pose.orientation)]
        request.waypoints.points.append(copy.deepcopy(trajectory_point))
        # Create start point from target position information
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [self.target_pose.position.x, \
          self.target_pose.position.y, self.target_pose.position.z, \
          self.quaternion2Yaw(self.target_pose.orientation)]
        request.waypoints.points.append(copy.deepcopy(trajectory_point))
        # Set up flags
        request.publish_path = False
        request.publish_trajectory = False
        request.plan_path = True
        request.plan_trajectory = True

        response = self.plan_trajectory_service.call(request)

        # If we did not manage to obtain a successful plan then go to
        # appropriate state.
        if response.success == False:
          print "**********************************************"
          print "In state:", self.state
          print "Path planning failed!"
          print "**********************************************"
          print " "
          self.state = "end"
        # If plan was successful then execute it.
        else:
          self.trajectory_pub.publish(response.trajectory)
          self.current_trajectory_execution_time = \
            response.trajectory.points[len(response.trajectory.points)-1].time_from_start.to_sec()

          self.state = "execute"

      # While trajectory is executing we check if it is done or if uav is 
      # stuck in potential field minimum
      if self.state == "execute":
        if self.state_previous != self.state:
          self.n_execution_attempts = self.n_execution_attempts + 1
          self.printStates()
          self.state_previous = "execute"
          self.state_pub.publish(self.state)
          self.execution_start = time.time()

        while not rospy.is_shutdown():
          self.executing_pub.publish(1)
          # If we reached predefined maximum of execution attempts consider the
          # point to be "reached". We can do this since we rely on the UAV to
          # explore a part of map during execution and new frontier will be
          # detected
          if self.n_execution_attempts >= self.max_execution_attempts:
            print "**********************************************"
            print "In state:", self.state
            print "Exceeded maximum number of execution attempts!"
            print "**********************************************"
            print " "
            self.state = "end"
            break
          # Pure timeout. It is based on trajectory duration which is multiplied
          # by factor determined in ros params
          elif (self.execution_timeout_factor*self.current_trajectory_execution_time) < (
            time.time()-self.execution_start):
            self.state = "plan"
            print "**********************************************"
            print "In state:", self.state
            print "Execution timeout factor triggered!"
            print "**********************************************"
            print " "
            break
          # When trajectory is executed simply go to end state.
          elif self.checkTrajectoryExecuted() == True:
            print "**********************************************"
            print "In state:", self.state
            print "Trajectory executed!"
            print "**********************************************"
            print " "
            self.state = "end"
            break
          # If trajectory is stuck in potential field minimum then plan new
          # trajectory. This relies on the fact that the UAV will explore a 
          # part of map in its initial run.
          elif (self.checkIfStuck() == True) and (
            self.current_trajectory_execution_time*self.potential_field_minimum_time_factor <= (
              time.time()-self.execution_start)):
            print "**********************************************"
            print "In state:", self.state
            print "Stuck in potential field minimum"
            print "**********************************************"
            print " "
            self.state = "plan"
            break
          rate.sleep()


      # End state, publish that you reached the point
      if self.state == "end":
        self.executing_pub.publish(0)
        if self.state_previous != self.state:
          self.printStates()
          self.state_previous = "end"
          self.state_pub.publish(self.state)

        # Reset relevant counters
        self.n_execution_attempts = 0
        time.sleep(0.05)
        # TODO: publish 
        self.state = "start"

      rate.sleep()

  def printStates(self):
    print "----------------------------------------------------"
    print "State changed. Previous state:", self.state_previous
    print "State changed. Current state:", self.state
    print "----------------------------------------------------"
    print " "

  def targetPointCallback(self, msg):
    if self.state == "start":
      self.target_pose = msg
      self.state = "plan"
      self.state_previous = "start"
    else:
      print "Not accepting new target pose. Current pose not yet"

  def globalPositionCallback(self, msg):
    self.current_pose = msg.pose.pose
    self.first_measurement_received = True

    # Collect array of data with rate of the loop. We can fill this list 
    # in a cyclic manner since we have info about first and last data point
    # stored in feedback_array_index
    if ((time.time()-self.start_time) > self.dt):
      self.start_time = time.time()
      self.feedback_array[self.feedback_array_index] = copy.deepcopy(self.current_pose)
      self.feedback_array_index = self.feedback_array_index + 1
      if self.feedback_array_index >= self.n_feedback_points:
        self.feedback_array_index = 0

  def executingTrajectoryCallback(self, msg):
    self.executing_trajectory = msg.data

  def quaternion2Yaw(self, quaternion):
    q0 = quaternion.w
    q1 = quaternion.x
    q2 = quaternion.y
    q3 = quaternion.z
    return math.atan2(2.0*(q0*q3 + q1*q2), 1.0-2.0*(q2*q2 + q3*q3))

  def checkTrajectoryExecuted(self):
    # Here we check if the UAV's has been within some radius from the target 
    # point for some time. If so we consider trajectory to be executed.
    for i in range(self.n_feedback_points):
      dx = self.target_pose.position.x - self.feedback_array[i].position.x
      dy = self.target_pose.position.y - self.feedback_array[i].position.y
      dz = self.target_pose.position.z - self.feedback_array[i].position.z

      delta = math.sqrt(dx*dx + dy*dy + dz*dz)

      if delta > self.r_trajectory:
        return False

    if self.executing_trajectory == 0:
      return True
    else:
      return False

  def checkIfStuck(self):
    # Similar as checking if trajectory is executed, the difference is that 
    # here we first compute mean position and then check if the UAV has been
    # within some radius from that mean for some time.

    # First find mean values
    x_mean = 0
    y_mean = 0
    z_mean = 0
    for i in range(self.n_feedback_points):
      x_mean = x_mean + self.feedback_array[i].position.x
      y_mean = y_mean + self.feedback_array[i].position.y
      z_mean = z_mean + self.feedback_array[i].position.z
    x_mean = x_mean/float(self.n_feedback_points)
    y_mean = y_mean/float(self.n_feedback_points)
    z_mean = z_mean/float(self.n_feedback_points)

    # Go through feedback points again and check if they are all within some
    # radius
    for i in range(self.n_feedback_points):
      dx = x_mean - self.feedback_array[i].position.x
      dy = y_mean - self.feedback_array[i].position.y
      dz = z_mean - self.feedback_array[i].position.z

      delta = math.sqrt(dx*dx + dy*dy + dz*dz)

      if delta > self.r_potential_field_minimum:
        return False

    if self.executing_trajectory == 0:
      return True
    else:
      return False

  def printFeedbackArray(self):
    for i in range(self.n_feedback_points):
      print self.feedback_array[i]

if __name__ == '__main__':

  rospy.init_node('uav_exploration_state_machine')
  exploration = UavExplorationSm()
  exploration.run()

