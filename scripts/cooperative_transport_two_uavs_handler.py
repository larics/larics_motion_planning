#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from std_msgs.msg import Float64
from geometry_msgs.msg import Transform, Twist
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint
import copy

class CooperativeTransportTwoUavsHandler:

  def __init__(self):
    # Joint trajectory point is only published while trajectory is non-empty
    #self.joint_trajectory_point_pub = rospy.Publisher(
    #  'cooperative_transport_two_uavs_trajectory_handler/joint_trajectory_point_out', 
    #  JointTrajectoryPoint, queue_size=1)

    # Parameters
    self.rate = rospy.get_param('~rate', 100)
    self.uav1_namespace = rospy.get_param('~first_uav_namespace', '/uav1')
    self.uav2_namespace = rospy.get_param('~second_uav_namespace', '/uav2')
    self.manipulator_dof = rospy.get_param('~manipulator_dof', int(5))

    # UAV1 publishers. First trajectory
    self.uav1_trajectory_point_pub = rospy.Publisher(
      self.uav1_namespace + '/trajectory_point_ref',
      MultiDOFJointTrajectoryPoint, queue_size=1)
    self.uav1_trajectory_point = MultiDOFJointTrajectoryPoint()
    # Publishers for manipulator joints
    self.uav1_manipulator_pubs = []
    for i in range(self.manipulator_dof):
      current_pub = JointPositionControllerPublisher(self.uav1_namespace + 
        '/joint' + str(i+1) + '_position_controller/command')
      self.uav1_manipulator_pubs.append(copy.copy(current_pub))

    # UAV2 publishers. Trajectory publisher
    self.uav2_trajectory_point_pub = rospy.Publisher(
      self.uav2_namespace + '/trajectory_point_ref',
      MultiDOFJointTrajectoryPoint, queue_size=1)
    self.uav2_trajectory_point = MultiDOFJointTrajectoryPoint()
    # Publishers for manipulator joints
    self.uav2_manipulator_pubs = []
    for i in range(self.manipulator_dof):
      current_pub = JointPositionControllerPublisher(self.uav2_namespace + 
        '/joint' + str(i+1) + '_position_controller/command')
      self.uav2_manipulator_pubs.append(copy.copy(current_pub))

    # First point received flag
    self.first_trajectory_point_received_flag = False
    
    self.current_joint_trajectory_point = JointTrajectoryPoint()

    # Subscriber for the joint trajectory point
    rospy.Subscriber('cooperative_transport_two_uavs_trajectory_handler/joint_trajectory_point_in', 
      JointTrajectoryPoint, self.jointTrajectoryPointCallback, queue_size=1)

  def run(self):
    rate = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      rate.sleep()

      if (self.first_trajectory_point_received_flag == True):
        # UAV1 joint trajectory point
        offset = 0
        self.uav1_trajectory_point = createMultiDofJointTrajectoryPoint(
          self.current_joint_trajectory_point, offset)
        self.uav1_trajectory_point_pub.publish(self.uav1_trajectory_point)
        # UAV1 manipulator command
        for i in range(self.manipulator_dof):
          self.uav1_manipulator_pubs[i].publish(
            self.current_joint_trajectory_point.positions[offset+i+6])

        # UAV2 joint trajectory point
        offset = 11
        self.uav2_trajectory_point = createMultiDofJointTrajectoryPoint(
          self.current_joint_trajectory_point, offset)
        self.uav2_trajectory_point_pub.publish(self.uav2_trajectory_point)
        # UAV2 manipulator command
        for i in range(self.manipulator_dof):
          self.uav2_manipulator_pubs[i].publish(
            self.current_joint_trajectory_point.positions[offset+i+6])

  def jointTrajectoryPointCallback(self, msg):
    self.first_trajectory_point_received_flag = True
    self.current_joint_trajectory_point = msg


# Helper class for joint position command
class JointPositionControllerPublisher:

    def __init__(self, topic):
        self.pub = rospy.Publisher(topic, Float64, queue_size=1)

    def publish(self, value):
        self.pub.publish(Float64(value))

def createMultiDofJointTrajectoryPoint(joint, offset):
    multi = MultiDOFJointTrajectoryPoint()

    transform = Transform()
    transform.translation.x = joint.positions[0+offset]
    transform.translation.y = joint.positions[1+offset]
    transform.translation.z = joint.positions[2+offset]
    transform.rotation.z = math.sin(joint.positions[5+offset]/2.0)
    transform.rotation.w = math.cos(joint.positions[5+offset]/2.0)

    vel = Twist()
    vel.linear.x = joint.velocities[0+offset]
    vel.linear.y = joint.velocities[1+offset]
    vel.linear.z = joint.velocities[2+offset]

    acc = Twist()
    acc.linear.x = joint.accelerations[0+offset]
    acc.linear.y = joint.accelerations[1+offset]
    acc.linear.z = joint.accelerations[2+offset]

    multi.transforms.append(transform)
    multi.velocities.append(vel)
    multi.accelerations.append(acc)

    return multi

if __name__ == '__main__':

  rospy.init_node('cooperative_transport_two_uavs_trajectory_handler')
  handler = CooperativeTransportTwoUavsHandler()
  handler.run()

