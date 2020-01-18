#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos
from geometry_msgs.msg import Twist, Transform
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float32
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
    MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint
import copy

class JointTrajectoryToMultiDofTrajectoryPoint:

    def __init__(self):
        # UAV publishers for trajectory
        self.uav_trajectory_point_pub = rospy.Publisher('trajectory_point_ref', 
            MultiDOFJointTrajectoryPoint, queue_size=1)
        self.executing_trajectory_pub = rospy.Publisher('executing_trajectory', 
            Int32, queue_size=1)

        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(self.rate) 
        self.airdrop_flag = rospy.get_param('~airdrop', False)
        if self.airdrop_flag == True:
            self.magnet_pub = rospy.Publisher('magnet/gain', Float32, queue_size=1)
            self.magnet_gain = 1.0
            self.airdrop_counter = 0
        self.t_start = rospy.Time.now()
        self.executing_trajectory_flag = False
        self.joint_trajectory = JointTrajectory()
        self.current_trajectory_point = JointTrajectoryPoint()
        self.uav_current_trajectory_point = MultiDOFJointTrajectoryPoint()

        rospy.Subscriber('joint_trajectory', JointTrajectory, 
            self.jointTrajectoryCallback, queue_size=1)

    def run(self):
        
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

            if self.executing_trajectory_flag == True:
                self.executing_trajectory_pub.publish(1)
                # Take first point from trajectory, publish it and remove it
                # from trajectory
                self.current_trajectory_point = self.joint_trajectory.points[0]
                self.uav_current_trajectory_point = jointTrajectoryPointToMultiDofJointTrajectoryPoint(
                    self.current_trajectory_point)
                if self.airdrop_flag == True:
                    if abs(self.joint_trajectory.points[0].positions[4]) < 0.1:
                        self.magnet_gain = 1.0
                    elif abs(self.joint_trajectory.points[0].positions[4] - 1.0) < 0.1:
                        self.airdrop_counter = self.airdrop_counter + 1
                    if self.airdrop_counter >= 12:
                        self.magnet_gain = 0.0
                    self.magnet_pub.publish(self.magnet_gain)
                self.publishAll()
                #self.joint_trajectory_point_pub.publish(self.current_trajectory_point)
                self.joint_trajectory.points.pop(0)
                
                if len(self.joint_trajectory.points) == 0:
                    self.executing_trajectory_flag = False
                    if self.airdrop_flag == True:
                        self.magnet_gain = 1.0
                        self.airdrop_counter = 0
                        self.magnet_pub.publish(self.magnet_gain)
            else:
                self.executing_trajectory_pub.publish(0)


    def jointTrajectoryCallback(self, msg):
        print "Received a trajectory."
        if self.executing_trajectory_flag == False and \
            len(msg.points) > 0:
            self.joint_trajectory = copy.deepcopy(msg)
            self.executing_trajectory_flag = True
        else:
            print "Currently executing a trajectory."

    def publishAll(self):
        self.uav_trajectory_point_pub.publish(self.uav_current_trajectory_point)


def jointTrajectoryPointToMultiDofJointTrajectoryPoint(joint):
    multi = MultiDOFJointTrajectoryPoint()

    transform = Transform()
    transform.translation.x = joint.positions[0]
    transform.translation.y = joint.positions[1]
    transform.translation.z = joint.positions[2]
    transform.rotation.z = math.sin(joint.positions[3]/2.0)
    transform.rotation.w = math.cos(joint.positions[3]/2.0)

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

    rospy.init_node('joint_trajectory_to_multi_dof_trajectory_point')
    trajectory_to_ref = JointTrajectoryToMultiDofTrajectoryPoint()
    trajectory_to_ref.run()

