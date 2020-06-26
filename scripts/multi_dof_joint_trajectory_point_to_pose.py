#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos, sqrt
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
    MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint
import copy
import time

class MultiDofJointTrajectoryPointToPose:

    def __init__(self):
        # UAV publishers for trajectory
        self.pose_pub = rospy.Publisher('visualization/pose', 
            PoseStamped, queue_size=1)
        rospy.Subscriber('position_hold/trajectory', 
            MultiDOFJointTrajectoryPoint, self.trajectoryPointCallback, queue_size=1)

    def run(self):
        rospy.spin()

    def trajectoryPointCallback(self, msg):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now();
        pose.header.frame_id = "world"
        pose.pose.position.x = msg.transforms[0].translation.x
        pose.pose.position.y = msg.transforms[0].translation.y
        pose.pose.position.z = msg.transforms[0].translation.z
        pose.pose.orientation.x = msg.transforms[0].rotation.x
        pose.pose.orientation.y = msg.transforms[0].rotation.y
        pose.pose.orientation.z = msg.transforms[0].rotation.z
        pose.pose.orientation.w = msg.transforms[0].rotation.w

        self.pose_pub.publish(pose)

if __name__ == '__main__':

    rospy.init_node('multi_dof_joint_trajectory_point_to_pose')
    trajectory_to_pose = MultiDofJointTrajectoryPointToPose()
    trajectory_to_pose.run()