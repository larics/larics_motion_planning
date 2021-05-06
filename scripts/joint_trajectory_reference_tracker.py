#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import copy

class JointTrajectoryReferenceTracker:

    def __init__(self):
        # Joint trajectory point is only published while trajectory is non-empty
        self.joint_trajectory_point_pub = rospy.Publisher(
            'reference_tracker/joint_trajectory_point', 
            JointTrajectoryPoint, queue_size=1)
        # Flag that publishes while trajectory is dequeued
        self.executing_trajectory_pub = rospy.Publisher(
            'reference_tracker/executing_trajectory', Int32, queue_size=1)
        # Current reference is continuous publisher
        self.current_joint_trajectory_reference_pub = rospy.Publisher(
            'reference_tracker/current_reference',
            JointTrajectoryPoint, queue_size=1)

        # Parameters
        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(self.rate) 
        
        self.executing_trajectory_flag = False
        self.joint_trajectory = JointTrajectory()
        self.current_trajectory_point = JointTrajectoryPoint()

        # Subscriber for the full joint trajectory
        rospy.Subscriber('reference_tracker/joint_trajectory', JointTrajectory, 
            self.jointTrajectoryCallback, queue_size=1)
        # Setting current point initially or overriding it completely.
        rospy.Subscriber('reference_tracker/set_current_trajectory_point', 
            JointTrajectoryPoint, self.setCurrentPointCallback, queue_size=1)

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()
            self.current_joint_trajectory_reference_pub.publish(
                self.current_trajectory_point)

            if self.executing_trajectory_flag == True:
                self.executing_trajectory_pub.publish(1)
                # Take first point from trajectory, publish it and remove it
                # from trajectory
                self.current_trajectory_point = self.joint_trajectory.points[0]
                self.publishAll()
                self.joint_trajectory.points.pop(0)
                
                if len(self.joint_trajectory.points) == 0:
                    self.executing_trajectory_flag = False
            else:
                self.executing_trajectory_pub.publish(0)

    def jointTrajectoryCallback(self, msg):
        print("[ReferenceTracker]: Received a trajectory.")
        if len(msg.points) > 0:
            self.joint_trajectory = copy.deepcopy(msg)
            self.executing_trajectory_flag = True
        else:
            print("Currently executing a trajectory.")

    def setCurrentPointCallback(self, msg):
        self.current_trajectory_point = msg


    def publishAll(self):
        self.joint_trajectory_point_pub.publish(self.current_trajectory_point)


if __name__ == '__main__':

    rospy.init_node('joint_trajectory_reference_tracker')
    reference_tracker = JointTrajectoryReferenceTracker()
    reference_tracker.run()

