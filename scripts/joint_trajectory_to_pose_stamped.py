#!/usr/bin/env python

__author__ = 'amilas'

import rospy, math
from math import sin, cos, sqrt
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import copy

class JointTrajectoryToPoseStamped:

    def __init__(self):
        # UAV publishers for trajectory
        self.uav_pose_pub = rospy.Publisher('pose_ref', 
            PoseStamped, queue_size=1)
        self.executing_trajectory_pub = rospy.Publisher('executing_trajectory', 
            Int32, queue_size=1)
        
        self.frame_id = rospy.get_param('~frame_id', 'world')
        self.rate = rospy.get_param('~rate', 100)
        self.executing_trajectory_flag = False
        self.joint_trajectory = JointTrajectory()
        self.current_trajectory_point = JointTrajectoryPoint()
        self.uav_reference_pose = PoseStamped()

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
                self.uav_reference_pose = self.jointTrajectoryPointToPoseStamped(
                    self.current_trajectory_point)
                self.publishAll()
                #self.joint_trajectory_point_pub.publish(self.current_trajectory_point)
                self.joint_trajectory.points.pop(0)
                
                if len(self.joint_trajectory.points) == 0:
                    self.executing_trajectory_flag = False
            else:
                self.executing_trajectory_pub.publish(0)

    def jointTrajectoryCallback(self, msg):
        print("Received a trajectory.")
        if len(msg.points) > 0:
            self.joint_trajectory = copy.deepcopy(msg)
            self.executing_trajectory_flag = True
        else:
            print("Currently executing a trajectory.")

    def publishAll(self):
        self.uav_pose_pub.publish(self.uav_reference_pose)


    def jointTrajectoryPointToPoseStamped(self, joint):
        pose = PoseStamped()
        
        pose.header.frame_id = self.frame_id
        
        pose.pose.position.x = joint.positions[0]
        pose.pose.position.y = joint.positions[1]
        pose.pose.position.z = joint.positions[2]
        
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = math.sin(joint.positions[3]/2.0)
        pose.pose.orientation.w = math.cos(joint.positions[3]/2.0)

        return pose

if __name__ == '__main__':

    rospy.init_node('joint_trajectory_to_pose_stamped')
    trajectory_to_pose = JointTrajectoryToPoseStamped()
    trajectory_to_pose.run()

