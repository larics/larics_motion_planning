#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float32
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
    MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint
from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryRequest, \
    GenerateTrajectoryResponse
import copy

class UavGoTo:

    def __init__(self):
        self.toppra_trajectory_service = rospy.ServiceProxy(
            "generate_toppra_trajectory", GenerateTrajectory)
        self.joint_trajectory_pub = rospy.Publisher('joint_trajectory', 
            JointTrajectory, queue_size=1)

        self.first_pose_received = False
        self.uav_current_pose = Pose()
        self.uav_reference_pose = Pose()
        rospy.Subscriber('pose', PoseStamped, self.uavPoseCallback, 
            queue_size=1)
        rospy.Subscriber('go_to/reference', Pose, 
            self.uavReferenceCallback, queue_size=1)

    def run(self):
        rospy.spin()

    def uavPoseCallback(self, msg):
        self.uav_current_pose = msg.pose
        self.first_pose_received = True

    def uavReferenceCallback(self, msg):
        # Get current yaw of the uav
        q0 = self.uav_current_pose.orientation.w
        q1 = self.uav_current_pose.orientation.x
        q2 = self.uav_current_pose.orientation.y
        q3 = self.uav_current_pose.orientation.z
        current_yaw = math.atan2(2.0*(q0*q3 + q1*q2), 
            1.0-2.0*(q3*q3 + q2*q2))

        # Calculate desired yaw of the uav
        q0 = msg.orientation.w
        q1 = msg.orientation.x
        q2 = msg.orientation.y
        q3 = msg.orientation.z
        reference_yaw = math.atan2(2.0*(q0*q3 + q1*q2), 
            1.0-2.0*(q3*q3 + q2*q2))

        # Create a service request which will be filled with waypoints
        request = GenerateTrajectoryRequest()

        waypoint = JointTrajectoryPoint()

        # Add first waypoint as current uav pose
        waypoint.positions = [self.uav_current_pose.position.x, \
            self.uav_current_pose.position.y, \
            self.uav_current_pose.position.z, current_yaw, 0]
        waypoint.velocities = [2, 2, 2, 1, 100]
        waypoint.accelerations = [2, 2, 2, 1, 100]
        request.waypoints.points.append(copy.deepcopy(waypoint))
        # Add second waypoint as current uav pose
        waypoint.positions = [msg.position.x, \
            msg.position.y, msg.position.z, \
            reference_yaw, 0]
        request.waypoints.points.append(copy.deepcopy(waypoint))

        # Call trajectory planning service
        request.waypoints.joint_names = ["x", "y", "z", "yaw", "airdrop"]
        request.sampling_frequency = 100.0
        response = self.toppra_trajectory_service(request)

        # Publish planned trajectory
        self.joint_trajectory_pub.publish(response.trajectory)

if __name__ == '__main__':

    rospy.init_node('uav_trajectory_go_to')
    go_to = UavGoTo()
    go_to.run()
