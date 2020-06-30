#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float32
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
    MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint
from larics_motion_planning.srv import ParabolicAirdropTrajectory, \
    ParabolicAirdropTrajectoryRequest, ParabolicAirdropTrajectoryResponse
import copy

class PlanAirdropFromCurrentReference:

    def __init__(self):
        self.airdrop_request = ParabolicAirdropTrajectoryRequest()
        self.parabolic_airdrop_service = rospy.ServiceProxy(
            "parabolic_airdrop_trajectory", ParabolicAirdropTrajectory)
        self.joint_trajectory_pub = rospy.Publisher('joint_trajectory', 
            JointTrajectory, queue_size=1)

        self.rate = rospy.get_param('~rate', 10)

        self.first_reference_received = False
        self.plan_to_target = False
        self.uav_current_reference = Pose()
        self.target = Pose()
        rospy.Subscriber('carrot/trajectory', 
            MultiDOFJointTrajectoryPoint, self.currentReferenceCallback, 
            queue_size=1)
        rospy.Subscriber('parabolic_airdrop/plan_to_target', Vector3, 
            self.targetCallback, queue_size=1)

    def run(self):
        self.airdrop_request.publish_path = True
        self.airdrop_request.publish_trajectory = True
        self.airdrop_request.plan_path = True
        self.airdrop_request.plan_trajectory = True
        self.airdrop_request.use_custom_parabola_params = False
        self.airdrop_request.use_custom_psi_params = True
        self.airdrop_request.custom_psi_params = [-14, 2, -14]


        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

            if self.plan_to_target == True and self.first_reference_received == True:
                self.plan_to_target = False

                print("Planning airdrop to target.")
                self.airdrop_request.uav_pose = self.uav_current_reference
                self.airdrop_request.target_pose = self.target

                response = self.parabolic_airdrop_service(self.airdrop_request)

    def targetCallback(self, msg):
        self.target.position = msg
        self.plan_to_target = True

    def currentReferenceCallback(self, msg):
        self.uav_current_reference.position.x = msg.transforms[0].translation.x
        self.uav_current_reference.position.y = msg.transforms[0].translation.y
        self.uav_current_reference.position.z = msg.transforms[0].translation.z
        self.uav_current_reference.orientation.x = msg.transforms[0].rotation.x
        self.uav_current_reference.orientation.y = msg.transforms[0].rotation.y
        self.uav_current_reference.orientation.z = msg.transforms[0].rotation.z
        self.uav_current_reference.orientation.w = msg.transforms[0].rotation.w
        self.first_reference_received = True

if __name__ == '__main__':

    rospy.init_node('plan_airdrop_from_current_reference')
    airdrop = PlanAirdropFromCurrentReference()
    airdrop.run()

