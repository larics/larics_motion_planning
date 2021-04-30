#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math, rospkg
from math import sin, cos
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, \
    Vector3, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float32
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
    MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint
from larics_motion_planning.srv import ParabolicAirdropTrajectory, \
    ParabolicAirdropTrajectoryRequest, ParabolicAirdropTrajectoryResponse
import copy
import numpy as np

class PlanAirdropFromCurrentReference:

    def __init__(self):
        self.airdrop_request = ParabolicAirdropTrajectoryRequest()
        self.parabolic_airdrop_service = rospy.ServiceProxy(
            "parabolic_airdrop_trajectory", ParabolicAirdropTrajectory)
        self.joint_trajectory_pub = rospy.Publisher('joint_trajectory', 
            JointTrajectory, queue_size=1)

        self.rate = rospy.get_param('~rate', 10)
        self.filename = rospy.get_param('~repeatability_config_file', 
            str('red_repeatability_configs.csv'))
        rospack = rospkg.RosPack()
        self.filename = rospack.get_path('larics_motion_planning') + \
            '/config/airdrop_configs/' + self.filename
        self.airdrop_configs = np.loadtxt(open(self.filename, "rb"), delimiter=",")

        self.first_reference_received = False
        self.plan_to_target = False
        self.uav_current_reference = Pose()
        self.target = Pose()
        rospy.Subscriber('carrot/trajectory', 
            MultiDOFJointTrajectoryPoint, self.currentReferenceCallback, 
            queue_size=1)
        rospy.Subscriber('parabolic_airdrop/plan_to_target', Vector3, 
            self.targetCallback, queue_size=1)
        rospy.Subscriber('parabolic_airdrop/id_plan_to_target', Quaternion, 
            self.idTargetCallback, queue_size=1)

    def run(self):
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
        self.airdrop_request.publish_path = True
        self.airdrop_request.publish_trajectory = True
        self.airdrop_request.plan_path = True
        self.airdrop_request.plan_trajectory = True
        self.airdrop_request.use_custom_parabola_params = False
        self.airdrop_request.use_custom_psi_params = True
        self.airdrop_request.custom_psi_params = [-14, 2, -14]

        self.target.position = msg
        self.plan_to_target = True

    def idTargetCallback(self, msg):
        # Read parabola params from file
        idd = int(msg.w)
        if idd >= self.airdrop_configs.shape[0]:
            self.plan_to_target = False
            print("Cannot plan to target. Index out of scope.")
        else:
            v0 = self.airdrop_configs[idd][0]
            dx = self.airdrop_configs[idd][1]
            alpha = self.airdrop_configs[idd][2]
            dz = self.airdrop_configs[idd][3]
            psi = self.airdrop_configs[idd][4]
            self.airdrop_request.publish_path = False
            self.airdrop_request.publish_trajectory = True
            self.airdrop_request.plan_path = False
            self.airdrop_request.plan_trajectory = True
            self.airdrop_request.use_custom_parabola_params = True
            self.airdrop_request.custom_parabola_params = [v0, dz, alpha, dx, psi]
            self.airdrop_request.use_custom_psi_params = False
            print(self.airdrop_request)

            self.target.position.x = msg.x
            self.target.position.y = msg.y
            self.target.position.z = msg.z
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

