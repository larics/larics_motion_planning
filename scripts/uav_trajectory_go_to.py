#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float32, Float64, Float64MultiArray
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

        # UAV type defines feedback source.
        self.uav_type = rospy.get_param('~uav_type', "kopterworx")

        # Gazebo arducopter simulation
        if self.uav_type == "arducopter_sim":
            rospy.Subscriber('pose', PoseStamped, self.uavPoseCallback, 
                queue_size=1)
        #rospy.Subscriber('msf_core/pose', PoseWithCovarianceStamped, 
        #    self.msfCorePoseCallback, queue_size=1)
        # Subscriber for NEO reference
        elif self.uav_type == "neo":
            rospy.Subscriber('command/current_reference', 
                MultiDOFJointTrajectory, self.currentReferenceCallback, 
                queue_size=1)
        elif self.uav_type == "kopterworx":
            rospy.Subscriber('carrot/trajectory', 
                MultiDOFJointTrajectoryPoint, self.carrotReferenceCallback, 
                queue_size=1)
        elif self.uav_type == "arducopter_manipulator_sim":
            print("uav_trajectory_go_to starting in: ", self.uav_type)
            # Manipulator degrees of freedom will affect total dof
            self.manipulator_dof = rospy.get_param("~manipulator_dof", int(5))
            # Also, roll and pitch may or may not be used in trajectory
            self.use_roll_pitch = rospy.get_param("~use_roll_pitch", True)

            # Initialize subscribers
            rospy.Subscriber('pose', PoseStamped, self.uavPoseCallback, 
                queue_size=1)
            self.manipulator_joint_states = []
            for i in range(self.manipulator_dof):
                joint = JointPositionControllerSubscriber()
                self.manipulator_joint_states.append(copy.deepcopy(joint))
                topic = "joint" + str(i+1) + "_position_controller/command" 
                rospy.Subscriber(topic, Float64, 
                    self.manipulator_joint_states[i].jointReferenceCallback, queue_size=1)

            # Initialize subscriber for full state reference
            rospy.Subscriber('go_to/full_state_ref', Float64MultiArray, 
                self.fullStateReferenceCallback, queue_size=1)
        else:
            print("Unknown uav_type parameter. Exiting uav_trajectory_go_to.")
            exit(0)

        # New reference subscriber
        rospy.Subscriber('go_to/reference', Pose, 
            self.uavReferenceCallback, queue_size=1)

    def run(self):
        rospy.spin()

    def uavPoseCallback(self, msg):
        self.uav_current_pose = msg.pose
        self.first_pose_received = True

    def msfCorePoseCallback(self, msg):
        self.uav_current_pose = msg.pose.pose
        self.first_pose_received = True

    def currentReferenceCallback(self, msg):
        self.uav_current_pose.position.x = msg.points[0].transforms[0].translation.x
        self.uav_current_pose.position.y = msg.points[0].transforms[0].translation.y
        self.uav_current_pose.position.z = msg.points[0].transforms[0].translation.z
        self.uav_current_pose.orientation.x = msg.points[0].transforms[0].rotation.x
        self.uav_current_pose.orientation.y = msg.points[0].transforms[0].rotation.y
        self.uav_current_pose.orientation.z = msg.points[0].transforms[0].rotation.z
        self.uav_current_pose.orientation.w = msg.points[0].transforms[0].rotation.w

        #print(self.uav_current_pose)

    def carrotReferenceCallback(self, msg):
        self.uav_current_pose.position.x = msg.transforms[0].translation.x
        self.uav_current_pose.position.y = msg.transforms[0].translation.y
        self.uav_current_pose.position.z = msg.transforms[0].translation.z
        self.uav_current_pose.orientation.x = msg.transforms[0].rotation.x
        self.uav_current_pose.orientation.y = msg.transforms[0].rotation.y
        self.uav_current_pose.orientation.z = msg.transforms[0].rotation.z
        self.uav_current_pose.orientation.w = msg.transforms[0].rotation.w


    def uavReferenceCallback(self, msg):

        if self.uav_type == "arducopter_manipulator_sim":
            self.handleUavAndManipulatorReference(self.constructReferenceFromPose(msg))
        else:
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
            waypoint.velocities = [2.5, 2.5, 0.5, 1, 100]
            waypoint.accelerations = [0.75, 0.75, 0.5, 0.5, 100]
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

    def constructReferenceFromPose(self, pose_input):
        q0 = pose_input.orientation.w
        q1 = pose_input.orientation.x
        q2 = pose_input.orientation.y
        q3 = pose_input.orientation.z
        roll = 0.0
        pitch = 0.0
        yaw = math.atan2(2.0*(q0*q3 + q1*q2), 
            1.0-2.0*(q3*q3 + q2*q2))

        if self.use_roll_pitch == True:
            output = [pose_input.position.x, pose_input.position.y, \
                pose_input.position.z, roll, pitch, yaw]
        else:
            output = [pose_input.position.x, pose_input.position.y, \
                pose_input.position.z, yaw]

        manipulator = [0]*self.manipulator_dof

        output = output + manipulator

        return output

    def fullStateReferenceCallback(self, msg):
        self.handleUavAndManipulatorReference(msg.data)

    def handleUavAndManipulatorReference(self, ref):
        # Get current yaw of the uav
        q0 = self.uav_current_pose.orientation.w
        q1 = self.uav_current_pose.orientation.x
        q2 = self.uav_current_pose.orientation.y
        q3 = self.uav_current_pose.orientation.z
        roll = 0.0
        pitch = 0.0
        yaw = math.atan2(2.0*(q0*q3 + q1*q2), 
            1.0-2.0*(q3*q3 + q2*q2))

        manipulator_joint_values = []
        for i in range(self.manipulator_dof):
            manipulator_joint_values.append(
                self.manipulator_joint_states[i].joint_state)
        manipulator_velocity_constraints = [0.5]*self.manipulator_dof
        manipulator_acceleration_constraints = [0.25]*self.manipulator_dof

        # Create service request
        request = GenerateTrajectoryRequest()
        waypoint = JointTrajectoryPoint()

        if self.use_roll_pitch == True:
            pos = [self.uav_current_pose.position.x, \
                self.uav_current_pose.position.y, \
                self.uav_current_pose.position.z, roll, pitch, yaw] + \
                manipulator_joint_values
            vel = [1, 1, 0.5, 100, 100, 1] + manipulator_velocity_constraints
            acc = [0.5, 0.5, 0.5, 100, 100, 0.5] + manipulator_acceleration_constraints
        else:
            pos = [self.uav_current_pose.position.x, \
                self.uav_current_pose.position.y, \
                self.uav_current_pose.position.z, yaw] + \
                manipulator_joint_values
            vel = [1, 1, 0.5, 1] + manipulator_velocity_constraints
            acc = [0.5, 0.5, 0.5, 0.5] + manipulator_acceleration_constraints

        # Add first waypoint as current uav pose
        waypoint.positions = pos
        waypoint.velocities = vel
        waypoint.accelerations = acc
        request.waypoints.points.append(copy.deepcopy(waypoint))

        # Final waypoint
        waypoint.positions = ref
        request.waypoints.points.append(copy.deepcopy(waypoint))

        # Check if waypoints are too close
        d = 0.0
        for i in range(len(request.waypoints.points[0].positions)):
            d = d + abs(request.waypoints.points[0].positions[i] - 
                request.waypoints.points[1].positions[i])

        if d < 0.05:
            print("Trajectory go to: points too close, not planning!", d)
        else:
            # Call trajectory planning service
            request.sampling_frequency = 100.0
            response = self.toppra_trajectory_service(request)

            # Publish planned trajectory
            self.joint_trajectory_pub.publish(response.trajectory)

class JointPositionControllerSubscriber:

    def __init__(self):
        self.joint_state = 0.0

    def jointReferenceCallback(self, msg):
        self.joint_state = msg.data
        #print("Joint data is: ", self.joint_state)

if __name__ == '__main__':

    rospy.init_node('uav_trajectory_go_to')
    go_to = UavGoTo()
    go_to.run()

