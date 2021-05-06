#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, \
    SetModelStateResponse
from geometry_msgs.msg import Quaternion
import copy

class GazeboTrajectoryTeleport:

    def __init__(self):
        # Parameters
        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(self.rate)
        self.model_name = rospy.get_param('~model_name', "mars_rover")
        
        self.current_trajectory_point = JointTrajectoryPoint()
        self.current_trajectory_point.positions = [0, 0, 0]
        self.current_trajectory_point.velocities = [0, 0, 0]
        self.model_state_request = SetModelStateRequest()
        self.model_state_request.model_state.model_name = self.model_name

        # Services
        self.set_model_state_service = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState)

        # Joint trajectory point subscriber
        rospy.Subscriber('aerial_manipulator_control/trajectory_ref_input', 
            JointTrajectoryPoint, self.jointTrajectoryPointCallback, 
            queue_size=1)

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()
            if len(self.current_trajectory_point.positions) >= 6:
                # Assuming roll pitch and yaw are provided
                roll = self.current_trajectory_point.positions[3]
                pitch = self.current_trajectory_point.positions[4]
                yaw = self.current_trajectory_point.positions[5]
                self.model_state_request.model_state.pose.orientation = \
                    eulerToQuaternion(yaw, pitch, roll)
                self.model_state_request.model_state.pose.position.x = \
                    self.current_trajectory_point.positions[0]
                self.model_state_request.model_state.pose.position.y = \
                    self.current_trajectory_point.positions[1]
                self.model_state_request.model_state.pose.position.z = \
                    self.current_trajectory_point.positions[2]
            elif len(self.current_trajectory_point.positions) >= 4:
                # Assuming only yaw is provided
                roll = 0
                pitch = 0
                yaw = self.current_trajectory_point.positions[3]
                self.model_state_request.model_state.pose.orientation = \
                    eulerToQuaternion(yaw, pitch, roll)
                self.model_state_request.model_state.pose.position.x = \
                    self.current_trajectory_point.positions[0]
                self.model_state_request.model_state.pose.position.y = \
                    self.current_trajectory_point.positions[1]
                self.model_state_request.model_state.pose.position.z = \
                    self.current_trajectory_point.positions[2]
            elif len(self.current_trajectory_point.positions) >= 3:
                # Assuming only position is provided
                roll = 0
                pitch = 0
                yaw = 0
                self.model_state_request.model_state.pose.orientation = \
                    eulerToQuaternion(yaw, pitch, roll)
                self.model_state_request.model_state.pose.position.x = \
                    self.current_trajectory_point.positions[0]
                self.model_state_request.model_state.pose.position.y = \
                    self.current_trajectory_point.positions[1]
                self.model_state_request.model_state.pose.position.z = \
                    self.current_trajectory_point.positions[2]
            else:
                print("Not enough data provided.")

            self.set_model_state_service.call(self.model_state_request)



    def jointTrajectoryPointCallback(self, msg):
        self.current_trajectory_point = msg


def eulerToQuaternion(yaw, pitch, roll):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q

if __name__ == '__main__':

    rospy.init_node('gazebo_trajectory_teleport')
    gazebo_teleport = GazeboTrajectoryTeleport()
    gazebo_teleport.run()

