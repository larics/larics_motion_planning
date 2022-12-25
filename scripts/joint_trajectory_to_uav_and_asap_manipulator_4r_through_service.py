#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math, time
from math import sin, cos
from geometry_msgs.msg import Twist, Transform, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float64MultiArray
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
    MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint
from larics_motion_planning.srv import MultiDofTrajectory, \
    MultiDofTrajectoryRequest, MultiDofTrajectoryResponse
import copy

class JointTrajectoryToUavAndAsapManipulator4rReference:

    def __init__(self):
        # UAV publishers for trajectory
        self.uav_trajectory_point_pub = rospy.Publisher('trajectory_point_ref', 
            MultiDOFJointTrajectoryPoint, queue_size=1)
        self.executing_trajectory_pub = rospy.Publisher('executing_trajectory_through_service', 
            Int32, queue_size=1)
        self.trajectory_go_to_pub = rospy.Publisher('go_to/full_state_ref', 
            Float64MultiArray, queue_size=1)

        # Manipulator publishers
        self.manipulator_joint1_pub = rospy.Publisher(
            'joint1_position_controller/command', Float64, queue_size=1)
        self.manipulator_joint2_pub = rospy.Publisher(
            'joint2_position_controller/command', Float64, queue_size=1)
        self.manipulator_joint3_pub = rospy.Publisher(
            'joint3_position_controller/command', Float64, queue_size=1)
        self.manipulator_joint4_pub = rospy.Publisher(
            'joint4_position_controller/command', Float64, queue_size=1)

        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(self.rate) 
        self.t_start = rospy.Time.now()
        self.executing_trajectory_flag = False
        self.trajectory = JointTrajectory()
        self.current_trajectory_point = JointTrajectoryPoint()
        self.uav_current_trajectory_point = MultiDOFJointTrajectoryPoint()
        self.roll = 0
        self.pitch = 0
        self.uav_current_pose = PoseStamped()
        self.uav_current_yaw = 0.0

        #rospy.Subscriber('joint_trajectory', JointTrajectory, 
        #    self.jointTrajectoryCallback, queue_size=1)
        rospy.Subscriber('imu', Imu, self.imuCallback, queue_size=1)
        rospy.Subscriber('pose', PoseStamped, self.uavPoseCallback, queue_size=1)
        # In the service callback, we will request for the UAV to move to the
        # initial position. We have to wait until it gets to the initial position
        # and since it is handled through another node, we simply check its 
        # flag if the trajectory finished executing.
        self.executing_trajectory_in_other_node = 0
        self.executing_trajectory_in_other_node_previous = 0
        rospy.Subscriber('executing_trajectory', Int32, 
            self.executingTrajectoryInOtherNodeCallback, queue_size=1)

        self.execute_trajectory_service = rospy.Service('execute_trajectory', 
            MultiDofTrajectory, self.executeTrajectoryCallback)

    def run(self):
        
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

            if self.executing_trajectory_flag == True:
                self.executing_trajectory_pub.publish(1)
                # Take first point from trajectory, publish it and remove it
                # from trajectory
                self.current_trajectory_point = self.trajectory.points[0]
                self.uav_current_trajectory_point = jointTrajectoryPointToMultiDofJointTrajectoryPoint(
                    self.current_trajectory_point)
                self.publishAll()
                #self.trajectory_point_pub.publish(self.current_trajectory_point)
                self.trajectory.points.pop(0)
                
                if len(self.trajectory.points) == 0:
                    self.executing_trajectory_flag = False
            else:
                self.executing_trajectory_pub.publish(0)

    def executingTrajectoryInOtherNodeCallback(self, msg):
        self.executing_trajectory_in_other_node_previous = \
            self.executing_trajectory_in_other_node
        self.executing_trajectory_in_other_node = msg.data

    def executeTrajectoryCallback(self, req):
        if len(req.waypoints.points) < 0:
            print("0 points in trajectory.")
            response = MultiDofTrajectoryResponse()
            response.path_length = -1.0
            return response

        # Send the model UAV to the initial point
        initial_point = Float64MultiArray()
        initial_point.data = req.waypoints.points[0].positions
        self.trajectory_go_to_pub.publish(initial_point)
        start_time = time.time()
        initial_timer = 0.0
        temp_rate = rospy.Rate(self.rate)
        # First loop waits for 2s to see if trajectory started. If it has not
        # started that means the model UAV is already at the right spot so we
        # can move directly to executing the trajectory
        trajectory_start_flag = True
        while (not rospy.is_shutdown()) and (initial_timer < 2.0):
            if (self.executing_trajectory_in_other_node_previous == 0) and \
                (self.executing_trajectory_in_other_node == 1):
                trajectory_start_flag = False
            initial_timer = time.time() - start_time
            temp_rate.sleep()

        # If trajectory has not started, the model UAV is at the right spot.
        # Otherwise wait for the model UAV to get to the right spot.
        if (trajectory_start_flag  == False):
            print("Model UAV not at required point. Moving model to first point.")
            # Second loop waits for 2s so we are sure trajectory started
            start_time = time.time()
            initial_timer = 0.0
            trajectory_end_flag = False
            while (not rospy.is_shutdown()) and ((initial_timer < 2.0) or \
                (trajectory_end_flag == False)):
                if (self.executing_trajectory_in_other_node_previous == 1) and \
                    (self.executing_trajectory_in_other_node == 0):
                    trajectory_end_flag = True
                initial_timer = time.time() - start_time
                temp_rate.sleep()
            print("Model trajectory executed. Waiting for 5s to start with intended trajectory.")
            # At this point the trajectory should ended so just wait for 5s so
            # that the model UAV can settle down.
            start_time = time.time()
            while (not rospy.is_shutdown()) and ((time.time() - start_time) < 5.0):
                temp_rate.sleep()

        print("Starting to execute trajectory. Length: ", len(req.waypoints.points))
        # Go through all points and execute the trajectory point by point
        response = MultiDofTrajectoryResponse()
        rate = rospy.Rate(self.rate)
        for i in range(len(req.waypoints.points)):
            self.current_trajectory_point = req.waypoints.points[i]
            self.uav_current_trajectory_point = jointTrajectoryPointToMultiDofJointTrajectoryPoint(
                self.current_trajectory_point)
            self.publishAll()

            # Record the roll and pitch data and place it in response
            point = copy.deepcopy(self.current_trajectory_point)
            lst = list(point.positions)
            lst[3] = self.roll
            lst[4] = self.pitch
            point.positions = tuple(lst)
            response.trajectory.points.append(point)

            # Also record the executed positions and yaw
            point_executed = copy.deepcopy(point)
            lst = list(point_executed.positions)
            lst[0] = self.uav_current_pose.pose.position.x
            lst[1] = self.uav_current_pose.pose.position.y
            lst[2] = self.uav_current_pose.pose.position.z
            lst[3] = self.roll
            lst[4] = self.pitch
            lst[5] = self.uav_current_yaw
            point_executed.positions = tuple(lst)
            response.executed_trajectory.points.append(point_executed)

            rate.sleep()

        # Since the UAV 'lags' when executing the trajectory we want to
        # prolong the roll and pitch recording until the UAV gets to the steady
        # state.
        tstart = time.time()
        while True: #(((abs(self.roll) > 0.001) or (abs(self.pitch) > 0.001)) or ((time.time() - tstart) < 0.5)):
            point = copy.deepcopy(self.current_trajectory_point)
            lst = list(point.positions)
            lst[3] = self.roll
            lst[4] = self.pitch
            point.positions = tuple(lst)
            response.trajectory.points.append(point)

            # Also record the executed positions and yaw
            point_executed = copy.deepcopy(point)
            lst = list(point_executed.positions)
            lst[0] = self.uav_current_pose.pose.position.x
            lst[1] = self.uav_current_pose.pose.position.y
            lst[2] = self.uav_current_pose.pose.position.z
            lst[3] = self.roll
            lst[4] = self.pitch
            lst[5] = self.uav_current_yaw
            point_executed.positions = tuple(lst)
            response.executed_trajectory.points.append(point_executed)

            # This can be checked through roll and pitch angles.
            # Also minimum time is required.
            if abs(self.roll) < 0.001 and abs(self.pitch) < 0.001 and (time.time()-tstart) > 2.1:
                break

            rate.sleep()

        print("Lenghtened trajectory length: ", len(response.trajectory.points))
        print("Trajectory executed!")
        return response

    def jointTrajectoryCallback(self, msg):
        print("Received a trajectory.")
        if self.executing_trajectory_flag == False and \
            len(msg.points) > 0:
            self.trajectory = copy.deepcopy(msg)
            self.executing_trajectory_flag = True
        else:
            print("Currently executing a trajectory.")

    def imuCallback(self, msg):
        q0 = msg.orientation.w
        q1 = msg.orientation.x
        q2 = msg.orientation.y
        q3 = msg.orientation.z

        self.roll = math.atan2(2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2))
        self.pitch = math.asin(2*(q0*q2 - q3*q1))
        #print("{:10.6f}".format(self.roll), "{:10.6f}".format(self.pitch))

    def uavPoseCallback(self, msg):
        self.uav_current_pose = msg
        q0 = msg.pose.orientation.w
        q1 = msg.pose.orientation.x
        q2 = msg.pose.orientation.y
        q3 = msg.pose.orientation.z

        self.uav_current_yaw = math.atan2(2.0*(q0*q3 + q1*q2), 
            1.0-2.0*(q2*q2+q3*q3))

    def publishAll(self):
        self.manipulator_joint1_pub.publish(self.current_trajectory_point.positions[4+2])
        self.manipulator_joint2_pub.publish(self.current_trajectory_point.positions[5+2])
        self.manipulator_joint3_pub.publish(self.current_trajectory_point.positions[6+2])
        self.manipulator_joint4_pub.publish(self.current_trajectory_point.positions[7+2])
        self.uav_trajectory_point_pub.publish(self.uav_current_trajectory_point)


def jointTrajectoryPointToMultiDofJointTrajectoryPoint(joint):
    multi = MultiDOFJointTrajectoryPoint()

    transform = Transform()
    transform.translation.x = joint.positions[0]
    transform.translation.y = joint.positions[1]
    transform.translation.z = joint.positions[2]
    # Index 5 denotes yaw because we have roll and pitch as placeholders
    transform.rotation.z = math.sin(joint.positions[5]/2.0)
    transform.rotation.w = math.cos(joint.positions[5]/2.0)

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

    rospy.init_node('joint_trajectory_to_uav_and_asap_manipulator_4r_reference_through_service')
    trajectory_to_ref = JointTrajectoryToUavAndAsapManipulator4rReference()
    trajectory_to_ref.run()

