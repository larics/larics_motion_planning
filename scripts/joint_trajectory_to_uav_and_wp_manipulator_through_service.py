#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math, time
from math import sin, cos
from geometry_msgs.msg import Twist, Transform
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
    MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint
from larics_motion_planning.srv import MultiDofTrajectory, \
    MultiDofTrajectoryRequest, MultiDofTrajectoryResponse
import copy

class JointTrajectoryToUavAndWpManipulatorReference:

    def __init__(self):
        # UAV publishers for trajectory
        self.uav_trajectory_point_pub = rospy.Publisher('trajectory_point_ref', 
            MultiDOFJointTrajectoryPoint, queue_size=1)
        self.executing_trajectory_pub = rospy.Publisher('executing_trajectory', 
            Int32, queue_size=1)

        # Manipulator publishers
        self.manipulator_joint1_pub = rospy.Publisher(
            'joint1_position_controller/command', Float64, queue_size=1)
        self.manipulator_joint2_pub = rospy.Publisher(
            'joint2_position_controller/command', Float64, queue_size=1)
        self.manipulator_joint3_pub = rospy.Publisher(
            'joint3_position_controller/command', Float64, queue_size=1)
        self.manipulator_joint4_pub = rospy.Publisher(
            'joint4_position_controller/command', Float64, queue_size=1)
        self.manipulator_joint5_pub = rospy.Publisher(
            'joint5_position_controller/command', Float64, queue_size=1)

        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(self.rate) 
        self.t_start = rospy.Time.now()
        self.executing_trajectory_flag = False
        self.trajectory = JointTrajectory()
        self.current_trajectory_point = JointTrajectoryPoint()
        self.uav_current_trajectory_point = MultiDOFJointTrajectoryPoint()
        self.roll = 0
        self.pitch = 0

        #rospy.Subscriber('joint_trajectory', JointTrajectory, 
        #    self.jointTrajectoryCallback, queue_size=1)
        rospy.Subscriber('imu', Imu, self.imuCallback, queue_size=1)

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

    def executeTrajectoryCallback(self, req):
        if len(req.waypoints.points) < 0:
            print "0 points in trajectory."
            response = MultiDofTrajectoryResponse()
            response.path_length = -1.0
            return response

        print "Starting to execute trajectory. Length: ", len(req.waypoints.points)
        # Go through all points
        response = MultiDofTrajectoryResponse()
        rate = rospy.Rate(self.rate)
        for i in range(len(req.waypoints.points)):
            self.current_trajectory_point = req.waypoints.points[i]
            self.uav_current_trajectory_point = jointTrajectoryPointToMultiDofJointTrajectoryPoint(
                self.current_trajectory_point)
            self.publishAll()

            point = copy.deepcopy(self.current_trajectory_point)
            lst = list(point.positions)
            lst[3] = self.roll
            lst[4] = self.pitch
            point.positions = tuple(lst)
            response.trajectory.points.append(point)

            rate.sleep()

        tstart = time.time()
        while True: #(((abs(self.roll) > 0.001) or (abs(self.pitch) > 0.001)) or ((time.time() - tstart) < 0.5)):
            point = copy.deepcopy(self.current_trajectory_point)
            lst = list(point.positions)
            lst[3] = self.roll
            lst[4] = self.pitch
            point.positions = tuple(lst)
            response.trajectory.points.append(point)

            if abs(self.roll) < 0.001 and abs(self.pitch) < 0.001 and (time.time()-tstart) > 2.1:
                break

            rate.sleep()

        print "Lenghtened trajectory length: ", len(response.trajectory.points)
        print "Trajectory executed!"
        return response

    def jointTrajectoryCallback(self, msg):
        print "Received a trajectory."
        if self.executing_trajectory_flag == False and \
            len(msg.points) > 0:
            self.trajectory = copy.deepcopy(msg)
            self.executing_trajectory_flag = True
        else:
            print "Currently executing a trajectory."

    def imuCallback(self, msg):
        q0 = msg.orientation.w
        q1 = msg.orientation.x
        q2 = msg.orientation.y
        q3 = msg.orientation.z

        self.roll = math.atan2(2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2))
        self.pitch = math.asin(2*(q0*q2 - q3*q1))
        #print "{:10.6f}".format(self.roll), "{:10.6f}".format(self.pitch)

    def publishAll(self):
        self.manipulator_joint1_pub.publish(self.current_trajectory_point.positions[4+2])
        self.manipulator_joint2_pub.publish(self.current_trajectory_point.positions[5+2])
        self.manipulator_joint3_pub.publish(self.current_trajectory_point.positions[6+2])
        self.manipulator_joint4_pub.publish(self.current_trajectory_point.positions[7+2])
        self.manipulator_joint5_pub.publish(self.current_trajectory_point.positions[8+2])
        self.uav_trajectory_point_pub.publish(self.uav_current_trajectory_point)

def jointTrajectoryPointToMultiDofJointTrajectoryPoint(joint):
    multi = MultiDOFJointTrajectoryPoint()

    transform = Transform()
    transform.translation.x = joint.positions[0]
    transform.translation.y = joint.positions[1]
    transform.translation.z = joint.positions[2]
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

    rospy.init_node('joint_trajectory_to_uav_and_wp_manipulator_reference_through_service')
    trajectory_to_ref = JointTrajectoryToUavAndWpManipulatorReference()
    trajectory_to_ref.run()

