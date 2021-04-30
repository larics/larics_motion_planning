#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos
from geometry_msgs.msg import Twist, Transform
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float32
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
    MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint
import copy

class JointTrajectoryToUavAndWpManipulatorReference:

    def __init__(self):
        # UAV publishers for trajectory
        self.uav_trajectory_point_pub = rospy.Publisher('/euroc3/command/trajectory', 
            MultiDOFJointTrajectory, queue_size=1)
        self.executing_trajectory_pub = rospy.Publisher('executing_trajectory', 
            Int32, queue_size=1)

        # Manipulator publishers
        self.manipulator_joint1_pub = rospy.Publisher(
            '/euroc3/wp_manipulator_3r/joint1', Float32, queue_size=1)
        self.manipulator_joint2_pub = rospy.Publisher(
            '/euroc3/wp_manipulator_3r/joint2', Float32, queue_size=1)
        self.manipulator_joint3_pub = rospy.Publisher(
            '/euroc3/wp_manipulator_3r/joint3', Float32, queue_size=1)

        # Alternatively, publish directly as joint trajectory
        self.manipulator_trajectory_pub = rospy.Publisher(
            '/euroc3/wp_manipulator_3r/joint_trajectory', JointTrajectory, queue_size=1)

        rospy.Subscriber('/uav/joint_trajectory', JointTrajectory, 
            self.jointTrajectoryCallback, queue_size=1)

        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(self.rate) 
        self.t_start = rospy.Time.now()
        self.executing_trajectory_flag = False
        self.trajectory = JointTrajectory()
        self.current_trajectory_point = JointTrajectoryPoint()
        self.uav_current_trajectory_point = MultiDOFJointTrajectoryPoint()

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


    def jointTrajectoryCallback(self, msg):
        print("Received a trajectory.")
        if self.executing_trajectory_flag == False and \
            len(msg.points) > 0:
            self.trajectory = copy.deepcopy(msg)
            self.executing_trajectory_flag = True
        else:
            print("Currently executing a trajectory.")

    def publishAll(self):
        self.manipulator_joint1_pub.publish(self.current_trajectory_point.positions[4+2])
        self.manipulator_joint2_pub.publish(self.current_trajectory_point.positions[5+2])
        self.manipulator_joint3_pub.publish(self.current_trajectory_point.positions[6+2])
        temp_trajectory = MultiDOFJointTrajectory()
        temp_trajectory.points.append(self.uav_current_trajectory_point)
        self.uav_trajectory_point_pub.publish(temp_trajectory)

        # Manipulator joint trajectory message
        q1 = self.current_trajectory_point.positions[4+2]
        q2 = self.current_trajectory_point.positions[5+2]
        q3 = self.current_trajectory_point.positions[6+2]
        joint_trajectory = JointTrajectory()
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = [q1, q2, -q3]
        joint_trajectory_point.velocities = [0, 0, 0]
        joint_trajectory_point.accelerations = [0, 0, 0]
        joint_trajectory_point.effort = [0, 0, 0]
        joint_trajectory_point.time_from_start = rospy.Duration(0.001)

        joint_trajectory.points.append(joint_trajectory_point)
        joint_trajectory.header.frame_id = "world"
        joint_trajectory.joint_names.append("joint1")
        joint_trajectory.joint_names.append("joint2")
        joint_trajectory.joint_names.append("joint3")
        joint_trajectory.header.stamp = rospy.Time.now()
        self.manipulator_trajectory_pub.publish(joint_trajectory)


def jointTrajectoryPointToMultiDofJointTrajectoryPoint(joint):
    multi = MultiDOFJointTrajectoryPoint()

    transform = Transform()
    transform.translation.x = joint.positions[0]
    transform.translation.y = joint.positions[1]
    transform.translation.z = joint.positions[2]
    transform.rotation.z = math.sin(joint.positions[3+2]/2.0)
    transform.rotation.w = math.cos(joint.positions[3+2]/2.0)

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

    rospy.init_node('joint_trajectory_to_uav_and_wp_manipulator_reference')
    trajectory_to_ref = JointTrajectoryToUavAndWpManipulatorReference()
    trajectory_to_ref.run()

