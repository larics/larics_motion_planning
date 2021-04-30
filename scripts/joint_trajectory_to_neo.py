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

class JointTrajectoryToNeo:

    def __init__(self):
        # UAV publishers for trajectory
        self.uav_trajectory_point_pub = rospy.Publisher('command/trajectory', 
            MultiDOFJointTrajectory, queue_size=1)

        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(self.rate) 
        self.t_start = rospy.Time.now()
        self.executing_trajectory_flag = False
        self.trajectory = JointTrajectory()
        self.current_trajectory_point = JointTrajectoryPoint()
        self.uav_current_trajectory_point = MultiDOFJointTrajectoryPoint()

        self.airdrop_flag = rospy.get_param('~airdrop', False)
            if self.airdrop_flag == True:
                self.serial_io_pub = rospy.Publisher('serial_io', Int32, queue_size=1)
                self.magnet_on_off = 0
                self.magnet_on_off_old = 0

                self.delta = 1000
                self.delta_previous = 1000
                rospy.Subscriber('pose', PoseStamped, self.uavPoseCallback, queue_size=1)
                self.uav_current_pose = Pose()


        rospy.Subscriber('joint_trajectory', JointTrajectory, 
            self.jointTrajectoryCallback, queue_size=1)

    def run(self):
        
        rospy.spin()


    def jointTrajectoryCallback(self, msg):
        print("Received a trajectory.")
        if len(msg.points) > 0:
            self.trajectory = copy.deepcopy(msg)
            multi_dof_trajectory = MultiDOFJointTrajectory()
            for i in range(len(msg.points)):
                multi_dof_trajectory.points.append(copy.deepcopy(
                    jointTrajectoryPointToMultiDofJointTrajectoryPoint(msg.points[i])))

            self.uav_trajectory_point_pub.publish(multi_dof_trajectory)
            print("Published multi dof trajectory.")
        else:
            print("Trajectory length must be greater than zero!")

    def uavPoseCallback(self, msg):
        self.uav_current_pose = msg.pose

def jointTrajectoryPointToMultiDofJointTrajectoryPoint(joint):
    multi = MultiDOFJointTrajectoryPoint()

    transform = Transform()
    transform.translation.x = joint.positions[0]
    transform.translation.y = joint.positions[1]
    transform.translation.z = joint.positions[2]
    transform.rotation.z = math.sin(joint.positions[3]/2.0)
    transform.rotation.w = math.cos(joint.positions[3]/2.0)

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
    trajectory_to_ref = JointTrajectoryToNeo()
    trajectory_to_ref.run()

