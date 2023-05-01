#!/usr/bin/env python

import rospy, math
from math import sin, cos
from geometry_msgs.msg import Twist, Transform
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float32
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
    MultiDOFJointTrajectory, JointTrajectory

class JointTrajectoryToMultiDofTrajectory:

    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'world')
        
        # UAV publishers for trajectory
        self.uav_trajectory_pub = rospy.Publisher('trajectory_ref', 
            MultiDOFJointTrajectory, queue_size=1)
        rospy.Subscriber('joint_trajectory', JointTrajectory, 
            self.jointTrajectoryCallback, queue_size=1)

    def jointTrajectoryCallback(self, msg):
        # Create and publish multi dof trajectory       
        uav_current_trajectory = self.JointTrajectoryToMultiDofTrajectory(msg) 
        self.uav_trajectory_pub.publish(uav_current_trajectory)

    def JointTrajectoryToMultiDofTrajectory(self, joint_trajectory):
        multi_dof_trajectory = MultiDOFJointTrajectory()
        multi_dof_trajectory.header.frame_id = self.frame_id
        multi_dof_trajectory.header.stamp = rospy.Time.now()
        multi_dof_trajectory.joint_names = []

        for i in range(0, len(joint_trajectory.points)):
            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_transform.translation.x = joint_trajectory.points[i].positions[0]
            temp_transform.translation.y = joint_trajectory.points[i].positions[1]
            temp_transform.translation.z = joint_trajectory.points[i].positions[2]
            temp_transform.rotation.z = math.sin(joint_trajectory.points[i].positions[3]/2.0)
            temp_transform.rotation.w = math.cos(joint_trajectory.points[i].positions[3]/2.0)

            temp_vel = Twist()
            temp_vel.linear.x = joint_trajectory.points[i].velocities[0]
            temp_vel.linear.y = joint_trajectory.points[i].velocities[1]
            temp_vel.linear.z = joint_trajectory.points[i].velocities[2]

            temp_acc = Twist()
            temp_acc.linear.x = joint_trajectory.points[i].accelerations[0]
            temp_acc.linear.y = joint_trajectory.points[i].accelerations[1]
            temp_acc.linear.z = joint_trajectory.points[i].accelerations[2]

            temp_point.transforms.append(temp_transform)
            temp_point.velocities.append(temp_vel)
            temp_point.accelerations.append(temp_acc)
            temp_point.time_from_start = joint_trajectory.points[i].time_from_start

            multi_dof_trajectory.points.append(temp_point)

        return multi_dof_trajectory

if __name__ == '__main__':

    rospy.init_node('joint_trajectory_to_multi_dof_trajectory')
    trajectory_to_ref = JointTrajectoryToMultiDofTrajectory()
    rospy.spin()
