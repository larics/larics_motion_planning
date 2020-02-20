#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos, sqrt
from geometry_msgs.msg import Twist, Transform, Pose, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float32
from std_srvs.srv import Empty, EmptyRequest
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
    MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint
import copy
import time

class ParabolicAirdropDequeue:

    def __init__(self):
        # UAV publishers for trajectory
        self.uav_trajectory_point_pub = rospy.Publisher('trajectory_point_ref', 
            MultiDOFJointTrajectoryPoint, queue_size=1)
        self.executing_trajectory_pub = rospy.Publisher('executing_trajectory', 
            Int32, queue_size=1)

        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(self.rate) 
        self.airdrop_flag = rospy.get_param('~airdrop', True)
        if self.airdrop_flag == True:
            self.magnet_pub = rospy.Publisher('magnet/gain', Float32, queue_size=1)
            self.dropoff_delta_pub = rospy.Publisher('dropoff_delta/position', Pose, queue_size=1)
            self.dropoff_delta_velocity_pub = rospy.Publisher('dropoff_delta/velocity', 
                Twist, queue_size=1)
            self.magnet_on_service = rospy.ServiceProxy("magnet/override_ON", Empty)
            self.magnet_off_service = rospy.ServiceProxy("magnet/override_OFF", Empty)
            self.airdrop_pose = Pose()
            self.airdrop_velocity = Twist()
            self.magnet_gain = 1.0
            self.airdrop_counter = 0
            self.uav_current_pose = Pose()
            self.uav_current_velocity = Twist()
            rospy.Subscriber('global_position/local', Odometry, self.uavOdometryCallback, queue_size=1)
            #rospy.Subscriber('pose', PoseStamped, self.uavPoseCallback, queue_size=1)
            #rospy.Subscriber('velocity_relative', TwistStamped, self.uavVelocityCallback, queue_size=1)
            self.delta = 1000
            self.delta_previous = 1000
        self.t_start = rospy.Time.now()
        self.executing_trajectory_flag = False
        self.joint_trajectory = JointTrajectory()
        self.current_trajectory_point = JointTrajectoryPoint()
        self.uav_current_trajectory_point = MultiDOFJointTrajectoryPoint()

        rospy.Subscriber('parabolic_trajectory', JointTrajectory, 
            self.jointTrajectoryCallback, queue_size=1)

    def run(self):
        ball_released_flag = False
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

            if self.executing_trajectory_flag == True:
                self.executing_trajectory_pub.publish(1)
                # Take first point from trajectory, publish it and remove it
                # from trajectory
                self.current_trajectory_point = self.joint_trajectory.points[0]
                self.uav_current_trajectory_point = jointTrajectoryPointToMultiDofJointTrajectoryPoint(
                    self.current_trajectory_point)
                if self.airdrop_flag == True:
                    airdrop_delta = Pose()
                    airdrop_delta.position.x = abs(self.airdrop_pose.position.x - 
                        self.uav_current_pose.position.x)
                    airdrop_delta.position.y = abs(self.airdrop_pose.position.y - 
                        self.uav_current_pose.position.y)
                    airdrop_delta.position.z = abs(self.airdrop_pose.position.z - 
                        self.uav_current_pose.position.z)
                    airdrop_delta.orientation.w = sqrt(airdrop_delta.position.x**2 + 
                        airdrop_delta.position.y**2 + airdrop_delta.position.z**2)
                    airdrop_delta.orientation.x = bool(self.airdrop_counter)
                    self.delta_previous = self.delta
                    self.delta = airdrop_delta.orientation.w

                    airdrop_delta_velocity = Twist()
                    airdrop_delta_velocity.linear.x = abs(self.airdrop_velocity.linear.x - 
                        self.uav_current_velocity.linear.x)
                    airdrop_delta_velocity.linear.y = abs(self.airdrop_velocity.linear.y - 
                        self.uav_current_velocity.linear.y)
                    airdrop_delta_velocity.linear.z = abs(self.airdrop_velocity.linear.z - 
                        self.uav_current_velocity.linear.z)
                    airdrop_delta_velocity.angular.z = sqrt(airdrop_delta_velocity.linear.x**2 + 
                        airdrop_delta_velocity.linear.y**2 + airdrop_delta_velocity.linear.z**2)
                    airdrop_delta_velocity.angular.x = bool(self.airdrop_counter)

                    if abs(self.joint_trajectory.points[0].positions[4]) < 0.1:
                        self.magnet_gain = 1.0
                    elif abs(self.joint_trajectory.points[0].positions[4] - 1.0) < 0.1:
                        self.airdrop_counter = self.airdrop_counter + 1
                    if self.airdrop_counter >= 100: #or (self.delta < 0.025 or 
                        #self.delta > self.delta_previous):
                        self.magnet_gain = 0.0
                        self.magnet_off_service.call(EmptyRequest())
                        if ball_released_flag == False:
                            rospy.loginfo("Ball released")
                            ball_released_flag = True

                    self.dropoff_delta_pub.publish(airdrop_delta)
                    self.dropoff_delta_velocity_pub.publish(airdrop_delta_velocity)
                    self.magnet_pub.publish(self.magnet_gain)
                self.publishAll()
                #self.joint_trajectory_point_pub.publish(self.current_trajectory_point)
                self.joint_trajectory.points.pop(0)
                
                if len(self.joint_trajectory.points) == 0:
                    self.executing_trajectory_flag = False
                    if self.airdrop_flag == True:
                    	if self.magnet_gain < 0.1:
                    		self.magnet_off_service.call(EmptyRequest())
                    		time.sleep(1)
                        self.magnet_gain = 1.0
                        self.magnet_on_service.call(EmptyRequest())
                        self.airdrop_counter = 0
                        ball_released_flag = False
                        self.magnet_pub.publish(self.magnet_gain)
            else:
                self.executing_trajectory_pub.publish(0)
# Parabolic airdrop counter values
# acc: 2.5, cnt: 12
# acc: 3.5, cnt

    def jointTrajectoryCallback(self, msg):
        print "Received a trajectory."
        if self.executing_trajectory_flag == False and \
            len(msg.points) > 0:
            self.joint_trajectory = copy.deepcopy(msg)
            self.executing_trajectory_flag = True
            if self.airdrop_flag == True:
                for i in range(len(msg.points)):
                    if msg.points[i].positions[4] == 1:
                        self.airdrop_pose.position.x = msg.points[i].positions[0]
                        self.airdrop_pose.position.y = msg.points[i].positions[1]
                        self.airdrop_pose.position.z = msg.points[i].positions[2]
                        self.airdrop_velocity.linear.x = msg.points[i].velocities[0]
                        self.airdrop_velocity.linear.y = msg.points[i].velocities[1]
                        self.airdrop_velocity.linear.z = msg.points[i].velocities[2]
                        break
        else:
            print "Currently executing a trajectory."

    def uavOdometryCallback(self,msg):
        self.uav_current_pose = msg.pose.pose
        # This is velocity in uav frame
        self.uav_current_velocity = msg.twist.twist

    def uavPoseCallback(self,msg):
        self.uav_current_pose = msg.pose

    def uavVelocityCallback(self,msg):
        self.uav_current_velocity = msg.twist

    def publishAll(self):
        self.uav_trajectory_point_pub.publish(self.uav_current_trajectory_point)


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

    rospy.init_node('parabolic_airdrop_dequeue')
    trajectory_to_ref = ParabolicAirdropDequeue()
    trajectory_to_ref.run()

