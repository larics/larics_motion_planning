#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import copy

class JointTrajectoryDequeue:

    def __init__(self):
        # UAV publishers for trajectory
        self.joint_trajectory_point_pub = rospy.Publisher('joint_trajectory_point', 
            JointTrajectoryPoint, queue_size=1)
        self.executing_trajectory_pub = rospy.Publisher('executing_trajectory', 
            Int32, queue_size=1)

        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(self.rate) 
        
        self.executing_trajectory_flag = False
        self.joint_trajectory = JointTrajectory()
        self.current_trajectory_point = JointTrajectoryPoint()

        rospy.Subscriber('joint_trajectory', JointTrajectory, 
            self.jointTrajectoryCallback, queue_size=1)

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

            if self.executing_trajectory_flag == True:
                self.executing_trajectory_pub.publish(1)
                # Take first point from trajectory, publish it and remove it
                # from trajectory
                self.current_trajectory_point = self.joint_trajectory.points[0]
                self.publishAll()
                self.joint_trajectory.points.pop(0)
                
                if len(self.joint_trajectory.points) == 0:
                    self.executing_trajectory_flag = False
            else:
                self.executing_trajectory_pub.publish(0)

    def jointTrajectoryCallback(self, msg):
        print "Received a trajectory."
        if len(msg.points) > 0:
            self.joint_trajectory = copy.deepcopy(msg)
            self.executing_trajectory_flag = True
        else:
            print "Currently executing a trajectory."


    def publishAll(self):
        self.joint_trajectory_point_pub.publish(self.current_trajectory_point)


if __name__ == '__main__':

    rospy.init_node('joint_trajectory_dequeue')
    trajectory_to_ref = JointTrajectoryDequeue()
    trajectory_to_ref.run()

