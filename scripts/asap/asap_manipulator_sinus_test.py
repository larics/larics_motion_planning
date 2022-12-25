#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin
from std_msgs.msg import Float32, Bool
import copy

class AsapManipulatorSinusPub:

    def __init__(self):
        # Manipulator publishers
        self.manipulator_joint1_pub = rospy.Publisher(
            'joint1', Float32, queue_size=1)
        self.manipulator_joint2_pub = rospy.Publisher(
            'joint2', Float32, queue_size=1)
        self.manipulator_joint3_pub = rospy.Publisher(
            'joint3', Float32, queue_size=1)
        self.manipulator_joint4_pub = rospy.Publisher(
            'joint4', Float32, queue_size=1)

        self.rate = rospy.get_param('~rate', 100)
        self.joint1_value = 0.0
        self.joint2_value = 0.0
        self.joint3_value = 0.0
        self.joint4_value = 0.0

        self.joint1_init = 1.0
        self.joint2_init = 0.5
        self.joint3_init = -1.0
        self.joint4_init = 0.5

        self.execute_flag = False


        rospy.Subscriber('start_sinus_trajectory', Bool, self.startStopCallback,
            queue_size=1)

    def run(self):
        
        rate = rospy.Rate(self.rate)
        i = 0.0
        while not rospy.is_shutdown():
            rate.sleep()

            if self.execute_flag == True:
                self.joint1_value = self.joint1_init + 0.2*sin(i/50)
                self.joint2_value = self.joint2_init + 0.2*sin(i/50)
                self.joint3_value = self.joint3_init + 0.2*sin(i/50)
                self.joint4_value = self.joint4_init + 0.2*sin(i/50)
                i = i + 1.0

                self.publishAll()


    def publishAll(self):
        self.manipulator_joint1_pub.publish(self.joint1_value)
        self.manipulator_joint2_pub.publish(self.joint2_value)
        self.manipulator_joint3_pub.publish(self.joint3_value)
        self.manipulator_joint4_pub.publish(self.joint4_value)

    def startStopCallback(self, msg):
        self.execute_flag = msg.data
        

if __name__ == '__main__':

    rospy.init_node('asap_manipulator_sinus_pub')
    manipulator = AsapManipulatorSinusPub()
    manipulator.run()

