#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
from std_msgs.msg import Float64

class JointPositionControllerHandler:

    def __init__(self, topic):
        self.joint_state = 0.0
        self.pub = rospy.Publisher(topic, Float64, queue_size=1)

    def publish(self, value):
        self.pub.publish(Float64(value))

    def jointStateCallback(self, msg):
        self.joint_state = msg.process_value