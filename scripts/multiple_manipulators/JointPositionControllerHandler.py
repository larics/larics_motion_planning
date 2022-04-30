#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState


class JointPositionControllerHandler:

    def __init__(self, pub_topic, sub_topic):
        self.pub = rospy.Publisher(pub_topic, Float64, queue_size=1)

        # Subscriber for state
        self.joint_state = Float64()
        if sub_topic != "none":
            self.sub = rospy.Subscriber(sub_topic, JointControllerState, self.jointStateCallback,
                queue_size=1)

    def publish(self, value):
        self.pub.publish(Float64(value))

    def jointStateCallback(self, msg):
        self.joint_state.data = msg.process_value

    def getJointState(self):
        return self.joint_state