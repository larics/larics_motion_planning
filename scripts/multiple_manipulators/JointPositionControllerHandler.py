#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
from std_msgs.msg import Float64, Float32
from control_msgs.msg import JointControllerState


class JointPositionControllerHandler:

    def __init__(self, pub_topic, sub_topic, float_type=64):

        if float_type != 32:
            self.pub = rospy.Publisher(pub_topic, Float64, queue_size=1)
            self.joint_state = Float64()
            self.joint_reference = Float64()
        else:
            self.pub = rospy.Publisher(pub_topic, Float32, queue_size=1)
            self.joint_state = Float32()
            self.joint_reference = Float32()

        # Subscriber for state
        if sub_topic != "none":
            self.sub = rospy.Subscriber(sub_topic, JointControllerState, self.jointStateCallback,
                queue_size=1)

    def publish(self, value):
        self.joint_reference.data = value
        self.pub.publish(self.joint_reference)

    def jointStateCallback(self, msg):
        self.joint_state.data = msg.process_value

    def getJointState(self):
        return self.joint_state