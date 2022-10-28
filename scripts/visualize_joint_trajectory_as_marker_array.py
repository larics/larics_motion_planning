#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos, sqrt
from geometry_msgs.msg import Twist, Transform, Pose, PoseStamped, TwistStamped, \
    PoseWithCovarianceStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float32
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
    MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
import copy

class JointTrajectoryToMarkerArray:

    def __init__(self):
        
        self.joint_trajectory = JointTrajectory()

        self.rate = rospy.get_param('rate', 10)
        self.marker_array = MarkerArray()
        self.id = 1

        # For multiple trajectories use array of predefined colors
        self.color_hex_array = ["#00876c", "#40996f", "#67aa72", "#8bbb77", 
            "#b0cb7e", "#d5da88", "#fbe895", "#f8ce7c", "#f5b468", "#f0985a", 
            "#ea7b52", "#e15e4f", "#d43d51"]
        # This set of colors was used for six model planned trajectories
        # in the thesis.
        #self.color_hex_array = ["bf7000", "bf7000", "c9b300", "c9b300", "860080", "860080"]
        self.current_color_id = 0

        self.marker_array_pub = rospy.Publisher(
            'visualization/multiple_trajectories', MarkerArray, queue_size=1)

        rospy.Subscriber('joint_trajectory', JointTrajectory, 
            self.jointTrajectoryCallback, queue_size=1)
        # Subscriber to clear marker array

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()
            self.marker_array_pub.publish(self.marker_array)

    def jointTrajectoryCallback(self, msg):
        print("Received a trajectory.")
        if len(msg.points) > 0:
            marker = self.jointTrajectoryToMarker(msg, self.id)
            self.id = self.id + 1
            self.marker_array.markers.append(marker)
            self.marker_array_pub.publish(self.marker_array)

    def jointTrajectoryToMarker(self, trajectory, idd):
        marker = Marker()

        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.id = idd
        marker.ns = str(idd)

        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "world"

        #marker.pose.position.x = 0.55
        #marker.pose.position.y = 1.20
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.04*5.0
        #marker.scale.y = 0.1
        #marker.scale.z = 0.1

        # Choose color
        color_id = self.current_color_id % len(self.color_hex_array)
        self.current_color_id = self.current_color_id + 1
        rgb = hex_to_rgb(self.color_hex_array[color_id])

        marker.color.r = rgb[0]/255.0
        marker.color.g = rgb[1]/255.0
        marker.color.b = rgb[2]/255.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration()

        for i in range(len(trajectory.points)-1):
            point = Point()
            point.x = trajectory.points[i].positions[0]
            point.y = trajectory.points[i].positions[1]
            point.z = trajectory.points[i].positions[2]
            marker.points.append(copy.deepcopy(point))
            point.x = trajectory.points[i+1].positions[0]
            point.y = trajectory.points[i+1].positions[1]
            point.z = trajectory.points[i+1].positions[2]
            marker.points.append(copy.deepcopy(point))

        return marker


def hex_to_rgb(value):
    value = value.lstrip('#')
    lv = len(value)
    return tuple(int(value[i:i + lv // 3], 16) for i in range(0, lv, lv // 3))

if __name__ == '__main__':

    rospy.init_node('joint_trajectory_to_marker_array')
    joint_trajectory_to_marker_array = JointTrajectoryToMarkerArray()
    joint_trajectory_to_marker_array.run()
