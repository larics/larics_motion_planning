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

        self.marker_array_pub = rospy.Publisher(
            'visualization/multiple_trajectories', MarkerArray, queue_size=1)

        rospy.Subscriber('joint_trajectory', JointTrajectory, 
            self.jointTrajectoryCallback, queue_size=1)
        # Subscriber to clear marker array

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

    def jointTrajectoryCallback(self, msg):
        print "Received a trajectory."
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

        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "world"

        #marker.pose.position.x = 0.55
        #marker.pose.position.y = 1.20
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.04
        #marker.scale.y = 0.1
        #marker.scale.z = 0.1

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
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



if __name__ == '__main__':

    rospy.init_node('joint_trajectory_to_marker_array')
    joint_trajectory_to_marker_array = JointTrajectoryToMarkerArray()
    joint_trajectory_to_marker_array.run()
