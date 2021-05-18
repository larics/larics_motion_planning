#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, time, getpass
import copy, os, psutil
import numpy as np
import math
from math import sin, cos, tan, atan2
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from larics_motion_planning.srv import GetPlantBoxInspectionPoints, \
  GetPlantBoxInspectionPointsRequest, GetPlantBoxInspectionPointsResponse, \
  VisualizeState, VisualizeStateRequest, VisualizeStateResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class VisualizeToppraRawData:

  def __init__(self, node_name):
    self.rate = rospy.get_param('~rate', 1)
    # Service for direct kinematics.
    self.visualize_state_service = rospy.ServiceProxy(
      "visualize_state", VisualizeState)

    # Publishers
    self.marker_array = MarkerArray()
    self.marker_array_pub = rospy.Publisher(node_name + 
      '/visualization_markers', MarkerArray, queue_size=1)

    self.trajectory = JointTrajectory()
    self.waypoints = JointTrajectory()
    self.visualization_changed = False

    # Subscribers
    rospy.Subscriber('toppra_raw_waypoints', JointTrajectory, 
      self.toppraWaypointsCallback, queue_size=1)
    rospy.Subscriber('toppra_raw_trajectory', JointTrajectory, 
      self.toppraTrajectoryCallback, queue_size=1)

  def run(self):
    rate = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      rate.sleep()
      if self.visualization_changed == True:
        self.visualization_changed = False
        self.setUpMarkerArray()
      self.marker_array_pub.publish(self.marker_array)
      #print(self.box_config_vector)

  def toppraWaypointsCallback(self, msg):
    self.waypoints = msg

  def toppraTrajectoryCallback(self, msg):
    if len(msg.points) > 5000:
      self.visualization_changed = True
    self.trajectory = msg

  def setUpMarkerArray(self):
    # Delete everything in marker array
    self.marker_array = MarkerArray()

    # First add waypoints
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"
    marker.id = len(self.marker_array.markers)
    marker.ns = "waypoints"
    marker.type = Marker.SPHERE_LIST
    marker.action = marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 247.0/255.0
    marker.color.g = 134.0/255.0
    marker.color.b = 12.0/255.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()
    marker.pose.orientation.w = 1.0
    for i in range(len(self.waypoints.points)):
      point = Point()
      point.x = self.waypoints.points[i].positions[0]
      point.y = self.waypoints.points[i].positions[1]
      point.z = self.waypoints.points[i].positions[2]
      marker.points.append(copy.deepcopy(point))
    self.marker_array.markers.append(copy.deepcopy(marker))

    # Next trajectory
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"
    marker.id = len(self.marker_array.markers)
    marker.ns = "trajectory"
    marker.type = Marker.LINE_LIST
    marker.action = marker.ADD
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03
    marker.color.r = 0.0/255.0
    marker.color.g = 255.0/255.0
    marker.color.b = 0.0/255.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()
    marker.pose.orientation.w = 1.0
    for i in range(0,len(self.trajectory.points)-1,20):
      point = Point()
      point.x = self.trajectory.points[i].positions[0]
      point.y = self.trajectory.points[i].positions[1]
      point.z = self.trajectory.points[i].positions[2]
      marker.points.append(copy.deepcopy(point))
      point.x = self.trajectory.points[i+1].positions[0]
      point.y = self.trajectory.points[i+1].positions[1]
      point.z = self.trajectory.points[i+1].positions[2]
      marker.points.append(copy.deepcopy(point))
    self.marker_array.markers.append(copy.deepcopy(marker))

    # End effector waypoints
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"
    marker.id = len(self.marker_array.markers)
    marker.ns = "waypoints_ee"
    marker.type = Marker.SPHERE_LIST
    marker.action = marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 247.0/255.0
    marker.color.g = 134.0/255.0
    marker.color.b = 200.0/255.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()
    marker.pose.orientation.w = 1.0
    for i in range(len(self.waypoints.points)):
      point = Point()
      req = VisualizeStateRequest()
      for j in range(len(self.waypoints.points[i].positions)):
        req.state.data.append(self.waypoints.points[i].positions[j])
      res = self.visualize_state_service(req)
      #time.sleep(1)
      point.x = res.end_effector.position.x
      point.y = res.end_effector.position.y
      point.z = res.end_effector.position.z
      marker.points.append(copy.deepcopy(point))
    self.marker_array.markers.append(copy.deepcopy(marker))

    # Next trajectory
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"
    marker.id = len(self.marker_array.markers)
    marker.ns = "trajectory_ee"
    marker.type = Marker.LINE_LIST
    marker.action = marker.ADD
    marker.scale.x = 0.015
    marker.scale.y = 0.015
    marker.scale.z = 0.015
    marker.color.r = 255.0/255.0
    marker.color.g = 0.0/255.0
    marker.color.b = 0.0/255.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()
    marker.pose.orientation.w = 1.0
    req = VisualizeStateRequest()
    for j in range(len(self.trajectory.points[0].positions)):
      req.state.data.append(self.trajectory.points[0].positions[j])
    res = self.visualize_state_service(req)
    for i in range(0,len(self.trajectory.points)-1,20):
      point = Point()
      point.x = res.end_effector.position.x
      point.y = res.end_effector.position.y
      point.z = res.end_effector.position.z
      marker.points.append(copy.deepcopy(point))
      req = VisualizeStateRequest()
      try:
        for j in range(len(self.trajectory.points[i+1].positions)):
          req.state.data.append(self.trajectory.points[i+1].positions[j])
        res = self.visualize_state_service(req)
      except:
        print("Something went wrong")
      point.x = res.end_effector.position.x
      point.y = res.end_effector.position.y
      point.z = res.end_effector.position.z
      marker.points.append(copy.deepcopy(point))
      print(i, len(self.trajectory.points))
    self.marker_array.markers.append(copy.deepcopy(marker))

    print("Markers done, but at what cost")

    self.saveToCsv()

  def saveToCsv(self):
    waypoints_file = open("/home/"+getpass.getuser() + "/waypoints.csv", "w")
    header_line = "uav_x,uav_y,uav_z,ee_x,ee_y,ee_z"
    waypoints_file.write(header_line)
    for i in range(len(self.marker_array.markers[0].points)):
      line = "\n" + str(self.marker_array.markers[0].points[i].x) + ',' +\
        str(self.marker_array.markers[0].points[i].y) + ',' +\
        str(self.marker_array.markers[0].points[i].z) + ',' +\
        str(self.marker_array.markers[2].points[i].x) + ',' +\
        str(self.marker_array.markers[2].points[i].y) + ',' +\
        str(self.marker_array.markers[2].points[i].z)
      waypoints_file.write(line)
    waypoints_file.close()


    trajectory_file = open("/home/"+getpass.getuser() + "/trajectory.csv", "w")
    header_line = "uav_x,uav_y,uav_z,ee_x,ee_y,ee_z"
    trajectory_file.write(header_line)
    for i in range(0,len(self.marker_array.markers[1].points),2):
      line = "\n" + str(self.marker_array.markers[1].points[i].x) + ',' +\
        str(self.marker_array.markers[1].points[i].y) + ',' +\
        str(self.marker_array.markers[1].points[i].z) + ',' +\
        str(self.marker_array.markers[3].points[i].x) + ',' +\
        str(self.marker_array.markers[3].points[i].y) + ',' +\
        str(self.marker_array.markers[3].points[i].z)
      trajectory_file.write(line)
    trajectory_file.close()

def JointTrajectoryToPath(trajectory):
  path = Path()
  pose = PoseStamped()
  for i in range(len(trajectory.points)):
    pose.pose.position.x = trajectory.points[i].positions[0]
    pose.pose.position.y = trajectory.points[i].positions[1]
    pose.pose.position.z = trajectory.points[i].positions[2]
    pose.pose.orientation.w = 1.0
    path.append(copy.deepcopy(pose))

  return path

if __name__ == '__main__':
  node_name = 'visualize_toppra_raw_data'
  rospy.init_node(node_name)
  visualize_raw_data = VisualizeToppraRawData(node_name)
  visualize_raw_data.run()
