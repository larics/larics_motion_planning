#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
import copy, os, psutil
import numpy as np
from math import sin, cos, tan, atan2
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped
from larics_motion_planning.srv import GetPlantBoxInspectionPoints, \
  GetPlantBoxInspectionPointsRequest, GetPlantBoxInspectionPointsResponse

class BoxInspectionPoints:

  def __init__(self, node_name):
    # Get node parameters
    self.rate = rospy.get_param('~rate', 1)
    # Params for box dimensions
    self.lx = rospy.get_param('~box_length', 0.9)
    self.ly = rospy.get_param('~box_width', 0.63)
    self.lz = rospy.get_param('~box_height', 0.2)
    # Get r1 and r2 distances from box during inspection
    self.r1 = rospy.get_param('~r1', 1.0)
    self.r2 = rospy.get_param('~r2', 0.5)

    # This vector contains x,y,z,yaw 
    self.box_config_vector = [0]*4
    # List of inspection points
    self.T_points = []

    # Publishers
    self.marker_array = MarkerArray()
    self.marker_array_pub = rospy.Publisher(node_name + 
      '/visualization_markers', MarkerArray, queue_size=1)

    # Subscribers
    rospy.Subscriber(node_name + '/box_config', Float64MultiArray, 
      self.boxConfigCallback, queue_size=1)

    # Service for plant box points
    self.inspection_points_service = rospy.Service(node_name + 
      '/get_inspection_points', GetPlantBoxInspectionPoints, 
      self.inspectionPointsCallback)

  def run(self):
    rate = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      rate.sleep()
      self.marker_array_pub.publish(self.marker_array)
      #print self.box_config_vector

  def boxConfigCallback(self, msg):
    if len(msg.data) < len(self.box_config_vector):
      print("[PlantInspection]->boxConfigCallback: At least 4 points must be provided")
    else:
      for i in range(0,len(self.box_config_vector)):
        self.box_config_vector[i] = msg.data[i]
      self.T_points = self.getInspectionPoints(self.box_config_vector)
      self.createVisualizationMessage()

  def inspectionPointsCallback(self, req):
    res = GetPlantBoxInspectionPointsResponse()

    if len(req.box_config_vector) < len(self.box_config_vector):
      print("[PlantInspection]->inspectionPointsCallback: At least 4 points must be provided")
      res.success = False
    else:
      for i in range(0,len(self.box_config_vector)):
        self.box_config_vector[i] = req.box_config_vector[i]
      self.T_points = self.getInspectionPoints(self.box_config_vector)
      self.createVisualizationMessage()

      # Apart from visualizing, return the pose array
      for i in range(len(self.T_points)):
        temp_pose = PoseStamped()
        temp_pose.pose = numpyMatrixToPose(self.T_points[i])
        res.inspection_points.poses.append(copy.deepcopy(temp_pose))

      res.success = True

    return res

  def createVisualizationMessage(self):
    self.marker_array.markers = []

    # common parts of marker message
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"

    # First add inspection points as sphere list
    marker.id = 0
    marker.ns = "inspection points"
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
    for i in range(len(self.T_points)):
      marker.points.append(numpyMatrixToPoint(self.T_points[i]))
    self.marker_array.markers.append(copy.deepcopy(marker))

    # Next add the plant box
    marker.id = len(self.marker_array.markers)
    marker.ns = "plant box"
    marker.type = Marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = self.lx
    marker.scale.y = self.ly
    marker.scale.z = self.lz
    marker.color.r = 37.0/255.0
    marker.color.g = 194.0/255.0
    marker.color.b = 79.0/255.0
    marker.color.a = 0.7
    marker.lifetime = rospy.Duration()
    marker.points = [] # Reset points
    marker.pose.position.x = self.box_config_vector[0]
    marker.pose.position.y = self.box_config_vector[1]
    marker.pose.position.z = self.box_config_vector[2]
    marker.pose.orientation.z = sin(self.box_config_vector[3]/2.0)
    marker.pose.orientation.w = cos(self.box_config_vector[3]/2.0)
    self.marker_array.markers.append(copy.deepcopy(marker))

    # Next add the plant box centroid
    marker.id = len(self.marker_array.markers)
    marker.ns = "plant box center"
    marker.type = Marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.01
    marker.color.r = 51.0/255.0
    marker.color.g = 41.0/255.0
    marker.color.b = 207.0/255.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()
    marker.points = [] # Reset points
    marker.pose.position.x = self.box_config_vector[0]
    marker.pose.position.y = self.box_config_vector[1]
    marker.pose.position.z = self.box_config_vector[2] + self.lz/2.0
    marker.pose.orientation.z = sin(self.box_config_vector[3]/2.0)
    marker.pose.orientation.w = cos(self.box_config_vector[3]/2.0)
    self.marker_array.markers.append(copy.deepcopy(marker))

    # Add arrows pointing the inspection direction
    for i in range(3):
      marker.id = len(self.marker_array.markers)
      marker.ns = "Directions"
      marker.type = Marker.ARROW
      marker.action = marker.ADD
      marker.scale.x = 0.5
      marker.scale.y = 0.04
      marker.scale.z = 0.04
      marker.color.r = 191.0/255.0
      marker.color.g = 25.0/255.0
      marker.color.b = 58.0/255.0
      marker.color.a = 1.0
      marker.lifetime = rospy.Duration()
      marker.points = [] # Reset points
      temp_point = numpyMatrixToPoint(self.T_points[i*2])
      marker.pose.position.x = temp_point.x
      marker.pose.position.y = temp_point.y
      marker.pose.position.z = temp_point.z
      alpha = atan2(self.T_points[i*2][1][0], self.T_points[i*2][0][0])
      marker.pose.orientation.z = sin(alpha/2.0)
      marker.pose.orientation.w = cos(alpha/2.0)
      self.marker_array.markers.append(copy.deepcopy(marker))


  def getInspectionPoints(self, box_vector):
    x0 = box_vector[0]
    y0 = box_vector[1]
    z0 = box_vector[2]
    yaw0 = box_vector[3]

    # Get inspection points in crate frame
    T_B_list = computeInspectionPointsInCrateFrame(
      self.lx, self.ly, self.r1, self.r2)

    # World to box transform
    T_W_B = transformMatrixFromTranslationAndYaw(box_vector)

    # Get points in world frame
    T_W_list = []
    for i in range(len(T_B_list)):
      T_W_list.append(np.matmul(T_W_B,T_B_list[i]))

    return T_W_list

def transformMatrixFromTranslationAndYaw(values):
    x = values[0]
    y = values[1]
    z = values[2]
    yaw = values[3]

    matrix = np.array([[cos(yaw), -sin(yaw), 0, x], 
      [sin(yaw), cos(yaw), 0, y], 
      [0, 0, 1, z],
      [0, 0, 0, 1]])
    return matrix

def computeInspectionPointsInCrateFrame(lx, ly, r1, r2):
  alpha = atan2(ly,lx)
  d1 = r1*ly/lx + ly/2
  d2 = r2*ly/lx + ly/2

  list_T = []

  # First append points from left
  temp_T = transformMatrixFromTranslationAndYaw([-lx/2-r1,d1,0,-alpha])
  list_T.append(copy.deepcopy(temp_T))
  temp_T = transformMatrixFromTranslationAndYaw([-lx/2-r2,d2,0,-alpha])
  list_T.append(copy.deepcopy(temp_T))
  # Middle points
  temp_T = transformMatrixFromTranslationAndYaw([-lx/2-r1,0,0,0])
  list_T.append(copy.deepcopy(temp_T))
  temp_T = transformMatrixFromTranslationAndYaw([-lx/2-r2,0,0,0])
  list_T.append(copy.deepcopy(temp_T))
  # Points on the right
  temp_T = transformMatrixFromTranslationAndYaw([-lx/2-r1,-d1,0,alpha])
  list_T.append(copy.deepcopy(temp_T))
  temp_T = transformMatrixFromTranslationAndYaw([-lx/2-r2,-d2,0,alpha])
  list_T.append(copy.deepcopy(temp_T))

  return list_T

def numpyMatrixToPoint(matrix):
  point = Point()
  point.x = matrix[0,3]
  point.y = matrix[1,3]
  point.z = matrix[2,3]
  return point

def numpyMatrixToPose(matrix):
  pose = Pose()
  pose.position.x = matrix[0,3]
  pose.position.y = matrix[1,3]
  pose.position.z = matrix[2,3]
  yaw = atan2(matrix[1,0], matrix[0,0])

  pose.orientation.w = cos(yaw/2.0)
  pose.orientation.z = sin(yaw/2.0)
  return pose

if __name__ == '__main__':
  node_name = 'plant_box_inspection_points'
  rospy.init_node(node_name)
  inspection_points = BoxInspectionPoints(node_name)
  inspection_points.run()
