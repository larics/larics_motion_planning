#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
import copy, os, psutil
import numpy as np
from math import sin, cos, tan, atan2
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class BoxInspectionPoints:

  def __init__(self, node_name):
    # Get node parameters
    self.rate = rospy.get_param('~rate', 1)
    # Params for box dimensions
    self.lx = rospy.get_param('~box_length', 0.9)
    self.ly = rospy.get_param('~box_width', 0.63)
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

  def createVisualizationMessage(self):
    self.marker_array.markers = []

    # common parts of marker message
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"

    # First add inspection points as sphere list
    marker.id = 1
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
    for i in range(len(self.T_points)):
      marker.points.append(numpyMatrixToPoint(self.T_points[i]))
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

if __name__ == '__main__':
  node_name = 'plant_inspection'
  rospy.init_node(node_name)
  inspection_points = BoxInspectionPoints(node_name)
  inspection_points.run()
