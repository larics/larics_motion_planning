#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
import copy, os, psutil
import numpy as np
import math
from math import sin, cos, tan, atan2
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped
from larics_motion_planning.srv import GetPlantBoxInspectionPoints, \
  GetPlantBoxInspectionPointsRequest, GetPlantBoxInspectionPointsResponse, \
  VisualizeState, VisualizeStateRequest, VisualizeStateResponse

class BoxInspectionPoints:

  def __init__(self, node_name):
    # Get node parameters
    self.rate = rospy.get_param('~rate', 1)
    # Params for box dimensions
    self.lx = rospy.get_param('~box_length', 0.5) #0.9
    self.ly = rospy.get_param('~box_width', 0.25) #0.63
    self.lz = rospy.get_param('~box_height', 0.5) #0.2
    # Get r1 and r2 distances from box during inspection
    self.r1 = rospy.get_param('~r1', 1.0)
    self.r2 = rospy.get_param('~r2', 0.5)
    # Get how much above(or below) relative to the crate do you want inspection
    self.plants_delta_z = rospy.get_param('~plants_delta_z', 0.0)

    # Another type of inspection points is ellipsoid. These are additional
    # parameters for it.
    # First the distance from the plant itself
    self.ellipse_d = rospy.get_param('~ellipse/distance', 1.0)
    # Which type of waypoint planner is used. By default it will be box, but
    # change it if ellipse is selected.
    self.waypoints_type = rospy.get_param('~waypoints_type', 'ellipse_row')
    print("[PlantInspection]->init: Using " + self.waypoints_type + " waypoints type.")
    # Angle step for sampling the parametrized ellipse. Automatically
    # recalculate angle step to be integer multiple of pi.
    self.ellipse_angle_step = rospy.get_param('~ellipse/angle_step', 0.75)
    self.ellipse_n_points = math.ceil(2.0*math.pi/self.ellipse_angle_step)
    self.ellipse_angle_step = 2.0*math.pi/self.ellipse_n_points
    self.ellipse_n_points = self.ellipse_n_points + 1
    print("[PlantInspection]->init: Recalculated ellipse angle step: " +
      str(self.ellipse_angle_step))
    # Alpha down and up refer to max and min angles of the manipulator pitch
    # during inspection.
    self.ellipse_alpha_down = rospy.get_param('~ellipse/alpha_down', 0.267) #0.685
    self.ellipse_alpha_up = rospy.get_param('~ellipse/alpha_up', -0.267)

    # Well, more additional parameters for ellipse row
    self.ellipse_row_lx = rospy.get_param('~ellipse_row/lx', 2.0)
    self.ellipse_row_ly = rospy.get_param('~ellipse_row/ly', 2.0)
    self.ellipse_row_n_plants = rospy.get_param(
      '~ellipse_row/number_of_plants', int(4))
    # Also, calling visualize state to get end effector pose is part of ellipse
    # row stuff
    self.visualize_state_service = rospy.ServiceProxy(
      "visualize_state", VisualizeState)

    # This vector contains x,y,z,yaw 
    self.box_config_vector = [0]*4
    # List of inspection points
    self.T_points = []
    # Additionally alpha angles for ellipse and delta yaw for manipulator to
    # always look towards the center of the plant
    self.alpha = []
    self.delta_yaw = []

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
      #print(self.box_config_vector)

  def boxConfigCallback(self, msg):
    res = GetPlantBoxInspectionPointsResponse()
    if len(msg.data) < len(self.box_config_vector):
      print("[PlantInspection]->boxConfigCallback: At least 4 points must be provided")
    else:
      for i in range(0,len(self.box_config_vector)):
        self.box_config_vector[i] = msg.data[i]
      if self.waypoints_type == 'box':
        self.T_points = self.getInspectionPointsBox(self.box_config_vector)
      elif self.waypoints_type == 'ellipse':
        self.T_points, self.alpha, self.delta_yaw = self.getInspectionPointsEllipse(self.box_config_vector)
        res.ellipse_alpha_vector = self.alpha
        res.ellipse_delta_yaw = self.delta_yaw
      elif self.waypoints_type == 'ellipse_row':
        self.T_points, self.alpha, self.delta_yaw = self.getInspectionPointsEllipseRow(self.box_config_vector)
        res.ellipse_alpha_vector = self.alpha
        res.ellipse_delta_yaw = self.delta_yaw
      else:
        print("[PlantInspection]->boxConfigCallback: No such waypoints type!")
      self.createVisualizationMessage()

  def inspectionPointsCallback(self, req):
    res = GetPlantBoxInspectionPointsResponse()

    if len(req.box_config_vector) < len(self.box_config_vector):
      print("[PlantInspection]->inspectionPointsCallback: At least 4 points must be provided")
      res.success = False
    else:
      for i in range(0,len(self.box_config_vector)):
        self.box_config_vector[i] = req.box_config_vector[i]
      if self.waypoints_type == 'box':
        self.T_points = self.getInspectionPointsBox(self.box_config_vector)
      elif self.waypoints_type == 'ellipse':
        self.T_points, self.alpha, self.delta_yaw = self.getInspectionPointsEllipse(self.box_config_vector)
        res.ellipse_alpha_vector = self.alpha
        res.ellipse_delta_yaw = self.delta_yaw
      elif self.waypoints_type == 'ellipse_row':
        self.T_points, self.alpha, self.delta_yaw = self.getInspectionPointsEllipseRow(self.box_config_vector)
        res.ellipse_alpha_vector = self.alpha
        res.ellipse_delta_yaw = self.delta_yaw
      else:
        print("[PlantInspection]->boxConfigCallback: No such waypoints type!")
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
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"
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
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"
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
    if self.waypoints_type == "box":
      for i in range(3):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "world"
        marker.id = len(self.marker_array.markers)
        marker.ns = "Directions"
        marker.type = Marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = abs(self.r1-self.r2)
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


  def getInspectionPointsBox(self, box_vector):
    x0 = box_vector[0]
    y0 = box_vector[1]
    z0 = box_vector[2]
    yaw0 = box_vector[3]

    # Get inspection points in crate frame
    T_B_list = computeInspectionPointsInCrateFrame(
      self.lx, self.ly, self.r1, self.r2, self.plants_delta_z)

    # World to box transform
    T_W_B = transformMatrixFromTranslationAndYaw(box_vector)

    # Get points in world frame
    T_W_list = []
    for i in range(len(T_B_list)):
      T_W_list.append(np.matmul(T_W_B,T_B_list[i]))

    return T_W_list

  def getInspectionPointsEllipse(self, box_vector):
    print("PlantInspection]->getInspectionPointsEllipse started.")
    x0 = box_vector[0]
    y0 = box_vector[1]
    z0 = box_vector[2]
    yaw0 = box_vector[3]

    # First sample the ellipse in y-z axis
    list_T = []
    list_alpha = []
    list_yaw = []
    for i in range(int(self.ellipse_n_points)):
      dy = (self.ly/2.0)*sin((self.ellipse_n_points-i-1)*self.ellipse_angle_step)
      dz = (self.lz/2.0)*cos((self.ellipse_n_points-i-1)*self.ellipse_angle_step)
      # Get ellipse points in the center of plant frame
      temp_T = transformMatrixFromTranslationAndYaw([0,dy,dz,0])
      # Rotate distance vector which is initially only along x axis.
      Rd = transformMatrixFromTranslationAndYaw([0,0,0,box_vector[3]])
      Td = transformMatrixFromTranslationAndYaw([self.ellipse_d,0,0,0])
      Td = np.matmul(Rd, Td)
      # Rotate and translate initial points to be farther away from the
      # box at the imagined origin.
      temp_T = np.matmul(Td, temp_T)
      # Finally translate points to be in front of the box
      Tw = transformMatrixFromTranslationAndYaw([x0, y0, z0, 0])
      temp_T = np.matmul(Tw, temp_T)
      list_T.append(copy.deepcopy(temp_T))

      # Calcululate alpha angle which will be pitch of the manipulator during
      # inspection
      if dz >= 0.0:
        # When the uav is above the plant center, this will look down since
        # alpha_up is negative, and dz is positive
        list_alpha.append(self.ellipse_alpha_up*dz/(self.lz/2.0))
      else:
        # When the uav is below the plant center, this will look up
        list_alpha.append(-self.ellipse_alpha_down*dz/(self.lz/2.0))

      # Also calculate delta yaw which the UAV has to execute to look towards
      # the center of the plant.
      list_yaw.append(atan2(dy,self.ellipse_d))

    return list_T, list_alpha, list_yaw

  def getInspectionPointsEllipseRow(self, row_center):
    print("PlantInspection]->getInspectionPointsEllipseRow started.")
    # These are now row centroids.
    x0 = row_center[0]
    y0 = row_center[1]
    z0 = row_center[2]
    yaw0 = row_center[3]

    list_T = []
    list_alpha = []
    list_yaw = []

    # Following will find the points in local frame of plant row! Afterwards, 
    # we will transform them to the world frame
    # Separation between plants along x axis
    sep = self.ellipse_row_lx/float(self.ellipse_row_n_plants)
    if (self.ellipse_row_n_plants % 2) == 0:
      start = -float(self.ellipse_row_n_plants/2-1)*sep - 0.5*sep
    else:
      start = -float(math.floor(self.ellipse_row_n_plants/2))*sep

    # This above was in local frame along x axis of the row. Rotate it and do
    # the front row
    sep_x = sep*cos(yaw0)
    sep_y = sep*sin(yaw0)
    start_x = start*cos(yaw0) + x0
    start_y = start*sin(yaw0) + y0
    lx = 0*self.ellipse_row_lx*cos(yaw0) - self.ellipse_row_ly*sin(yaw0)
    ly = 0*self.ellipse_row_lx*sin(yaw0) + self.ellipse_row_ly*cos(yaw0)

    # First the side ellipse
    i = 0
    box_vector = [start_x + float(i)*sep_x + lx/4.0, start_y + float(i)*sep_y + ly/4.0, \
        z0, yaw0+math.pi/2 + math.pi/2]
    l_T, l_alpha, l_yaw = self.getInspectionPointsEllipse(box_vector)
    list_T.append(copy.deepcopy(l_T[0]))
    list_alpha.append(copy.deepcopy(l_alpha[0]))
    list_yaw.append(copy.deepcopy(l_yaw[0]))
    # Then a point on the side
    i = 0
    box_vector = [start_x + float(i)*sep_x + lx/2.0, start_y + float(i)*sep_y + ly/2.0, \
        z0, yaw0+math.pi/2 + math.pi/2]
    l_T, l_alpha, l_yaw = self.getInspectionPointsEllipse(box_vector)
    list_T.append(copy.deepcopy(l_T[0]))
    list_alpha.append(copy.deepcopy(l_alpha[0]))
    list_yaw.append(copy.deepcopy(l_yaw[0]))
    # Then the front row
    for i in range(self.ellipse_row_n_plants):
      box_vector = [start_x + float(i)*sep_x, start_y + float(i)*sep_y, \
        z0, yaw0+math.pi/2]
      l_T, l_alpha, l_yaw = self.getInspectionPointsEllipse(box_vector)
      list_T.extend(copy.deepcopy(l_T))
      list_alpha.extend(copy.deepcopy(l_alpha))
      list_yaw.extend(copy.deepcopy(l_yaw))
    # Then a point on the side
    i = self.ellipse_row_n_plants - 1
    box_vector = [start_x + float(i)*sep_x + lx/2.0, start_y + float(i)*sep_y + ly/2.0, \
        z0, yaw0+math.pi/2 - math.pi/2]
    l_T, l_alpha, l_yaw = self.getInspectionPointsEllipse(box_vector)
    list_T.append(copy.deepcopy(l_T[0]))
    list_alpha.append(copy.deepcopy(l_alpha[0]))
    list_yaw.append(copy.deepcopy(l_yaw[0]))
    # And the ellipse on the other side
    i = self.ellipse_row_n_plants - 1
    box_vector = [start_x + float(i)*sep_x + lx/4.0, start_y + float(i)*sep_y + ly/4.0, \
        z0, yaw0+math.pi/2 - math.pi/2]
    l_T, l_alpha, l_yaw = self.getInspectionPointsEllipse(box_vector)
    list_T.append(copy.deepcopy(l_T[0]))
    list_alpha.append(copy.deepcopy(l_alpha[0]))
    list_yaw.append(copy.deepcopy(l_yaw[0]))

    # Back row first side ellipse
    i = self.ellipse_row_n_plants - 1
    box_vector = [start_x + float(i)*sep_x - lx/4.0, start_y + float(i)*sep_y - ly/4.0, \
        z0, yaw0+math.pi/2+math.pi+math.pi/2]
    l_T, l_alpha, l_yaw = self.getInspectionPointsEllipse(box_vector)
    list_T.append(copy.deepcopy(l_T[0]))
    list_alpha.append(copy.deepcopy(l_alpha[0]))
    list_yaw.append(copy.deepcopy(l_yaw[0]))
    # Then a point
    i = self.ellipse_row_n_plants - 1
    box_vector = [start_x + float(i)*sep_x - lx/2.0, start_y + float(i)*sep_y - ly/2.0, \
        z0, yaw0+math.pi/2+math.pi+math.pi/2]
    l_T, l_alpha, l_yaw = self.getInspectionPointsEllipse(box_vector)
    list_T.append(copy.deepcopy(l_T[0]))
    list_alpha.append(copy.deepcopy(l_alpha[0]))
    list_yaw.append(copy.deepcopy(l_yaw[0]))
    
    # Back row
    for i in range(self.ellipse_row_n_plants-1, -1, -1):
      box_vector = [start_x + float(i)*sep_x, start_y + float(i)*sep_y, \
        z0, yaw0+math.pi/2+math.pi]
      l_T, l_alpha, l_yaw = self.getInspectionPointsEllipse(box_vector)
      list_T.extend(copy.deepcopy(l_T))
      list_alpha.extend(copy.deepcopy(l_alpha))
      list_yaw.extend(copy.deepcopy(l_yaw))
    # Then a point
    i = 0
    box_vector = [start_x + float(i)*sep_x - lx/2.0, start_y + float(i)*sep_y - ly/2.0, \
        z0, yaw0+math.pi/2+math.pi-math.pi/2]
    l_T, l_alpha, l_yaw = self.getInspectionPointsEllipse(box_vector)
    list_T.append(copy.deepcopy(l_T[0]))
    list_alpha.append(copy.deepcopy(l_alpha[0]))
    list_yaw.append(copy.deepcopy(l_yaw[0]))
    # Other side ellipse
    i = 0
    box_vector = [start_x + float(i)*sep_x - lx/4.0, start_y + float(i)*sep_y - ly/4.0, \
        z0, yaw0+math.pi/2+math.pi-math.pi/2]
    l_T, l_alpha, l_yaw = self.getInspectionPointsEllipse(box_vector)
    list_T.append(copy.deepcopy(l_T[0]))
    list_alpha.append(copy.deepcopy(l_alpha[0]))
    list_yaw.append(copy.deepcopy(l_yaw[0]))

    return list_T, list_alpha, list_yaw

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

def computeInspectionPointsInCrateFrame(lx, ly, r1, r2, dz):
  alpha = atan2(ly,lx)
  d1 = r1*ly/lx + ly/2
  d2 = r2*ly/lx + ly/2

  list_T = []

  # First append points from left
  temp_T = transformMatrixFromTranslationAndYaw([-lx/2-r1,d1,dz,-alpha])
  list_T.append(copy.deepcopy(temp_T))
  temp_T = transformMatrixFromTranslationAndYaw([-lx/2-r2,d2,dz,-alpha])
  list_T.append(copy.deepcopy(temp_T))
  # Middle points
  temp_T = transformMatrixFromTranslationAndYaw([-lx/2-r1,0,dz,0])
  list_T.append(copy.deepcopy(temp_T))
  temp_T = transformMatrixFromTranslationAndYaw([-lx/2-r2,0,dz,0])
  list_T.append(copy.deepcopy(temp_T))
  # Points on the right
  temp_T = transformMatrixFromTranslationAndYaw([-lx/2-r1,-d1,dz,alpha])
  list_T.append(copy.deepcopy(temp_T))
  temp_T = transformMatrixFromTranslationAndYaw([-lx/2-r2,-d2,dz,alpha])
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
