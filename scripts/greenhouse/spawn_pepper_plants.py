#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, rospkg, roslaunch
import copy, os, csv, time
import numpy as np
from math import sin, cos, tan, atan2
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped
from larics_motion_planning.srv import GetPlantBoxInspectionPoints, \
  GetPlantBoxInspectionPointsRequest, GetPlantBoxInspectionPointsResponse

class PepperPlantSpawner:

  def __init__(self):
    # Get node parameters
    self.rate = rospy.get_param('~rate', 1)
    # Params for box dimensions
    self.lx = rospy.get_param('~box_length', 0.9)
    self.ly = rospy.get_param('~box_width', 0.63)
    self.lz = rospy.get_param('~box_height', 0.2)

    # This vector contains x,y,z,yaw 
    self.box_config_vector = [0]*4
    # List of inspection points
    self.T_points = []

    # Load data from file
    rospack = rospkg.RosPack()
    path = rospack.get_path('larics_motion_planning')
    self.box_centroids_file = rospy.get_param('~box_centroids_file', 
      path + '/config/greenhouse/greenhouse_box_centroids.csv')
    file = open(self.box_centroids_file)
    csvfile = csv.reader(file)
    pepper_count = 0

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #roslaunch.configure_logging(uuid)
    #cli_args = []
    launch_files = []
    for row in csvfile:
      self.box_config_vector[0] = float(row[0])
      self.box_config_vector[1] = float(row[1])
      self.box_config_vector[2] = float(row[2])
      self.box_config_vector[3] = float(row[5])

      self.T_points = self.getInspectionPoints(self.box_config_vector)
      launch_files = []
      for i in range(len(self.T_points)):
        cfg = numpyMatrixToPosAndYaw(self.T_points[i])
        x = "x:=" + str(cfg[0])
        y = "y:=" + str(cfg[1])
        z = "z:=" + str(cfg[2])
        yaw = "yaw:=" + str(cfg[3])
        name = "name:=pepper" + str(pepper_count)
        node_name = "node_name:=spawn_pepper" + str(pepper_count)
        pepper_count = pepper_count + 1
        command = "roslaunch larics_gazebo_worlds spawn_pepper_plant.launch" + \
          name + x + y + z + yaw
        #os.system(command)
        args = ['larics_gazebo_worlds', 'spawn_pepper_plant.launch', node_name, name, x, y, z, yaw]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(args)[0]
        roslaunch_args = args[2:]
        launch_files.append((roslaunch_file, roslaunch_args))
      parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
      parent.start()
      print "Spawned: ", pepper_count
      time.sleep(2)
        #cli_args.append(args)
      #print cli_args
      

  def run(self):
    pass

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

  def getInspectionPoints(self, box_vector):
    x0 = box_vector[0]
    y0 = box_vector[1]
    z0 = box_vector[2]
    yaw0 = box_vector[3]

    # Get inspection points in crate frame
    T_B_list = computeInspectionPointsInCrateFrame(
      self.lx, self.ly)

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

def computeInspectionPointsInCrateFrame(lx, ly):
  list_T = []

  # Add 4 plants per crate
  temp_T = transformMatrixFromTranslationAndYaw([lx/4,ly/4,0,0])
  list_T.append(copy.deepcopy(temp_T))
  temp_T = transformMatrixFromTranslationAndYaw([lx/4,-ly/4,0,0])
  list_T.append(copy.deepcopy(temp_T))
  temp_T = transformMatrixFromTranslationAndYaw([-lx/4,ly/4,0,0])
  list_T.append(copy.deepcopy(temp_T))
  temp_T = transformMatrixFromTranslationAndYaw([-lx/4,-ly/4,0,0])
  list_T.append(copy.deepcopy(temp_T))


  return list_T

def numpyMatrixToPoint(matrix):
  point = Point()
  point.x = matrix[0,3]
  point.y = matrix[1,3]
  point.z = matrix[2,3]
  return point

def numpyMatrixToPosAndYaw(matrix):
  pose = [0]*4
  pose[0] = matrix[0,3]
  pose[1] = matrix[1,3]
  pose[2] = matrix[2,3]
  yaw = atan2(matrix[1,0], matrix[0,0])
  pose[3] = yaw
  return pose

if __name__ == '__main__':
  rospy.init_node('pepper_spawner')
  spawn_peppers = PepperPlantSpawner()
  spawn_peppers.run()
