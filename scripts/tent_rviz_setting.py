#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
import copy, os, psutil
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class TentRviz:

  def __init__(self):
    self.tent_pub = rospy.Publisher('visualization/tent', MarkerArray, 
      queue_size=1)
    self.rate = rospy.get_param('~rate', 1)

  def run(self):
    rate = rospy.Rate(self.rate)
    self.tent_marker = self.getTentMarker(1)
    self.cameras_marker = self.getCamerasMarker(2)
    self.target_marker = self.getTargetMarker(3)
    self.target_marker_2 = self.getTargetMarker2(4)
    self.sponges_marker = self.getSpongesMarker(5)
    self.tatami_marker = self.getTatamiMarker(6)
    self.stand_marker = self.getStandMarker(7)

    marker_array = MarkerArray()
    marker_array.markers.append(self.tent_marker)
    marker_array.markers.append(self.cameras_marker)
    marker_array.markers.append(self.target_marker)
    marker_array.markers.append(self.target_marker_2)
    marker_array.markers.append(self.sponges_marker)
    marker_array.markers.append(self.tatami_marker)
    marker_array.markers.append(self.stand_marker)

    while not rospy.is_shutdown():
      rate.sleep()

      self.tent_pub.publish(marker_array)

  def getTentMarker(self, idd):
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

    marker.scale.x = 0.1
    #marker.scale.y = 0.1
    #marker.scale.z = 0.1

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 0.4

    marker.lifetime = rospy.Duration()

    #marker.mesh_resource = "file:///home/antun/Desktop/sator_simple.dae"
    #marker.mesh_use_embedded_materials = True

    point = Point()
    point.z = 0
    point.x = 4.3
    point.y = 3.82
    marker.points.append(copy.deepcopy(point))
    point.x = 4.3
    point.y = -6.1
    marker.points.append(copy.deepcopy(point))
    point.x = 4.3
    point.y = -6.1
    marker.points.append(copy.deepcopy(point))
    point.x = -3.25
    point.y = -6.1
    marker.points.append(copy.deepcopy(point))
    point.x = -3.25
    point.y = -6.1
    marker.points.append(copy.deepcopy(point))
    point.x = -3.25
    point.y = 3.82
    marker.points.append(copy.deepcopy(point))
    point.x = -3.25
    point.y = 3.82
    marker.points.append(copy.deepcopy(point))
    point.x = 4.3
    point.y = 3.82
    marker.points.append(copy.deepcopy(point))
    # Upper part
    point.z = 4
    point.x = 4.3
    point.y = 3.82
    marker.points.append(copy.deepcopy(point))
    point.x = 4.3
    point.y = -6.1
    marker.points.append(copy.deepcopy(point))
    point.x = 4.3
    point.y = -6.1
    marker.points.append(copy.deepcopy(point))
    point.x = -3.25
    point.y = -6.1
    marker.points.append(copy.deepcopy(point))
    point.x = -3.25
    point.y = -6.1
    marker.points.append(copy.deepcopy(point))
    point.x = -3.25
    point.y = 3.82
    marker.points.append(copy.deepcopy(point))
    point.x = -3.25
    point.y = 3.82
    marker.points.append(copy.deepcopy(point))
    point.x = 4.3
    point.y = 3.82
    marker.points.append(copy.deepcopy(point))
    # Vertical lines
    # Upper part
    point.z = 4
    point.x = 4.3
    point.y = 3.82
    marker.points.append(copy.deepcopy(point))
    point.z = 0
    point.x = 4.3
    point.y = 3.82
    marker.points.append(copy.deepcopy(point))
    point.z = 4
    point.x = 4.3
    point.y = -6.1
    marker.points.append(copy.deepcopy(point))
    point.z = 0
    point.x = 4.3
    point.y = -6.1
    marker.points.append(copy.deepcopy(point))
    point.z = 4
    point.x = -3.25
    point.y = -6.1
    marker.points.append(copy.deepcopy(point))
    point.z = 0
    point.x = -3.25
    point.y = -6.1
    marker.points.append(copy.deepcopy(point))
    point.z = 4
    point.x = -3.25
    point.y = 3.82
    marker.points.append(copy.deepcopy(point))
    point.z = 0
    point.x = -3.25
    point.y = 3.82
    marker.points.append(copy.deepcopy(point))

    return marker

  def getCamerasMarker(self, idd):
    marker = Marker()

    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD
    marker.id = idd

    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"

    #marker.pose.position.x = 0.55
    #marker.pose.position.y = 1.20
    marker.pose.position.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.8

    marker.lifetime = rospy.Duration()

    point = Point()
    point.x = 3.95
    point.y = 3.20
    point.z = 3.9
    marker.points.append(copy.deepcopy(point))
    point.x = 3.95
    point.y = -1.2
    point.z = 3.9
    marker.points.append(copy.deepcopy(point))
    point.x = 3.95
    point.y = -5.4
    point.z = 3.9
    marker.points.append(copy.deepcopy(point))
    point.x = 0.6
    point.y = -5.85
    point.z = 4.1
    marker.points.append(copy.deepcopy(point))
    point.x = -2.85
    point.y = -5.4
    point.z = 3.9
    marker.points.append(copy.deepcopy(point))
    point.x = -2.85
    point.y = -1.0
    point.z = 3.9
    marker.points.append(copy.deepcopy(point))
    point.x = -2.85
    point.y = 2.9
    point.z = 3.9
    marker.points.append(copy.deepcopy(point))

    return marker

  def getTargetMarker(self, idd):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.id = idd

    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"

    marker.pose.position.x = 0.0
    marker.pose.position.y = -3.25
    marker.pose.position.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.25
    marker.scale.y = 0.25
    marker.scale.z = 0.001

    marker.color.r = 1.0
    marker.color.g = 0.855
    marker.color.b = 0.55
    marker.color.a = 1

    marker.lifetime = rospy.Duration()

    return marker

  def getTargetMarker2(self, idd):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.id = idd

    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"

    marker.pose.position.x = 0.0
    marker.pose.position.y = -3.25
    marker.pose.position.z = 0.01
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.001

    marker.color.r = 0.93
    marker.color.g = 0.13
    marker.color.b = 0.13
    marker.color.a = 1

    marker.lifetime = rospy.Duration()

    return marker

  def getSpongesMarker(self, idd):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.id = idd

    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"

    marker.pose.position.x = 2.8
    marker.pose.position.y = 1.4
    marker.pose.position.z = 0.45
    marker.pose.orientation.w = 1.0

    marker.scale.x = 1.1
    marker.scale.y = 2.2
    marker.scale.z = 0.9

    marker.color.r = 0.99
    marker.color.g = 0.13
    marker.color.b = 0.13
    marker.color.a = 1

    marker.lifetime = rospy.Duration()

    return marker

  def getTatamiMarker(self, idd):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.id = idd

    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"

    marker.pose.position.x = 3.2
    marker.pose.position.y = -0.2
    marker.pose.position.z = 0.25
    marker.pose.orientation.w = 1.0

    marker.scale.x = 1.1
    marker.scale.y = 1.1
    marker.scale.z = 0.5

    marker.color.r = 0.99
    marker.color.g = 0.13
    marker.color.b = 0.13
    marker.color.a = 1

    marker.lifetime = rospy.Duration()

    return marker

  def getStandMarker(self, idd):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.id = idd

    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"

    marker.pose.position.x = 3.5
    marker.pose.position.y = -1.55
    marker.pose.position.z = 1.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 1.5
    marker.scale.y = 1.5
    marker.scale.z = 2.0

    marker.color.r = 0.99
    marker.color.g = 0.13
    marker.color.b = 0.13
    marker.color.a = 1

    marker.lifetime = rospy.Duration()

    return marker

if __name__ == '__main__':

  rospy.init_node('tent_rviz_setting')
  tent = TentRviz()
  tent.run()
