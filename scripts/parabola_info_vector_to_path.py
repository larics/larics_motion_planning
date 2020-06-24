#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
import copy
from math import sin, cos, ceil
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class ParabolaInfoToPath:

  def __init__(self):
    self.parabola_path_pub = rospy.Publisher('visualization/parabola_path',
      Path, queue_size=1)

    self.path = Path()
    self.path.header.frame_id = "world"
    self.rate = rospy.get_param('~rate', 1)

    self.info_vector = Float64MultiArray()
    rospy.Subscriber('parabolic_airdrop/info_vector', Float64MultiArray, 
      self.infoVectorCallback, queue_size=1)

  def run(self):
    rate = rospy.Rate(self.rate)

    while not rospy.is_shutdown():
      rate.sleep()

      self.parabola_path_pub.publish(self.path)

  def infoVectorCallback(self, msg):
    self.info_vector = msg
    self.path = Path()

    d = self.info_vector.data[0]
    dz = self.info_vector.data[1]
    v0 = self.info_vector.data[2]
    theta = self.info_vector.data[3]
    psi = self.info_vector.data[4]
    duration = self.info_vector.data[5]
    x0 = self.info_vector.data[12]
    y0 = self.info_vector.data[13]
    z0 = self.info_vector.data[14]
    self.path = self.generateParabola(d, dz, v0, theta, psi, duration, x0, y0, z0)


  def generateParabola(self, d, dz, v0, theta, psi, duration, x0, y0, z0):
    g = 9.81

    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "world"

    ts = 0.01
    n = int(ceil(duration/ts))
    pose = PoseStamped()

    for i in range(n):
      t = float(i)*ts
      pose.pose.position.x = x0 + v0*cos(theta)*cos(psi)*t
      pose.pose.position.y = y0 + v0*cos(theta)*sin(psi)*t
      pose.pose.position.z = z0 + v0*cos(psi)*t - g*t*t/2.0


      path.poses.append(copy.deepcopy(pose))

    return path


if __name__ == '__main__':

  rospy.init_node('parabola_info_vector_to_path')
  par = ParabolaInfoToPath()
  par.run()
