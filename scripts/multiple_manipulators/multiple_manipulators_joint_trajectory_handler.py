#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import copy

"""
Napravit cemo tako da za svaki novi manipulator dodamo klasu koja ima
publishere za taj manipulator. 

Svaku od tih klasa ce trebati konfigurirati
tako da joj se podese topici i namespace-ovi jer je sada planer u 
/planner namespaceu. Takoder, treba dodati u neki config file (a mozda
najbolje u onaj za planer) te stvari i koji se manipulatori koriste i
koliko ih je. 

Klasa za svaki manipulator neka ide u zaseban file pa cemo ih
ovdje loadati. 

Takoder, svaka od tih klasa ce primiti neki komad poruke i onda
ga slati naokolo gdje vec treba. Rate za sve je isti jer ce jedna for petlja
proci kroz sve klase i samo im setati ono sto trebaju publishati i onda ih
natjerati da to publishaju.
"""

class MultipleManipulatorsJointTrajectoryHandler:

  def __init__(self):
    # Parameters
    self.rate = rospy.get_param('~rate', 100)



    # Subscriber for joint trajectory point with all degrees of freedom
    self.current_trajectory_point = JointTrajectoryPoint()
    rospy.Subscriber(
      'multiple_manipulators_joint_trajectory_handler/joint_trajectory_point_in', 
      JointTrajectoryPoint, self.jointTrajectoryPointCallback, queue_size=1)

  def run(self):
    rate = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      rate.sleep()

  def jointTrajectoryPointCallback(self, msg):
    self.current_trajectory_point = msg


if __name__ == '__main__':
  rospy.init_node('multiple_manipulators_joint_trajectory_handler')
  handler = MultipleManipulatorsJointTrajectoryHandler()
  handler.run()