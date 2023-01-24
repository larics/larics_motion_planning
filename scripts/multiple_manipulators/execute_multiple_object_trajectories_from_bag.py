#!/usr/bin/env python

__author__ = 'aivanovic'

import rosbag, rospy, rospkg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from larics_motion_planning.srv import MultiDofTrajectory, \
  MultiDofTrajectoryRequest, MultiDofTrajectoryResponse

class ExecuteTrajectoriesFromBagPath:

  def __init__(self):
    rospack = rospkg.RosPack()
    
    self.bag_path = rospack.get_path('larics_motion_planning') + \
      '/scripts/multiple_manipulators/warehouse_paths.bag'

    # This is in object namespace
    self.plan_full_state_trajectory = rospy.ServiceProxy(
      "plan_full_state_trajectory", MultiDofTrajectory)

  def run(self):
    bag = rosbag.Bag(self.bag_path)
    num_topics = bag.get_message_count('/object/path_as_joint_trajectory')
    
    # Go through all paths in the bag and plan a trajectory.
    i = 1
    for topic, msg, t in bag.read_messages(topics=['/object/path_as_joint_trajectory']):
      printstr = "Planning trajectory " + str(i) + "/" + str(num_topics)
      print(printstr)
      req = MultiDofTrajectoryRequest()
      req.plan_path = False
      req.publish_path = False
      req.plan_trajectory = False
      req.publish_trajectory = False
      req.waypoints = msg
      res = self.plan_full_state_trajectory(req)
      printstr = "Executing trajectory " + str(i) + "/" + str(num_topics)
      print(printstr)
      i = i + 1
      rospy.sleep(20.0)

    bag.close()


if __name__ == '__main__':

  rospy.init_node('execute_multiple_trajectories_from_bag')
  executor = ExecuteTrajectoriesFromBagPath()
  executor.run()