#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math, time, copy
import numpy as np
from math import sin, cos
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, Bool
from std_srvs.srv import Empty, EmptyRequest
from larics_motion_planning.srv import ParabolicAirdropTrajectory, \
  ParabolicAirdropTrajectoryRequest, ParabolicAirdropTrajectoryResponse
from trajectory_msgs.msg import JointTrajectory

class ExecuteMultipleParabolicAirdrops:

  def __init__(self):
    self.spawn_ball_service = rospy.ServiceProxy(
      "spawn_ball", Empty)
    self.go_to_pub = rospy.Publisher('go_to/reference', 
      Pose, queue_size=1)
    self.go_to_pose = Pose()
    self.go_to_pose.position.x = -9
    self.go_to_pose.position.z = 4.5
    self.go_to_pose.orientation.w = 1.0

    # Publisher for trajectory
    self.joint_trajectory_pub = rospy.Publisher('joint_trajectory', 
      JointTrajectory, queue_size=1)

    # AirdropService
    self.airdrop_service = rospy.ServiceProxy(
      'parabolic_airdrop_trajectory', ParabolicAirdropTrajectory)
    self.airdrop_request = ParabolicAirdropTrajectoryRequest()
    self.airdrop_request.uav_pose = copy.deepcopy(self.go_to_pose)
    self.airdrop_request.target_pose.position.z = 0.1
    self.airdrop_request.plan_trajectory = True
    self.airdrop_request.publish_trajectory = False # We will publish!
    self.airdrop_request.use_custom_parabola_params = True
    self.airdrop_request.custom_parabola_params.append(0)
    self.airdrop_request.custom_parabola_params.append(0)
    self.airdrop_request.custom_parabola_params.append(0)
    self.airdrop_request.custom_parabola_params.append(0)
    self.airdrop_request.custom_parabola_params.append(0)

    self.drops_per_config = rospy.get_param('~drops_per_config', int(5))
    self.filename = rospy.get_param('~configs_file', 
      str('/home/antun/catkin_ws/src/larics_motion_planning/config/airdrop_configs/airdrop_configs_for_paper.csv'))
    self.airdrop_configs = np.loadtxt(open(self.filename, "rb"), delimiter=",")

    self.start_flag = False
    rospy.Subscriber('execute_multiple_aridrops/start', Bool, self.startCallback, 
      queue_size=1)
    print "Constructor done."

  def run(self):
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
      rate.sleep()

      if self.start_flag == True:
        self.start_flag = False

        print "Starting to execute the procedure."
        # Go through every config

        for i in range(len(self.airdrop_configs)):
          v0 = self.airdrop_configs[i][0]
          dz = self.airdrop_configs[i][1]
          alpha = math.radians(self.airdrop_configs[i][2])
          dx = self.airdrop_configs[i][3]
          psi = self.airdrop_configs[i][4]
          current_drop_count = copy.deepcopy(i)

          # And execute as many times as user prompted
          should_publish = True
          for j in range(self.drops_per_config):
            if should_publish == True:
              print " "
              print "Executing ", (current_drop_count+1), "/", len(self.airdrop_configs), " for ", j+1, "/", self.drops_per_config, " time"
              print "Go to position"
              self.go_to_pub.publish(self.go_to_pose)
              time.sleep(10)

              # Spawn ball twice to eliminate weird shaking
              print "Spawn ball first time"
              self.spawn_ball_service(EmptyRequest())
              time.sleep(7)
              #print "Spawn ball second time"
              #self.spawn_ball_service(EmptyRequest())
              #time.sleep(7)

            # Call airdrop service
            #self.airdrop_request.custom_parabola_params.clear()
            print "Plan and execute trajectory"
            self.airdrop_request.custom_parabola_params[0] = v0
            self.airdrop_request.custom_parabola_params[1] = dz
            self.airdrop_request.custom_parabola_params[2] = alpha
            self.airdrop_request.custom_parabola_params[3] = dx
            self.airdrop_request.custom_parabola_params[4] = psi
            res = self.airdrop_service(self.airdrop_request)
            duration = len(res.trajectory.points)/100.0

            # Check trajectory z axis
            should_publish = True
            for i in range(len(res.trajectory.points)):
              if res.trajectory.points[i].positions[2] < 0.3:
                should_publish = False
                break

            if should_publish == True:
              self.joint_trajectory_pub.publish(res.trajectory)
              time.sleep(duration + 5)
            else:
              print "Ripmax, z<0.3 at some point."
              print "Config v0 dz alpha dx psi"
              print "Config: ", v0, " ", dz, " ", alpha, " ", dx, " ", psi



  def startCallback(self, msg):
    if self.start_flag == False:
      self.start_flag = True

if __name__ == '__main__':

  rospy.init_node('execute_multiple_parabolic_airdrops')
  execute = ExecuteMultipleParabolicAirdrops()
  execute.run()

