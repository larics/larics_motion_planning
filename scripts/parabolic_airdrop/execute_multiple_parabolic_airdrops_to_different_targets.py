#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math, time, copy
import numpy as np
from math import sin, cos
from geometry_msgs.msg import Pose, PoseStamped
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

    # Set up executing for office or camellia city environment
    self.environment_type = rospy.get_param("~environment_type", "office")

    self.start_pose = Pose()
    self.delta_displacement_pose = Pose()
    if self.environment_type == "office":
      # Starting pose outside office
      self.start_pose.position.x = 35#-30#35
      self.start_pose.position.y = 20#-100#20
      self.start_pose.position.z = 1.0#2#1.0
      self.start_pose.orientation.w = 1.0
      # Displacement from final pose
      self.delta_displacement_pose.position.x = 0.0
      self.delta_displacement_pose.position.y = 0.0
      self.delta_displacement_pose.position.z = 6.0
      # Sleep times
      self.t_go_above_current = 15
      self.t_go_above_start = 45
      self.t_go_to_start = 22
      self.t_spawn_ball = 2
    elif self.environment_type == "camellia":
      # Starting pose outside office
      self.start_pose.position.x = -30.0
      self.start_pose.position.y = -100.0
      self.start_pose.position.z = 2.0
      self.start_pose.orientation.w = 1.0
      # Displacement from final pose
      self.delta_displacement_pose.position.x = -10.0
      self.delta_displacement_pose.position.y = 0.0
      self.delta_displacement_pose.position.z = 0.0
      # Sleep times
      self.t_go_above_current = 12
      self.t_go_above_start = 55
      self.t_go_to_start = 22
      self.t_spawn_ball = 2

    # Publisher for trajectory
    self.joint_trajectory_pub = rospy.Publisher('joint_trajectory', 
      JointTrajectory, queue_size=1)

    # AirdropService
    self.airdrop_service = rospy.ServiceProxy(
      'parabolic_airdrop_trajectory', ParabolicAirdropTrajectory)
    self.airdrop_request = ParabolicAirdropTrajectoryRequest()
    self.airdrop_request.uav_pose = copy.deepcopy(self.start_pose)
    self.airdrop_request.plan_path = True
    self.airdrop_request.plan_trajectory = True
    self.airdrop_request.publish_trajectory = False # We will publish!
    self.airdrop_request.use_custom_parabola_params = False
    self.airdrop_request.use_custom_psi_params = True
    self.airdrop_request.custom_psi_params.append(0)
    self.airdrop_request.custom_psi_params.append(1.0)
    self.airdrop_request.custom_psi_params.append(0)


    self.drops_per_config = rospy.get_param('~drops_per_config', int(5))
    self.filename = rospy.get_param('~configs_file', 
      str('/home/antun/catkin_ws/src/larics_motion_planning/config/airdrop_configs/office_multiple_targets.csv'))
    self.targets = np.loadtxt(open(self.filename, "rb"), delimiter=",")

    print("Config file: " + self.filename)
    print("Environment type: " + self.environment_type)
    print("Drops per config: " + str(self.drops_per_config))

    self.start_flag = False
    rospy.Subscriber('execute_multiple_aridrops/start', Bool, self.startCallback, 
      queue_size=1)
    self.uav_current_pose = Pose()
    rospy.Subscriber('pose', PoseStamped, self.uavPoseCallback, queue_size=1)
    print("Constructor done.")

  def run(self):
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
      rate.sleep()

      if self.start_flag == True:
        self.start_flag = False

        print("Starting to execute the procedure.")
        # Go through every config

        for i in range(len(self.targets)):
          x = self.targets[i][0]
          y = self.targets[i][1]
          z = self.targets[i][2]
          psi_min = self.targets[i][3]
          psi_inc = self.targets[i][4]
          psi_max = self.targets[i][5]
          current_drop_count = copy.deepcopy(i)

          # And execute as many times as user prompted
          should_publish = True
          for j in range(self.drops_per_config):
            rate.sleep()
            if should_publish == True:
              print(" ")
              print("Executing ", (current_drop_count+1), "/", len(self.targets), " for ", j+1, "/", self.drops_per_config, " time")
              print("Go above current position")
              pose = copy.deepcopy(self.uav_current_pose)
              # Add delta displacement to avoid obstacles
              pose.position.x = pose.position.x + self.delta_displacement_pose.position.x
              pose.position.y = pose.position.y + self.delta_displacement_pose.position.y
              pose.position.z = pose.position.z + self.delta_displacement_pose.position.z
              #pose.position.x = pose.position.x - 10.0
              self.go_to_pub.publish(pose)
              time.sleep(self.t_go_above_current)

              print("Go above starting position")
              pose_above = copy.deepcopy(self.start_pose)
              # Go to the same height as in previous goto
              pose_above.position.z = pose.position.z
              self.go_to_pub.publish(pose_above)
              time.sleep(self.t_go_above_start)

              print("Go to starting position")
              self.go_to_pub.publish(self.start_pose)
              time.sleep(self.t_go_to_start)

              # Spawn ball twice to eliminate weird shaking
              print("Spawn ball first time")
              self.spawn_ball_service(EmptyRequest())
              time.sleep(self.t_spawn_ball)
              #print("Spawn ball second time")
              #self.spawn_ball_service(EmptyRequest())
              #time.sleep(7)

            # Call airdrop service
            #self.airdrop_request.custom_parabola_params.clear()
            print("Plan and execute trajectory")
            self.airdrop_request.target_pose.position.x = x
            self.airdrop_request.target_pose.position.y = y
            self.airdrop_request.target_pose.position.z = z
            self.airdrop_request.custom_psi_params[0] = psi_min
            self.airdrop_request.custom_psi_params[1] = psi_inc
            self.airdrop_request.custom_psi_params[2] = psi_max
            print self.airdrop_request
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
              time.sleep(duration + 10)
            else:
              print("Ripmax, z<0.3 at some point.")
              print("Config v0 dz alpha dx psi")
              print("Config: ", v0, " ", dz, " ", alpha, " ", dx, " ", psi)



  def startCallback(self, msg):
    if self.start_flag == False:
      self.start_flag = True

  def uavPoseCallback(self, msg):
    self.uav_current_pose = msg.pose

if __name__ == '__main__':

  rospy.init_node('execute_multiple_parabolic_airdrops')
  execute = ExecuteMultipleParabolicAirdrops()
  execute.run()

