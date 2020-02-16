#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float32
from std_srvs.srv import Empty, EmptyResponse
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest, DeleteModelResponse
import copy, os

class SpawnBall:

    def __init__(self):
        self.first_pose_received = False
        self.uav_current_pose = Pose()
        self.uav_reference_pose = Pose()
        self.spawn_ball = rospy.Service('spawn_ball', Empty, 
            self.spawnBallCallback)

        self.delete_model_service = rospy.ServiceProxy(
            "/gazebo/delete_model", DeleteModel)


        rospy.Subscriber('pose', PoseStamped, self.uavPoseCallback, 
            queue_size=1)


    def run(self):
        rospy.spin()

    def uavPoseCallback(self, msg):
        self.uav_current_pose = msg.pose
        self.first_pose_received = True

    def spawnBallCallback(self, req):
        # Spawn ball at UAV position with some offset
        if self.first_pose_received == True:
            req = DeleteModelRequest()
            req.model_name = "ball"
            self.delete_model_service.call(req)

            command = 'roslaunch larics_gazebo_worlds spawn_ball.launch x:=' + \
                str(round(self.uav_current_pose.position.x, 2)) + ' y:=' + \
                str(round(self.uav_current_pose.position.y, 2)) + ' z:=' + \
                str(round(self.uav_current_pose.position.z, 2)-0.1)
            os.system(command)
        return EmptyResponse()

if __name__ == '__main__':

    rospy.init_node('spawn_ball_at_uav')
    go_to = SpawnBall()
    go_to.run()

