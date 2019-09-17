#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math, time
from math import sin, cos
from geometry_msgs.msg import Twist, Transform, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Int32, Float32
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
    MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint
from larics_motion_planning.srv import MultiDofTrajectory, \
    MultiDofTrajectoryRequest, MultiDofTrajectoryResponse
import copy
from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryRequest, \
    GenerateTrajectoryResponse

class JointTrajectoryToUavAndWpManipulatorReference:

    def __init__(self):
        # UAV publishers for trajectory
        self.uav_trajectory_point_pub = rospy.Publisher('/euroc3/command/trajectory', 
            MultiDOFJointTrajectory, queue_size=1)

        # Manipulator publishers
        self.manipulator_joint1_pub = rospy.Publisher(
            '/euroc3/wp_manipulator_3r/joint1', Float32, queue_size=1)
        self.manipulator_joint2_pub = rospy.Publisher(
            '/euroc3/wp_manipulator_3r/joint2', Float32, queue_size=1)
        self.manipulator_joint3_pub = rospy.Publisher(
            '/euroc3/wp_manipulator_3r/joint3', Float32, queue_size=1)

        # Alternatively, publish directly as joint trajectory
        self.manipulator_trajectory_pub = rospy.Publisher(
            '/euroc3/wp_manipulator_3r/joint_trajectory', JointTrajectory, queue_size=1)

        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(self.rate) 
        self.t_start = rospy.Time.now()
        self.executing_trajectory_flag = False
        self.trajectory = JointTrajectory()
        self.current_trajectory_point = JointTrajectoryPoint()
        self.uav_current_trajectory_point = MultiDOFJointTrajectoryPoint()
        self.roll = 0
        self.pitch = 0
        self.reinterpolate_flag = True

        #rospy.Subscriber('joint_trajectory', JointTrajectory, 
        #    self.jointTrajectoryCallback, queue_size=1)
        rospy.Subscriber('/euroc3/vrpn_client/estimated_transform', 
            TransformStamped, self.transformCallback, queue_size=1)

        self.execute_trajectory_service = rospy.Service('/simulate_arducopter', 
            MultiDofTrajectory, self.executeTrajectoryCallback)

        self.toppra_trajectory_service = rospy.ServiceProxy(
            "/uav/generate_toppra_trajectory", GenerateTrajectory)

    def run(self):
        
        rospy.spin()

    def executeTrajectoryCallback(self, req):
        response = MultiDofTrajectoryResponse()
        response.trajectory = copy.deepcopy(req.waypoints)
        return response
        if len(req.waypoints.points) < 0:
            print "0 points in trajectory."
            response = MultiDofTrajectoryResponse()
            response.path_length = -1.0
            return response

        print "Starting to execute trajectory. Length: ", len(req.waypoints.points)
        # Go through all points
        response = MultiDofTrajectoryResponse()
        rate = rospy.Rate(self.rate)
        for i in range(len(req.waypoints.points)):
            self.current_trajectory_point = req.waypoints.points[i]
            self.uav_current_trajectory_point = jointTrajectoryPointToMultiDofJointTrajectoryPoint(
                self.current_trajectory_point)
            self.publishAll()

            point = copy.deepcopy(self.current_trajectory_point)
            lst = list(point.positions)
            lst[3] = self.roll
            lst[4] = self.pitch
            point.positions = tuple(lst)
            response.trajectory.points.append(point)

            rate.sleep()

        tstart = time.time()
        while True: #(((abs(self.roll) > 0.001) or (abs(self.pitch) > 0.001)) or ((time.time() - tstart) < 0.5)):
            point = copy.deepcopy(self.current_trajectory_point)
            lst = list(point.positions)
            lst[3] = self.roll
            lst[4] = self.pitch
            point.positions = tuple(lst)
            response.trajectory.points.append(point)

            if (time.time()-tstart) > 2.1:
                break

            rate.sleep()

        print "Lenghtened trajectory length: ", len(response.trajectory.points)
        print "Trajectory executed!"

        # Reinterpolate trajectory with roll, pitch and new manipulator values
        # so it is within dynamic limits
        if self.reinterpolate_flag == True:
            toppra_request = GenerateTrajectoryRequest()
            toppra_request.waypoints = copy.deepcopy(response.trajectory)
            
            # Change first point velocities and accelerations to dynamic limits
            vel_limit = [0.8, 0.8, 0.5, 100, 100, 0.3, 1.0, 1.0, 1.0]
            lst = list(toppra_request.waypoints.points[0].velocities)
            toppra_request.waypoints.points[0].velocities = tuple(vel_limit)
            acc_limit = [0.8, 0.8, 0.4, 100, 100, 0.3, 1.2, 1.2, 1.2]
            lst = list(toppra_request.waypoints.points[0].accelerations)
            toppra_request.waypoints.points[0].accelerations = tuple(acc_limit)
            toppra_request.sampling_frequency = 100.0
            toppra_request.plot = False
            toppra_response = self.toppra_trajectory_service(toppra_request)

            print "Reinterpolated toppra trajectory length: ", len(toppra_response.trajectory.points)

            response.trajectory = copy.deepcopy(toppra_response.trajectory)

        return response

    def transformCallback(self, msg):
        q0 = msg.transform.rotation.w
        q1 = msg.transform.rotation.x
        q2 = msg.transform.rotation.y
        q3 = msg.transform.rotation.z

        self.roll = math.atan2(2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2))
        self.pitch = math.asin(2*(q0*q2 - q3*q1))
        #print "{:10.6f}".format(self.roll), "{:10.6f}".format(self.pitch)

    def publishAll(self):
        self.manipulator_joint1_pub.publish(self.current_trajectory_point.positions[4+2])
        self.manipulator_joint2_pub.publish(self.current_trajectory_point.positions[5+2])
        self.manipulator_joint3_pub.publish(self.current_trajectory_point.positions[6+2])
        temp_trajectory = MultiDOFJointTrajectory()
        temp_trajectory.points.append(self.uav_current_trajectory_point)
        self.uav_trajectory_point_pub.publish(temp_trajectory)

        # Manipulator joint trajectory message
        q1 = self.current_trajectory_point.positions[4+2]
        q2 = self.current_trajectory_point.positions[5+2]
        q3 = self.current_trajectory_point.positions[6+2]
        joint_trajectory = JointTrajectory()
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = [q1, q2, -q3]
        joint_trajectory_point.velocities = [0, 0, 0]
        joint_trajectory_point.accelerations = [0, 0, 0]
        joint_trajectory_point.effort = [0, 0, 0]
        joint_trajectory_point.time_from_start = rospy.Duration(0.001)

        joint_trajectory.points.append(joint_trajectory_point)
        joint_trajectory.header.frame_id = "world"
        joint_trajectory.joint_names.append("joint1")
        joint_trajectory.joint_names.append("joint2")
        joint_trajectory.joint_names.append("joint3")
        joint_trajectory.header.stamp = rospy.Time.now()
        self.manipulator_trajectory_pub.publish(joint_trajectory)

def jointTrajectoryPointToMultiDofJointTrajectoryPoint(joint):
    multi = MultiDOFJointTrajectoryPoint()

    transform = Transform()
    transform.translation.x = joint.positions[0]
    transform.translation.y = joint.positions[1]
    transform.translation.z = joint.positions[2]
    transform.rotation.z = math.sin(joint.positions[5]/2.0)
    transform.rotation.w = math.cos(joint.positions[5]/2.0)

    vel = Twist()
    vel.linear.x = joint.velocities[0]
    vel.linear.y = joint.velocities[1]
    vel.linear.z = joint.velocities[2]

    acc = Twist()
    acc.linear.x = joint.accelerations[0]
    acc.linear.y = joint.accelerations[1]
    acc.linear.z = joint.accelerations[2]

    multi.transforms.append(transform)
    multi.velocities.append(vel)
    multi.accelerations.append(acc)

    return multi

if __name__ == '__main__':

    rospy.init_node('joint_trajectory_to_neo_and_wp_manipulator_3r_reference_through_service')
    trajectory_to_ref = JointTrajectoryToUavAndWpManipulatorReference()
    trajectory_to_ref.run()

