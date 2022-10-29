#!/bin/bash
m=$1

# Go to end point first. In the for loop, we will go to the initial point and
# then execute trajectory, and it will last for the same amount of time in each
# pass.

# Now we can start executing the trajectory


for ((i=1;i<=m;i++));do
  echo "Starting pass " $i

  echo "Calling trajectory planning service with path planning on."

rosservice call /uav/model_trajectory_service_modifier/model_correction_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0,-7.0,1,0,0,0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.95,7.2,3.325,0,0,0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [7.95,7.2,3.325,0,0,0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: true
plan_trajectory: true
override_dynamic_constraints: false
velocity_constraints: [0]
acceleration_constraints: [0]"

# Trajectory start indicator. Probably not the real start, but close enough
# to approximate in analysis, or offset it by some amount.
echo "Publish trajectory start."
rostopic pub --once /trajectory_start std_msgs/Int32 "data: 1"

# The trajectory should last about 60-70s
sleep 90s


echo "Going to trajectory start point."
rostopic pub --once /uav/go_to/full_state_ref std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0,-7.0,1,0,0,0, 0.787, 0.787, 0.787, -1.57, 0.787]"
rostopic pub --once /model_uav/go_to/full_state_ref std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0,-7.0,1,0,0,0, 0.787, 0.787, 0.787, -1.57, 0.787]"

sleep 35s


done