#!/bin/bash
m=$1

# Go to end point first. In the for loop, we will go to the initial point and
# then execute trajectory, and it will last for the same amount of time in each
# pass.
echo "Going to the end point with model and uav."
rostopic pub --once /uav/go_to/full_state_ref std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [5,0,2,0,0,0,0.787, 0.787, 0.787, -1.57, 0.787]"

rostopic pub --once /model_uav/go_to/full_state_ref std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [5,0,2,0,0,0,0.787, 0.787, 0.787, -1.57, 0.787]"

# Make sure we get to the end point.
echo "Sleeping for 15s."
sleep 15s


for ((i=1;i<=m;i++));do
  echo "Starting pass " $i

  echo "Sending model and uav to the initial point."
rostopic pub --once /model_uav/go_to/full_state_ref std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0,0,2,0,0,0,0.787, 0.787, 0.787, -1.57, 0.787]"

rostopic pub --once /uav/go_to/full_state_ref std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0,0,2,0,0,0,0.787, 0.787, 0.787, -1.57, 0.787]"

  echo "Sleeping for 10s."
  sleep 10s

  echo "Calling model corrected trajectory service."
rosservice call /uav/model_correction_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0,0,2,0,0,0,0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [5,0,2,0,0,0,0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true
override_dynamic_constraints: false
velocity_constraints: [0]
acceleration_constraints: [0]"

  # Trajectory start indicator. Probably not the real start, but close enough
  # to approximate in analysis, or offset it by some amount.
  echo "Publish trajectory start."
  rostopic pub --once /trajectory_start std_msgs/Int32 "data: 1"

  echo "Sleeping for 15."
  sleep 15s


done