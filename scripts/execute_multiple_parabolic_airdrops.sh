#!/bin/bash
m=$1
for ((i=1;i<=m;i++));do
  echo "Doing parabolic airdrop" $i

  echo "Sending the uav to start point."
  rostopic pub -1 /uav/go_to/reference geometry_msgs/Pose "position:
  x: -9.0
  y: 0.0
  z: 4.6  
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0" 

  echo "Sleeping for 10s."
  sleep 10s

  echo "Spawning ball"
  rosservice call /uav/spawn_ball "{}" 

  echo "Sleeping for 10s."
  sleep 10s

  echo "Call parabolic trajectory service"
  rosservice call /uav/parabolic_airdrop_trajectory "uav_pose:
  position: {x: -9.0, y: 0.0, z: 4.6}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
target_pose:
  position: {x: 0.0, y: 0.0, z: 0.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true" 


  echo "Sleeping 25s."
  sleep 25s


done