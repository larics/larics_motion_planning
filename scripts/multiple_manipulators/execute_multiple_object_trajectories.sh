#!/bin/bash

# Usage: ./execute_multiple_object_trajectories.sh -n 10 -t straight

n=10
trajectory_type="straight"

while getopts n:t: flag
do
    case "${flag}" in
        n) n=${OPTARG};;
        t) trajectory_type=${OPTARG};;
    esac
done
echo "Number of trajectories: $n";
echo "Trajectory type: $trajectory_type";


if [[ "$trajectory_type" == "straight" ]]; then
    echo "Moving the system to the first point."
rostopic pub -1 /object/go_to/reference/multiarray std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0, 0, 1, 0, 0, 0]"
    
    sleep 10s
    read -n 1 -r -s -p $'Start the bag and press any key to continue....\n'

    echo "Starting to execute $n straight trajectories"
    for ((i=1; i<=$n; i++))
    do
        echo " "
        echo "Planning trajectory $i of $n"

rosservice call /object/plan_full_state_trajectory "waypoints:
    header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
    joint_names: ['']
    points:
    - positions: [0, 0, 1, 0, 0, 0]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
    - positions: [2, 0, 1, 0, 0, 0]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
    - positions: [0, 0, 1, 0, 0, 0]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: false
plan_trajectory: false
override_dynamic_constraints: false
velocity_constraints: [0]
acceleration_constraints: [0]"

        echo "Executing trajectory $i of $n"
        echo "Sleeping 15s"
        sleep 15s
    done
fi


# Square trajectories
if [[ "$trajectory_type" == "square" ]]; then
    echo "Moving the system to the first point."
rostopic pub -1 /object/go_to/reference/multiarray std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0, 0, 1, 0, 0, 0]"
    
    sleep 10s
    read -n 1 -r -s -p $'Start the bag and press any key to continue....\n'

    echo "Starting to execute $n square trajectories"

    for ((i=1; i<=$n; i++))
    do
        echo " "
        echo "Planning trajectory $i of $n"

rosservice call /object/plan_full_state_trajectory "waypoints:
    header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
    joint_names: ['']
    points:
    - positions: [0, 0, 1, 0, 0, 0]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
    - positions: [2, 0, 1, 0, 0, 0]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
    - positions: [2, 2, 1, 0, 0, 0]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
    - positions: [0, 2, 1, 0, 0, 0]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
    - positions: [0, 0, 1, 0, 0, 0]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: false
plan_trajectory: false
override_dynamic_constraints: false
velocity_constraints: [0]
acceleration_constraints: [0]"

        echo "Executing trajectory $i of $n"
        echo "Sleeping 25s"
        sleep 25s
    done
fi



# Circle trajectories
if [[ "$trajectory_type" == "circle" ]]; then
    echo "Moving the system to the first point."
rostopic pub -1 /object/go_to/reference/multiarray std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [2, 0, 1, 0, 0, 0]"
    
    sleep 10s
    read -n 1 -r -s -p $'Start the bag and press any key to continue....\n'

    echo "Starting to execute $n circular trajectories"

    for ((i=1; i<=$n; i++))
    do
        echo " "
        echo "Planning trajectory $i of $n"

rostopic pub -1 /object/generate_object_trajectory/config std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0, 0, 0, 1, 2.0, 0.2]"
        
        echo "Executing trajectory $i of $n"
        echo "Sleeping 90s"
        sleep 90s
    done
fi


# Storage racking environment
if [[ "$trajectory_type" == "warehouse" ]]; then
    echo "Moving the system to the first point."
rostopic pub -1 /object/go_to/reference/multiarray std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [-1.3, 7, 1, 0, 0, 0]"
    
    sleep 10s
    echo "Spawn the storage racking and start the octomap server!"
    read -n 1 -r -s -p $'Start the bag and press any key to continue....\n'

    echo "Starting to execute $n circular trajectories"

    for ((i=1; i<=$n; i++))
    do
        echo " "
        echo "Planning forward trajectory $i of $n"

rosservice call /object/plan_full_state_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [-1.3, 7, 1, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [-1.3, -8, 1, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: true
plan_trajectory: false
override_dynamic_constraints: false
velocity_constraints: [0]
acceleration_constraints: [0]"
        
        echo "Executing forward trajectory $i of $n"
        echo "Sleeping 30s"
        sleep 30s

        echo "Planning backward trajectory $i of $n"

rosservice call /object/plan_full_state_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [-1.3, -8, 1, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [-1.3, 7, 1, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: true
plan_trajectory: false
override_dynamic_constraints: false
velocity_constraints: [0]
acceleration_constraints: [0]"
        
        echo "Executing backward trajectory $i of $n"
        echo "Sleeping 30s"
        sleep 30s

    done
fi