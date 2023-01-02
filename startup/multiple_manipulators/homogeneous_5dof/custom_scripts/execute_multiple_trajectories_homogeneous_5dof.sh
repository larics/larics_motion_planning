#!/bin/bash

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
    echo "Starting to execute $n straight trajectories"
    for ((i=1; i<=$n; i++))
    do
        echo " "
        echo "Planning forward trajectory $i of $n"

rosservice call /planner/multiple_manipulators_model_correction_trajectory "waypoints:
    header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
    joint_names: ['']
    points:
    - positions: [-0.55,0.018,1,0,0,0,0.785398, 0.785398, 0.557, -0.861, 0.304, 0.553,-0.018,1,0,0,3.141592654,0.785398, 0.785398, 0.557, -0.861, 0.304]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
    - positions: [1.45,0.018,1,0,0,0,0.785398, 0.785398, 0.557, -0.861, 0.304, 2.553,-0.018,1,0,0,3.141592654,0.785398, 0.785398, 0.557, -0.861, 0.304]
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

        echo "Executing forward trajectory $i of $n"
        echo "Sleeping 15s"
        sleep 15s

        echo "Planning backward trajectory $i of $n"

rosservice call /planner/multiple_manipulators_model_correction_trajectory "waypoints:
    header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
    joint_names: ['']
    points:
    - positions: [1.45,0.018,1,0,0,0,0.785398, 0.785398, 0.557, -0.861, 0.304, 2.553,-0.018,1,0,0,3.141592654,0.785398, 0.785398, 0.557, -0.861, 0.304]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
    - positions: [-0.55,0.018,1,0,0,0,0.785398, 0.785398, 0.557, -0.861, 0.304, 0.553,-0.018,1,0,0,3.141592654,0.785398, 0.785398, 0.557, -0.861, 0.304]
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

        echo "Executing backward trajectory $i of $n"
        echo "Sleeping 15s"
        sleep 15s
    done

elif [[ "$trajectory_type" == "square" ]]; then
    echo "Starting to execute $n square trajectories"
    
    for ((i=1; i<=$n; i++))
    do
        echo " "
        echo "Planning square trajectory $i of $n"

rosservice call /planner/multiple_manipulators_model_correction_trajectory "waypoints:
    header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
    joint_names: ['']
    points:
    - positions: [-0.55,0.018,1,0,0,0,0.785398, 0.785398, 0.557, -0.861, 0.304, 0.553,-0.018,1,0,0,3.141592654,0.785398, 0.785398, 0.557, -0.861, 0.304]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
    - positions: [1.45,0.018,1,0,0,0,0.785398, 0.785398, 0.557, -0.861, 0.304, 2.553,-0.018,1,0,0,3.141592654,0.785398, 0.785398, 0.557, -0.861, 0.304]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
    - positions: [1.45,2.018,1,0,0,0,0.785398, 0.785398, 0.557, -0.861, 0.304, 2.553,1.982,1,0,0,3.141592654,0.785398, 0.785398, 0.557, -0.861, 0.304]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
    - positions: [-0.55,2.018,1,0,0,0,0.785398, 0.785398, 0.557, -0.861, 0.304, 0.553,1.982,1,0,0,3.141592654,0.785398, 0.785398, 0.557, -0.861, 0.304]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
    - positions: [-0.55,0.018,1,0,0,0,0.785398, 0.785398, 0.557, -0.861, 0.304, 0.553,-0.018,1,0,0,3.141592654,0.785398, 0.785398, 0.557, -0.861, 0.304]
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
        
        echo "Executing backward trajectory $i of $n"
        echo "Sleeping 25s"
        sleep 25s
    done
    

else
    echo "No such trajectory type. Your input is $trajectory_type"
fi



# Waypoints for trajectories
# Old waypoints have one joint at -1.57 which is the limit.
# The suspicion is that this does not allow the inverse kinematics to
# correct properly. These waypoints are:
# [0,0,1,0,0,0,0.785398, 0.785398, 0.785398, -1.5707963, 0.785398,1.045,0,1,0,0,3.141592654,0.785398, 0.785398, 0.785398, -1.5707963, 0.785398]
# [-1.0,0,1,0,0,0,0.785398, 0.785398, 0.785398, -1.5707963, 0.785398,0.045,0,1,0,0,3.141592654,0.785398, 0.785398, 0.785398, -1.5707963, 0.785398]

# New set of waypoints keeps the end-effector orientation the same as above,
# but joints are a bit further away from their limits:
# [-0.55,0.018,1,0,0,0,0.785398, 0.785398, 0.557, -0.861, 0.304, 0.553,-0.018,1,0,0,3.141592654,0.785398, 0.785398, 0.557, -0.861, 0.304]
# [1.45,0.018,1,0,0,0,0.785398, 0.785398, 0.557, -0.861, 0.304, 2.553,-0.018,1,0,0,3.141592654,0.785398, 0.785398, 0.557, -0.861, 0.304]