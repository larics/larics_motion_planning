#!/bin/bash

if [[ -z "${UAV_NAMESPACE}" ]]
then 
  echo "Variable UAV_NAMESPACE not set."
  exit
fi;

if [[ -z "${MODEL_UAV_NAMESPACE}" ]]
then 
  echo "Variable MODEL_UAV_NAMESPACE not set."
  exit
fi;

# Echo topic to get carrot position
POS_X=$(rostopic echo -n 1 /$UAV_NAMESPACE/carrot/pose/pose/position/x | head -n 1)
POS_Y=$(rostopic echo -n 1 /$UAV_NAMESPACE/carrot/pose/pose/position/y | head -n 1)
POS_Z=$(rostopic echo -n 1 /$UAV_NAMESPACE/carrot/pose/pose/position/z | head -n 1)
ORI_Z=$(rostopic echo -n 1 /$UAV_NAMESPACE/carrot/pose/pose/orientation/z | head -n 1)
ORI_W=$(rostopic echo -n 1 /$UAV_NAMESPACE/carrot/pose/pose/orientation/w | head -n 1)

# Calculate yaw based on quaternion orientation
YAW=`awk -v z=$ORI_Z -v w=$ORI_W 'BEGIN {printf "%f\n", atan2(2.0*w*z, 1.0-2.0*z*z)}'`

# Go with model to the first point
rostopic pub -1 /$MODEL_UAV_NAMESPACE/go_to/reference/joint_trajectory_point trajectory_msgs/JointTrajectoryPoint "
positions: [$POS_X, $POS_Y, $POS_Z, 0, 0, $YAW, 0.0, -0.5, 1, -0.5, 0, 0, 0, 0, 0, 0, 1]
velocities: [0]
accelerations: [0]
effort: [0]
time_from_start: {secs: 0, nsecs: 0}"