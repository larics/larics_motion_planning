# larics\_motion\_planning

rosbag record /uav/aerial_manipulator_control/end_effector/pose_output /uav/aerial_manipulator_control/end_effector/pose_stamped_ref_output /uav/aerial_manipulator_control/parameter_descriptions /uav/aerial_manipulator_control/parameter_updates /uav/aerial_manipulator_control/pose_output /uav/aerial_manipulator_control/pose_stamped_ref_output /uav/aerial_manipulator_control/state /uav/aerial_manipulator_control/transformation/world_end_effector /uav/aerial_manipulator_control/uav/pose_stamped_ref_output /uav/arm_joint_trajectory /uav/joint1_position_controller/command /uav/joint2_position_controller/command /uav/joint3_position_controller/command /uav/pose_ref /clock /uav/aerial_manipulator_control/end_effector/q_ref /uav/aerial_manipulator_control/force_torque_ref_input /uav/aerial_manipulator_control/pose_stamped_ref_input /uav/aerial_manipulator_control/trajectory_ref_input /uav/aerial_manipulator_control/uav/pose_stamped_ref_input /uav/control_mode /uav/end_effector/pose_ref /uav/force_sensor/force_torque_output /uav/joint1 /uav/joint2 /uav/joint3 /uav/joint_states /uav/odometry /model_uav/aerial_manipulator_control/end_effector/pose_output /model_uav/aerial_manipulator_control/end_effector/pose_stamped_ref_output /model_uav/aerial_manipulator_control/parameter_descriptions /model_uav/aerial_manipulator_control/parameter_updates /model_uav/aerial_manipulator_control/pose_output /model_uav/aerial_manipulator_control/pose_stamped_ref_output /model_uav/aerial_manipulator_control/state /model_uav/aerial_manipulator_control/transformation/world_end_effector /model_uav/aerial_manipulator_control/model_uav/pose_stamped_ref_output /model_uav/arm_joint_trajectory /model_uav/joint1_position_controller/command /model_uav/joint2_position_controller/command /model_uav/joint3_position_controller/command /model_uav/pose_ref /clock /model_uav/aerial_manipulator_control/end_effector/q_ref /model_uav/aerial_manipulator_control/force_torque_ref_input /model_uav/aerial_manipulator_control/pose_stamped_ref_input /model_uav/aerial_manipulator_control/trajectory_ref_input /model_uav/aerial_manipulator_control/model_uav/pose_stamped_ref_input /model_uav/control_mode /model_uav/end_effector/pose_ref /model_uav/force_sensor/force_torque_output /model_uav/joint1 /model_uav/joint2 /model_uav/joint3 /model_uav/joint_states /model_uav/odometry


## Just a reminder on how to run the code
```
roslaunch mmuav_gazebo uav_attitude_position.launch manipulator_type:="wp_manipulator" start_gazebo:=true

roslaunch larics_motion_planning global_planner.launch

rosrun topp_ros generate_toppra_trajectory.py __ns:=uav

rosrun octomap_server octomap_server_node greenhouse4096.binvox.bt _frame_id:="world"

roslaunch wp_manipulator planning_context.launch

rosrun aerial_manipulators_control wp_manipulator_control __ns:=uav
```

And for parabolic airdrop
```
roslaunch mmuav_gazebo uav_attitude_position.launch mount_magnet:=true z:=1

rosrun joy joy_node _autorepeat_rate:=30 __ns:=uav

rosrun mmuav_joy uav_manipulator_joy_command_node.py __ns:=uav

roslaunch larics_gazebo_worlds spawn_ball.launch

rosrun larics_motion_planning joint_trajectory_to_multi_dof_trajectory_point.py __ns:=uav
```

##Launch fileovi za staklenik
```
roslaunch mmuav_gazebo uav_attitude_position.launch manipulator_type:="wp_manipulator" start_gazebo:=false z:=1.0

roslaunch mmuav_gazebo uav_attitude_position.launch name:="model_uav" turn_off_all_collisions:="true" manipulator_type:="wp_manipulator" start_gazebo:=true z:=1.0 paused:="true" 

roslaunch wp_manipulator planning_context.launch

roslaunch larics_motion_planning arducopter_wp_manipulator.launch
```

##Launch fileovi za impedanciju
```
roslaunch mmuav_gazebo uav_attitude_position.launch manipulator_type:="wp_manipulator_3rx" manipulator_tool:="rod" start_gazebo:=false z:=0.2

roslaunch mmuav_gazebo uav_attitude_position.launch name:="model_uav" turn_off_all_collisions:="true" manipulator_type:="wp_manipulator_3rx" manipulator_tool:="rod" start_gazebo:=true z:=0.2 paused:="true" 
```

Za spawnati model treba iskljuciti sve collisione, pokrenuti launch file za impedanciju i cekati da letjelica padne i upali motore. Onda pauzirati simulaciju, kliknuti na ono lijevo od "steps" da se izvrti jos koji korak da se uvjeris da se motori vrte. Onda staviti pose z letjelice na 0.5 i playati simulaciju dalje.
``` bash
roslaunch gazebo_ros empty_world.launch use_sim_time:=true world_name:=$(rospack find larics_gazebo_worlds)/worlds/wall_with_circles_inspection.world

# model uav
roslaunch wp_manipulator_3rx planning_context.launch
roslaunch impedance_control impedance_control_asap_3rx.launch start_gazebo:=false turn_off_all_collisions:="false" name:="model_uav"
rosrun impedance_control get_uav_airborne.sh model_uav

roslaunch wp_manipulator_3rx planning_context.launch
roslaunch impedance_control impedance_control_asap_3rx.launch start_gazebo:=false 
rosrun impedance_control get_uav_airborne.sh

roslaunch larics_motion_planning arducopter_wp_manipulator_3rx.launch
```

./binvox -e -bb -20.0 -35.0 0.0 20 4.0 2.0 ../models/greenhouse/greenhouse.stl
[-20, -35.9957, -0.0211357, 1] - [20, 4.00428, 4.44961, 1]

binvox2bt --bb -20.0 -35.0 0.0 20 4.0 2.0 greenhouse.binvox

rostopic pub -1 /uav/reference_tracker/set_current_trajectory_point trajectory_msgs/JointTrajectoryPoint "
positions: [1.37, 2, 1.8, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
velocities: [0]
accelerations: [0]
effort: [0]
time_from_start: {secs: 0, nsecs: 0}" 

3.3852, 2.2, 2.0, 0.0, 0.0, 0.0, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 1


rosservice call /planner/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0]
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


Multirobot
[0,0,1,0,0,0,0.787, 0.787, 0.787, -1.57, 0.787,0,0,2,0,0,0,0.787, 0.787, 0.787, -1.57, 0.787]


rosservice call /planner/visualize_state "state:
  layout:
    dim:
    - label: ''
      size: 0
      stride: 0
    data_offset: 0
  data: [0,0,1,0,0,0,0.787, 0.787, 0.787, -1.57, 0.787,0,0,2,0,0,0,0.787, 0.787, 0.787, -1.57, 0.787]" 


rostopic pub /planner/reference_tracker/joint_trajectory_point trajectory_msgs/JointTrajectoryPoint "
positions: [0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0]
velocities: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
accelerations: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
effort: [0]
time_from_start: {secs: 0, nsecs: 0}"



rosservice call /planner/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0,0,1,0,0,0,0.787, 0.787, 0.787, -1.57, 0.787,1.0,0,2,0,0,0,0.787, 0.787, 0.787, -1.57, 0.787]
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

rosservice call /planner/multiple_manipulators_model_correction_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0, 0, 0.8, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0]
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


rosservice call /planner/multiple_manipulators_model_correction_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0,0,1,0,0,0,0.785398, 0.785398, 0.785398, -1.57, 0.785398,1.045,0,1,0,0,3.141592654,0.785398, 0.785398, 0.785398, -1.5707963, 0.785398]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [-1.0,0,1,0,0,0,0.785398, 0.785398, 0.785398, -1.5707963, 0.785398,0.045,0,1,0,0,3.141592654,0.785398, 0.785398, 0.785398, -1.5707963, 0.785398]
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

rosservice call /planner/multiple_manipulators_model_correction_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [-1.0,0,1,0,0,0,0.787, 0.787, 0.787, -1.57, 0.787,0.045,0,1,0,0,3.141592654,0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0,0,1,0,0,0,0.787, 0.787, 0.787, -1.57, 0.787,1.045,0,1,0,0,3.141592654,0.787, 0.787, 0.787, -1.57, 0.787]
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


rosservice call /model_planner/execute_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0]
    velocities: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    accelerations: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0]
    velocities: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    accelerations: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: false
plan_trajectory: false
override_dynamic_constraints: false
velocity_constraints: [0]
acceleration_constraints: [0]"



rosservice call /uav/multi_dof_trajectory "uav_pose:
  position: {x: -30.0, y: -100.0, z: 2.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
target_pose:
  position: {x: 2.946, y: 20.0, z: 6.9}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [-30, -100, 2, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [-3, 20, 7, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true
override_dynamic_constraints: false
velocity_constraints: [0]
acceleration_constraints: [0]"



# Asap manipulator
rosservice call /red/model_correction_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0, 0, 1, 0, 0, 0, 0.0, -0.5, 1, -0.5, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [2, 0, 1, 0, 0, 0, 0.0, -0.5, 1, -0.5, 0, 0, 0, 0, 0, 0, 1]
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

rosservice call /model_uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0, 0, 1, 0, 0, 0, 0.0, -0.5, 1, -0.5, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [2, 0, 1, 0, 0, 0, 0.0, -0.5, 1, -0.5, 0, 0, 0, 0, 0, 0, 1]
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

rosservice call /red/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0, 0, 1, 0, 0, 0, 0.0, 0.0, 0, 0.0, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [2, 0, 1, 0, 0, 0, 0.0, -0.5, 1, -0.5, 0, 0, 0, 0, 0, 0, 1]
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


# Object trajectory stuff
rosservice call /object/multiple_manipulators_object_trajectory "waypoints:
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
  - positions: [3, 0, 1, 0, 0, 0]
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


# Object trajectory stuff
rosservice call /object/multiple_manipulators_object_trajectory "waypoints:
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


rosservice call /object/multiple_manipulators_object_trajectory "waypoints:
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




rostopic pub -1 /object/go_to/reference/multiarray std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [1, 0, 1, 0, 0, 0]"


rostopic pub -1 /object/generate_object_trajectory/config std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0, 0, 0, 1, 2.0, 0.2]"