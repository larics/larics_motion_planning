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


rosservice call /uav/model_correction_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [1.37, 2.2, 2.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [5.0, 0, 1.5, 0.0, 0.0, 3.14159, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [3.3, 0, 1.5, 0.0, 0.0, 3.14159, -0.833, 0.517, -0.853, -0.1918, 0, 0.4618, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [3.1, 0, 1.9867, 0.0, 0.0, 3.14159, -0.833, 0.517, -0.853, -0.1918, 0, 0.4618, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [2.918, 0, 2.425, 0.0, 0.0, 3.14159, -0.833, 0.517, -0.853, -0.1918, 0, 0.4618, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true"

rosservice call /uav/model_correction_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [1.37, 2.2, 2.0, 0.0, 0.0, 0.0, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [3.3852, 2.2, 2.0, 0.0, 0.0, 0.0, -0.443, 0.907, -0.463, 0.5, 0, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true"


rosservice call /uav/model_correction_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [1.37, 2.2, 2.0, 0.0, 0.0, 0.0, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [14.1061, 14.4936, 10.8134, 0.0, 0.0, 0.724, -0.443, 0.907, -0.463, 0.35, 0.35, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true"


rosservice call /uav/model_correction_trajectory "waypoints:
rosservice call /uav/multi_dof_trajectory "waypoints:
rosservice call /uav/model_correction_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [1.37, 2.2, 2.0, 0.0, 0.0, 0.0, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [1.37, 2, 10.8, 0.0, 0.0, 0.724, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [10.36, 11.18, 10.8134, 0.0, 0.0, 0.724, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [14.1061, 14.4936, 10.8134, 0.0, 0.0, 0.724, -0.443, 0.907, -0.463, 0.35, 0.35, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true"

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [14.1061, 14.4936, 10.8134, 0.0, 0.0, 0.724, -0.443, 0.907, -0.463, 0.35, 0.35, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [1.37, 2.2, 2.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}  
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true"

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [1.37, 2.2, 2.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [1.37, 3.3, 2.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [1.37, 2.2, 2.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true"

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [1.37, 2, 1.8, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [2, 2, 1.3, 0.0, 0.0, 0.787, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.74, 3.74, 1.3, 0.0, 0.0, 0.787, -0.443, 0.907, -0.463, 0.35, 0.35, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true"

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [6.78, 3.78, 1.3, 0.0, 0.0, 0.787, -0.443, 0.907, -0.463, 1, 1, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [1.37, 2, 1.8, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true"

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [6.73, 3.73, 1.3, 0.0, 0.0, 0.787, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.75, 3.75, 1.3, 0.0, 0.0, 0.787, -0.443, 0.907, -0.463, 0.707, 0.707, 0, 0, 0, 0, 0.5]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true"

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [1.37, 2, 1.8, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.7, 3.7, 1.3, 0.0, 0.0, 0.787, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.7, 3.7, 1.3, 0.0, 0.0, 0.787, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.72, 3.72, 1.3, 0.0, 0.0, 0.787, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.74, 3.74, 1.3, 0.0, 0.0, 0.787, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.76, 3.76, 1.3, 0.0, 0.0, 0.787, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.78, 3.78, 1.3, 0.0, 0.0, 0.787, -0.443, 0.907, -0.463, 0, 0, 0, 0, 0, 0, 1]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true"


rosservice call /uav/model_correction_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0, 0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.11, -14.96, 1.1, 0.0, 0.0, 0.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true"

rosservice call /uav/model_correction_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [6.11, -14.96, 1.1, 0.0, 0.0, 0.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.11, -14.10, 1.1, 0.0, 0.0, 0.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.11, -13.20, 1.1, 0.0, 0.0, 0.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.11, -12.30, 1.1, 0.0, 0.0, 0.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true" 

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [4.11, -4.20, 1.1, 0.0, 0.0, 0.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.11, -4.20, 1.1, 0.0, 0.0, 0.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: false
plan_trajectory: true" 

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0.0, 0.0, 1.1, 0.0, 0.0, 3.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.0, 0.0, 1.1, 0.0, 0.0, -3.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.0, 0.0, 1.1, 0.0, 0.0, 0.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.0, 0.0, 1.1, 0.0, 0.0, 3.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.0, 0.0, 1.1, 0.0, 0.0, -3.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.0, 0.0, 1.1, 0.0, 0.0, 0.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.0, 0.0, 1.1, 0.0, 0.0, 3.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.0, 0.0, 1.1, 0.0, 0.0, -3.0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true" 

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [6.12, -8.27, 1.0, 0.0, 0.0, 1.57, 0.787, 0.787, 0.787, -1.57, 0.787, -3.0, 0.2, 0.6, 2.3]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [4.61, -4.30, 1.0, 0.0, 0.0, 1.57, 0.787, 0.787, 0.787, -1.57, 0.787, 3.0, 0.5, 0.1, 3.0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: true
plan_trajectory: true" 


rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [4.27, -8.4, 0.35, 0.0, 0.0, 1.57, 0.787, 0.787, 1.0, 0.0, 0.0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [6.12, -8.27, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: true
plan_trajectory: true" 



rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [2.0, 0.0, 1.98, 0.0, 0.0, 3.14, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [1.6, 0.0, 1.98, 0.0, 0.0, 3.14, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: false
plan_trajectory: true" 

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [-3.4, 0, 1.98, 0, 0, 0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [-3.0, 0, 1.98, 0, 0, 0, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: false
plan_trajectory: true" 

Stanja za pipe insertion
Ispred cijevi: [-3.4, 0, 1.98, 0, 0, 0, 0.787, 0.787, 0.787, -1.57, 0.787]
Skroz u cijevi: [-3.0, 0, 1.98, 0, 0, 0, 0.787, 0.787, 0.787, -1.57, 0.787]

Za roll pitch kompenzaciju
rosservice call /simulate_arducopter "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [2.0, 0.0, 1.98, 0.0, 0.0, 3.14, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [1.6, 0.0, 1.98, 0.0, 0.0, 3.14, 0.787, 0.787, 0.787, -1.57, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: false
plan_trajectory: true" 

3R manipulator
rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [-3.4, 0, 1.98, 0, 0, 0, 1.57, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [-3.0, 0, 1.98, 0, 0, 0, 1.57, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: false
plan_trajectory: true" 

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [-3.4, 0, 1.98, 0, 0, 0, 0, -0.787, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [-3.0, 0, 1.98, 0, 0, 0, 0, -0.787, 0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: false
plan_trajectory: true" 


rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [-0.5, 0, 1.5, 0, 0, 0, 0, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.5, 0, 1.5, 0, 0, 0, 0, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: false
plan_trajectory: true" 

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [-0.5, -0.277042244086, 1.45389283102, 0, 0, 0, 0, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.435133965528, -0.277042244086, 1.45389283102, 0, 0, 0, 0, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: false
plan_trajectory: true"

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [-1.33663805195, 0.431922721808, 1.45422649549, 0, 0, -1.57, 0, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [-1.35163195589, -0.4255168737, 1.45422649549, 0, 0, -1.39626, 0, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [-0.386846424239, -1.28840134141, 1.45422649549, 0, 0, 0, 0, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.370522995507, -1.4962575204, 1.45422649549, 0, 0, 0.411019597, 0, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.52, -0.62168, 1.45422649549, 0, 0, 0.811019597, 0, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.869235, -0.2540755, 1.45422649549, 0, 0, 0.811019597, 0, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [1.21846992967, 0.113532661392, 1.45422649549, 0, 0, 0.811019597, 0, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: false
plan_trajectory: true" 

  - positions: [0.84, -0.284848, 1.45422649549, 0, 0, 0.811019597, 0, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}


// Pod kutem
rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [-0.5, -0.224650453353, 1.0, 0, 0, 0, 0.5, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [-0.2, -0.224650453353, 1.0, 0, 0, 0, 0.5, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.08303, -0.224650453353, 1.1, 0, 0, 0, 0.5, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.3576, -0.224650453353, 1.25, 0, 0, 0, 0.5, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.54065, -0.224650453353, 1.35, 0, 0, 0, 0.5, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.729145548351, -0.224650453353, 1.45297384115, 0, 0, 0, 0.5, 0.787, -0.787]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: false
plan_path: false
plan_trajectory: true"

rosservice call /husky/cartesian_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  poses:
  - header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
    pose:
      position: {x: 4.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
  - header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
    pose:
      position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true" 


0.8, 0.8, 0.5, 100, 100, 0.3, 1.0, 1.0

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0.0, 1.5, 1.6, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [7.0, 1.5, 1.6, 0, 0, 0, 7.0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true"


rosservice call /red/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0.0, 0.0, 1.0, 0.0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [10.0, 0.0, 1.0, 0.0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true"

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0.0, 0.0, 1.0, 0.0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [10.0, 0.0, 1.0, 0.0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true
override_dynamic_constraints: true
velocity_constraints: [0]
acceleration_constraints: [0]"

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [-30.0, -100.0, 2.0, 0.0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [5.0, 20.0, 10.0, 0.0]
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

City
rosservice call /uav/parabolic_airdrop_trajectory "uav_pose:
  position: {x: -30.0, y: -100.0, z: 2.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
target_pose:
  position: {x: 2.946, y: 10.0, z: 6.9}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true" 


rosservice call /uav/parabolic_airdrop_trajectory "uav_pose:
  position: {x: 8, y: 8, z: 1.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
target_pose:
  position: {x: 0.06, y: -3.71, z: 1.75}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true
use_custom_parabola_params: true
custom_parabola_params: [2.5,0.5,0.5585,1.0213,3.14]" 

rosservice call /uav/parabolic_airdrop_trajectory "uav_pose:
  position: {x: 4.85, y: 9, z: 1.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
target_pose:
  position: {x: 3.47, y: -3.055, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true
use_custom_parabola_params: true
custom_parabola_params: [1,2,0.1745,0.64652,-2.3561]" 

rosservice call /uav/parabolic_airdrop_trajectory "uav_pose:
  position: {x: 5, y: -8, z: 1.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
target_pose:
  position: {x: 3.47, y: 8.923, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true
use_custom_parabola_params: true
custom_parabola_params: [1,2,0.1745,0.64652, 2.3561]" 

Kanta u officeu
rosservice call /uav/parabolic_airdrop_trajectory "uav_pose:
  position: {x: 5, y: -8, z: 1.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
target_pose:
  position: {x: 0.75, y: -1.75, z: 0.33}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true
use_custom_parabola_params: true
custom_parabola_params: [1.75,1.75,0.1745,1.0842,2.3561]" 

rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [35, 20, 1, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [3.25, 1.5, 2, 0]
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
acceleration_constraints: [0] 
use_custom_psi_params: true
custom_psi_params: [0]"

rosservice call /uav/parabolic_airdrop_trajectory "uav_pose:
  position: {x: 35.0, y: 20.0, z: 1.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
target_pose:
  position: {x: 6.12, y: 3.94, z: 1.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true
use_custom_parabola_params: false
custom_parabola_params: [0]
use_custom_psi_params: true
custom_psi_params: [180, 1, 180]" 


rosservice call /uav/parabolic_airdrop_trajectory "uav_pose:
  position: {x: 10.0, y: 20.0, z: 10.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
target_pose:
  position: {x: 2.946, y: 10.0, z: 7.4}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true" 


## For uav and husky with manipulators
rosservice call /uav/multi_dof_trajectory "waypoints:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  joint_names: ['']
  points:
  - positions: [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -3, 0, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -3, 0, 0, 0, 0, 0, 0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0]
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

rosservice call /euroc3/parabolic_airdrop_trajectory "uav_pose:
  position: {x: 0.0, y: 1.25, z: 1.7}
  orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}
target_pose:
  position: {x: 0, y: -3.25, z: 0.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true
use_custom_parabola_params: false
custom_parabola_params: [0.0]
use_custom_psi_params: true
custom_psi_params: [-90, 1, -90]" 

rosservice call /euroc3/parabolic_airdrop_trajectory "uav_pose:
  position: {x: 0.075, y: 1.25, z: 1.7}
  orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}
target_pose:
  position: {x: 0.075, y: -3.25, z: 0.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: false
publish_trajectory: true
plan_path: false
plan_trajectory: true
use_custom_parabola_params: false
custom_parabola_params: [0.0]
use_custom_psi_params: true
custom_psi_params: [-90, 1, -90]" 

rosservice call /euroc3/parabolic_airdrop_trajectory "uav_pose:
  position: {x: 0.075, y: 1.25, z: 1.7}
  orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}
target_pose:
  position: {x: 0.075, y: -3.25, z: 0.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true
use_custom_parabola_params: false
custom_parabola_params: [0.0]
use_custom_psi_params: true
custom_psi_params: [-112, 2, -80]" 

rosservice call /red/parabolic_airdrop_trajectory "uav_pose:
  position: {x: -3.0, y: 48.75, z: 5.0}
  orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}
target_pose:
  position: {x: -3.0, y: 10.0, z: 2.8}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true
use_custom_parabola_params: false
custom_parabola_params: [0.0]
use_custom_psi_params: true
custom_psi_params: [-14, 2, -14]" 

rostopic pub -r 10 /red/carrot/trajectory trajectory_msgs/MultiDOFJointTrajectoryPoint "transforms:
- translation:
    x: -4.13
    y: 45.85
    z: 5.7
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
velocities:
- linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
accelerations:
- linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
time_from_start:
  secs: 0
  nsecs: 0" 


rosservice call /red/parabolic_airdrop_trajectory "uav_pose:
  position: {x: -3.9, y: 45.8, z: 5.0}
  orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}
target_pose:
  position: {x: -2.3, y: 9.4, z: 2.4}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true
use_custom_parabola_params: false
custom_parabola_params: [0.0]
use_custom_psi_params: true
custom_psi_params: [-30, 2, -30]" 