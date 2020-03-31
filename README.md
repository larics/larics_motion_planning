# larics\_motion\_planning

## Just a reminder on how to run the code
```
roslaunch mmuav_gazebo uav_attitude_position.launch manipulator_type:="wp_manipulator" start_gazebo:=true

roslaunch larics_motion_planning global_planner.launch

rosrun topp_ros generate_toppra_trajectory __ns:=uav

rosrun octomap_server octomap_server_node greenhouse4096.binvox.bt _frame_id:="world"

roslaunch wp_manipulator planning_context.launch

rosrun aerial_manipulators_control wp_manipulator_control __ns:=uav
```

And for parabolic airdrop
```
roslaunch mmuav_gazebo uav_attitude_position.launch mount_magnet:=true

rosrun joy joy_node _autorepeat_rate:=30 __ns:=uav

rosrun mmuav_joy uav_manipulator_joy_command_node.py __ns:=uav

roslaunch larics_gazebo_worlds spawn_ball.launch

rosrun larics_motion_planning joint_trajectory_to_multi_dof_trajectory_point.py __ns:=uav
```

./binvox -e -bb -20.0 -35.0 0.0 20 4.0 2.0 ../models/greenhouse/greenhouse.stl
[-20, -35.9957, -0.0211357, 1] - [20, 4.00428, 4.44961, 1]

binvox2bt --bb -20.0 -35.0 0.0 20 4.0 2.0 greenhouse.binvox

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