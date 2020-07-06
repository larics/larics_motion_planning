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
roslaunch mmuav_gazebo uav_attitude_position.launch mount_magnet:=true z:=1

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
    x: -3.0
    y: 48.75
    z: 5.0
  rotation:
    x: 0.0
    y: 0.0
    z: -0.707
    w: 0.707
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