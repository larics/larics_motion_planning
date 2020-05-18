# Ballistic airdrop paper

## Common stuff explained

### Running the UAV
Launch for the UAV with the mounted magnet. The magnet has dipole moment in z axis equal to 1. Set the height of the UAV to 1m so the integral part of the controller does not wind up while running other things. This also launches empty Gazebo world within itself.
``` bash
roslaunch mmuav_gazebo uav_attitude_position.launch mount_magnet:=true z:=1
```

### Running the planner
Launch the planner which contains several nodes to control the UAV
``` bash
roslaunch larics_motion_planning parabolic_airdrop.launch 
```
After running the planner you can send the UAV to certain point as shown in code. This will always be a straight line and will not account for collisions.
``` bash
rostopic pub /uav/go_to/reference geometry_msgs/Pose "position:
  x: 5.0
  y: 3.0
  z: 2.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0" 
```

Spawning the ball under the UAV will snap it to the body of the UAV because both bodies have magnets attached. It is an empty service that deletes the old ball model and spawns new one.
``` bash
rosservice call /uav/spawn_ball "{}" 
```

The launch also runs `joint_trajectory_to_multi_dof_trajectory_point.py` contained within `larics_motion_planning` that is responsible for dequeuing the trajectory points. It takes the 5th degree of freedom into account which is magnet gain.

This also runs the `octomap_server` and `generate_toppra_trajectory` nodes.

#### Parabolic airdrop service call explained

You can call it with:
``` bash
rosservice call /uav/parabolic_airdrop_trajectory "uav_pose:
  position: {x: -30.0, y: -100.0, z: 2.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
target_pose:
  position: {x: 2.946, y: 10.0, z: 7.4}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true
use_custom_parabola_params: false
custom_parabola_params: [0]
use_custom_psi_params: false
custom_psi_params: [0]"
```
`uav_pose` is the starting position of the UAV, its current pose.
`target_pose` is where what we are aiming for. Orientation does not matter.
`publish_path` and `publish_trajectory` control is the global planner node publishing or not.
`plan_trajectory` and `plan_path` control are these parts planned. If working in empty world the `plan_path: false` is a good option to skip unnecessary RRT planning.
`use_custom_parabola_params` is used together with `custom_parabola_params`. A vector of 5 parameters must be provided and it will override the search for the parabola and use this one instead. Useful for testing single parabolic airdrop.
`use_custom_psi_params` is used together with `custom_psi_params` vector. It expects three values `[psi_min, psi_increment, psi_max]`. This will override search space with default parameters within [0, 2pi] to user specified.


## Running the code for Camelia city
Spawn the Camelia city model with MBZIRC tower. This model is quite big so the collision is turned off. For the ball to fall into the building spawn building sparately with collision. Turning off the collision does not affect the UAV motion because plans will avoid buildings.
``` bash
roslaunch larics_gazebo_worlds spawn_camellia_city_mbzirc_building.launch
roslaunch larics_gazebo_worlds spawn_mbzirc_tower.launch z:=0.5

```

Next, launch the planner for the Camellia city. This will set it up with proper config file.
```bash
roslaunch larics_motion_planning camellia_city_parabolic_airdrop.launch
```

Call the service for parabolic airdrop. Custom parabola params do not have to be used.
``` bash
rosservice call /uav/parabolic_airdrop_trajectory "uav_pose:
  position: {x: -30.0, y: -100.0, z: 2.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
target_pose:
  position: {x: 2.946, y: 10.0, z: 7.4}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true
use_custom_parabola_params: false
custom_parabola_params: [0]
use_custom_psi_params: false
custom_psi_params: [0]"
```


## Running the code for office space

Spawn the office. Floor of the upper offices is around 1.05m.
``` bash
roslaunch larics_gazebo_worlds spawn_office.launch
```
Also, spawn a bucket into the biggest office. This can serve as aiming point. Make sure that scale for the bucket is 0.03125, this means the upper diameter of the bucket is 0.5m.
``` bash
roslaunch larics_gazebo_worlds spawn_bucket.launch x:=12.0 y:=21.0 z:=1.05
```

Run the planner for the office with the appropriate config.
``` bash
roslaunch larics_motion_planning office_parabolic_airdrop.launch
```

Call the service. This is an example in which we aim for the bucket. Note that z is bottom of the bucket. It is hard to find a parabola for the office, this one worked well as an example. Bucket height is 0.334m.
``` bash
rosservice call /uav/parabolic_airdrop_trajectory "uav_pose:
  position: {x: 35.0, y: 20.0, z: 1.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
target_pose:
  position: {x: 12.0, y: 21.0, z: 1.384}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
publish_path: true
publish_trajectory: true
plan_path: true
plan_trajectory: true
use_custom_parabola_params: false
custom_parabola_params: [0]
use_custom_psi_params: true
custom_psi_params: [45, 1, 45]" 
```