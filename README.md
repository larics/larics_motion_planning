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

./binvox -e -bb -20.0 -35.0 0.0 20 4.0 2.0 ../models/greenhouse/greenhouse.stl
[-20, -35.9957, -0.0211357, 1] - [20, 4.00428, 4.44961, 1]

binvox2bt --bb -20.0 -35.0 0.0 20 4.0 2.0 greenhouse.binvox