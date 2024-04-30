## Gazebo, D435, and OctoMap

#### Launch Sim arm and Sim d435
```bash
roslaunch kortex_gazebo spawn_kortex_robot_d435.launch start_rviz:=false arm:=gen3_lite gripper:=gen3_lite_2f sim_d435:=true
```

#### Launch Sim arm and Real d435
```bash
roslaunch kortex_gazebo spawn_kortex_robot_d435.launch start_rviz:=false arm:=gen3_lite gripper:=gen3_lite_2f sim_d435:=false
roslaunch realsense2_camera rs_camera.launch
```



### Gazebo


### D435
The urdf file in realsense2_description has a gazebo plugin added with xacro tag at bottom of `_d435.urdf.xacro`

### OctoMap
This is the bright colored grid that you see in RViz that moveit detects as collision bodies.
The yaml and launch files in `kortex_move_it_config` take care of this. To see the OctoMap in rviz you need to add the `MotionPlanning` plugin to rviz (Not added by default because it can be taxing to slower machines).
