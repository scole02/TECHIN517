## Gazebo, D435, and OctoMap
`$ catkin build`
`$ roslaunch kortex_gazebo spawn_kortex_robot_d435.launch start_rviz:=false arm:=gen3_lite gripper:=gen3_lite_2f`
### Gazebo


### D435
The urdf file in realsense2_description has a gazebo plugin added with xacro tag at bottom of `_d435.urdf.xacro`

### OctoMap
This is the bright colored grid that you see in RViz that moveit detects as collision bodies.
The yaml and launch files in `kortex_move_it_config` take care of this.
