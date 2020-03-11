# group6_rwa3
Build Instructions
1. Unzip the files to "catkin_ws/src"
2. Run following command in the terminal:
 ```
catkin_make --only-pkg-with-deps group6_rwa3
 ```
 Run Instructions
...
1. Open terminal 
2. Type following commands in the terminal
 ```
cd catkin_ws 
source devel/setup.bash
 ```

3. Run following commands in new terminals:
 ```
roslaunch group6_rwa1 group6-rwa3.launch
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1
rosrun group6_rwa3 main_node
```
