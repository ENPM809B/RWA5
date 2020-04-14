# group2_rwa4
Build Instructions...

Extract the group2_rwa4.zip inside "~/catkin_ws/src/"
->open a terminal and the run following command in the terminal.
```
cd ~/catkin_ws/
catkin_make --only-pkg-with-deps group2_rwa4
```

Run Instructions
...
1. Open terminal
2. Type following commands in the terminal
 ```Terminal 1:
source ~/catkin_ws/devel/setup.bash
roslaunch group2_rwa4 group2_rwa4.launch
 ```
3. Run following commands in new terminals:
 ```Terminal 2:
source ~/catkin_ws/devel/setup.bash
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1
```Terminal 3:
source ~/catkin_ws/devel/setup.bash
rosrun group2_rwa4 main_node
```

# **NOTE**
**If the robot doesn't pick up the part, please exit gazebo and run it again. For some reason in some of our computers it doesn't pick the first time but picks it in the second run. For some it picks in the first run.
Thank you.**
