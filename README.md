## Repo for RSD 2019 by The Ape-Team
This repo is a ros/catkin workspace with three packages:
1. **universal_robot**

... Package for the UR robot. Has many models. Has the robot description files, moveit files, and many more. This is the robot itself.

2. **ur_modern_driver**

...The drivers for communicating with the **real** robot.

3. **our_ur**

...This is where we will be spending our time. This package borrows from the above packages and many others. 


### So far
- **our_ur** package has two launch files. One is for real robot mode `ur_cell_real.launch`, and the other for simulation purposes `ur_cell_simulation.launch`. We will be using the latter for now. 
- There's a simple program that subsribes to the `/joint_states` topic and prints out one of the joint position values, `ur_go_to_pose_service_node`, which is incorrectly named.
- There's a simple program that uses `MoveIt!` library and creates a `move_group` object of the group `manipulator` which commands the robot to go to hardcoded poses. The robot does move, but i think it always moves to the pose relative to a frame other than the world frame, i.e from each different starting pose, when the script is run, the robot goes to a different pose, depending on its starting position. 



## TO-DO

