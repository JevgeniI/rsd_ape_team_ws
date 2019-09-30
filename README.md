## Repo for RSD 2019 by The Ape-Team
This repo is a ros/catkin workspace with three packages:

1. **universal_robot**

... Package for the UR robot. Has many models. Has the robot description files, moveit files, and many more. This is the robot itself.

2. **ur_modern_driver**

...The drivers for communicating with the **real** robot.

3. **our_ur**

...This is where we will be spending our time. This package borrows from the above packages and many others.

### Dependencies
TO-DO

### Install and Build Instructions
```
sudo apt-get install ros-kinetic-desktop-full ros-kinetic-moveit ros-kinetic-moveit-visual-tools
cd ~
git clone https://github.com/mabouseif/rsd_ape_team_ws.git
cd ~/rsd_ape_team_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
rospack profile
```
and then download the UR robot simulator
[Download UR Simulator](https://www.universal-robots.com/download/?option=51593#section41511)
and follow the instructions to install it and run it.

In case of desire to test on the real robot, this is the place to start first.
Run the simulator first, turn the robot on (in simulatrion), and then run the following command `roslaunch our_ur ur_cell_real.launch`.
Currently this launch file launches rviz with the motion planning panel so you can play around and see that what you do in rviz translates to the simulator (and consequently the robot)


### So far
- **our_ur** package has two launch files. One is for real robot mode `ur_cell_real.launch`, and the other for simulation purposes `ur_cell_simulation.launch`. We will be using the latter for now. 
- There's a simple program that subsribes to the `/joint_states` topic and prints out one of the joint position values, `ur_go_to_pose_service_node`, which is incorrectly named.
- There's a simple program that uses `MoveIt!` library and creates a `move_group` object of the group `manipulator` which commands the robot to go to hardcoded poses. The robot does move, but i think it always moves to the pose relative to a frame other than the world frame, i.e from each different starting pose, when the script is run, the robot goes to a different pose, depending on its starting position. 



## TO-DO
TO-DO



## Run Instructions
- To run the robot in Gazebo simulation:
`roslaunch our_ur ur_cell_simulation.launch`
You can then use the motion planning panel in rviz to set goal poses and plan and execute trajectories.
- To run with the real/robot or URsim:
Start either one, and then run `roslaunch our_ur ur_cell_real.launch` and do the same as above.
You can also run `rosrun our_ur move_stuff_node` and the robot will go to a hardcoded predetermined pose.
