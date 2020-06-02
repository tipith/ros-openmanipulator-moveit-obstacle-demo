# ros-openmanipulator-moveit-obstacle-demo

A small program to manipulate MoveIt planning scene programmatically. Used frame ids and planning groups are compatible with [OpenManipulator] robot arm.

This program is used in testing and in demos.

[OpenManipulator]: http://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/

## prequisites

sudo apt install ros-melodic-moveit-* ros-melodic-turtlebot3-* python-catkin-tools
sudo apt install ros-melodic-joint-trajectory-controller ros-melodic-effort-controllers
cd ~catkin/src && git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git

## how to run

roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch

## some extras

rqt_plot /imu/linear_acceleration/x:y:z
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch