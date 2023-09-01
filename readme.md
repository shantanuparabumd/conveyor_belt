
# Conveyor Belt Plugging

<!-- ![Build Status](https://github.com/shantanuparabumd/project_legion/actions/workflows/project_legion_git_ci.yml/badge.svg) -->


<!-- ![Project Legion](/images/robotaxi.jpg) -->

## Authors

|Name|ID|Email|
|:---:|:---:|:---:|
|Shantanu Parab|119208625|sparab@umd.edu|



## Introduction

    In this project we implement a conveyor belt that can be used in any environment. This project makes use of solidworks to create a conveyor belt and export it to URDF we further add controllers and spawn the conveyor belt in an environment and then spwan a object over the belt. We write a pluggin to control the conveyor belt and move the object to the end of the belt. 

## Dependencies

- OS: Ubuntu Linux 22.04
- ROS Version: ROS2 Galactic
- C++



## Documents

|AIP Backlog and Worklog Sheet|[Link](https://docs.google.com/spreadsheets/d/1-Oc5Umwckcke2KnCPDlawZ_dHhSJcMz8yR0DFILcYkc/edit#gid=0)|



<!-- # Dependeny Installation and Setup

Installing ROS Controller (Run this in home directory)

`sudo apt install ros-galactic-ros2-control ros-galactic-ros2-controllers ros-galactic-gazebo-ros2-control`

Install xacro module to read xacro files
`pip install xacro`

Launch gazebo using launch file and then run the below 2 commands to start the controllers
# Manually Starting Controllers (Top 2 Only)

`ros2 control load_controller --set-state start joint_state_broadcaster`

`ros2 control load_controller --set-state start velocity_controller`

`ros2 control load_controller --set-state start joint_trajectory_controller`

# Check Topics

`ros2 topic list`

# Publish Velocity

`ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0,-1.0,1.0,-1.0],layout: {dim:[], data_offset: 1"}}`

`ros2 topic pub /joint_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.3,0.3,0.3,0.3],layout: {dim:[], data_offset: 1"}}`

Tried using this command but did not works try new commands

https://www.youtube.com/watch?v=BmLdjLNJHoY -->


export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/ros_ws2/build/conveyor_belt