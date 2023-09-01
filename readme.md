
# Conveyor Belt Plugin

![Build Status](https://github.com/shantanuparabumd/conveyor_belt/actions/workflows/main.yml/badge.svg)


![Conveyor Belt](/videos/conveyor.png)

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

## How to Use

### Create Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Pull and Build Package

```bash
git clone https://github.com/shantanuparabumd/conveyor_belt.git
```
**Resolve Dependencies**
```bash
cd ..
rosdep install -i --from-path src --rosdistro galactic -y
```

```bash
colcon build --packages-select conveyor_belt
```
### Launch Conveyor Belt

```bash
ros2 launch conveyor_belt robot.launch.py
```

### Spawn Object

```bash
ros2 run conveyor_belt spawn_object.py

```

**Watch your project work**

https://github.com/shantanuparabumd/conveyor_belt/assets/112659509/0ed853e3-b937-45b8-adbb-2cfae711ef3d

