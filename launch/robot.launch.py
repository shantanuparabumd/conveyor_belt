#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Get Gazebo ROS interface package
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Get the location for empty world
    world = os.path.join(
        get_package_share_directory('conveyor_belt'),
        'worlds',
        'empty_world.world'
    )

    # Launch Description to run Gazebo Server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Launch Description to run Gazebo Client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Get the package directory 
    pkg_robotaxi_gazebo = get_package_share_directory('conveyor_belt')

   

    # Launch Decription to Spawn Robot Model 
    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robotaxi_gazebo, 'launch',
                         'spawn_robot_ros2.launch.py'),
        )
    )


    

    # Launch Description 
    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        spawn_robot_world
        
    ])
