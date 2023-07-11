import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
import xacro
import random

# this is the function launch  system will look for


def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'conveyor_belt.urdf'
    xacro_file = "conveyor_belt.urdf.xacro"
    #xacro_file = "box_bot.xacro"
    package_description = "conveyor_belt"
    use_urdf = False
    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.01]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "belt"
    ####### DATA INPUT END ##########

    if use_urdf:
        # print("URDF URDF URDF URDF URDF URDF URDF URDF URDF URDF URDF ==>")
        robot_desc_path = os.path.join(get_package_share_directory(
            package_description), "robot", urdf_file)
    else:
        # print("XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO ==>")
        robot_desc_path = os.path.join(get_package_share_directory(
            package_description), "urdf", xacro_file)

    robot_desc = xacro.process_file(robot_desc_path)
    xml = robot_desc.toxml()

    entity_name = robot_base_name+"-"+str(random.random())

    # Spawn ROBOT Set Gazebo (Does not spwan robot only communicates with the Gazebo Client)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )

    # Publish Robot Desciption in String form in the topic /robot_description
    publish_robot_description = Node(
        package='conveyor_belt',
        executable='robot_description_publisher.py',
        name='robot_description_publisher',
        output='screen',
        arguments=['-xml_string', xml,
                   '-robot_description_topic', '/robot_description'
                   ]
    )

    # Robot State Publisher
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': xml}],
        output="screen"
    )

    # controller = 'wheel'
    # args = [controller]
    # controller_spawner = Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             name=controller + "_spawner",
    #             arguments=["joint_state_broadcaster", "--controller-manager","/controller_manager"]
    #         )
    # box_bot_name = "robotaxi"
    # spawn_controller_1_name = box_bot_name + "_spawn_controller_joint_state_broadcaster"
    # spawn_controller_2_name = box_bot_name + "_spawn_controller_joint_trajectory_controller"
    # spawn_controller_3_name = box_bot_name + "_spawn_controller_velocity_controller"
    # controller_manager_name = box_bot_name + "/controller_manager"

    # spawn_controller_1 = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     name=spawn_controller_1_name,
    #     namespace=box_bot_name,
    #     arguments=["joint_state_broadcaster", "--controller-manager", controller_manager_name],
    #     output="screen"
    # )

    # spawn_controller_2 = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     name=spawn_controller_2_name,
    #     namespace=box_bot_name,
    #     arguments=["joint_trajectory_controller", "--controller-manager", controller_manager_name],
    #     output="screen"
    # )

    # spawn_controller_3 = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     name=spawn_controller_3_name,
    #     namespace=box_bot_name,
    #     arguments=["velocity_controller", "--controller-manager", controller_manager_name],
    #     output="screen"
    # )

    # create and return launch description object
    return LaunchDescription(
        [   
            publish_robot_description,
            robot_state_publisher,
            spawn_robot,
            # spawn_controller_1,
            # spawn_controller_2,
            # spawn_controller_3
        ]
    )
