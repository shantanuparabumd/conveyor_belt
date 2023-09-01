#!/usr/bin/env python3

import os
import sys
import random
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import xacro

def main():
    """ Main for spwaning turtlebot node """
    ####### DATA INPUT ##########

    xacro_file = "box.urdf.xacro"
    package_description = "conveyor_belt"

    # Position and orientation
    # [X, Y, Z]
    position = [2.0, 0.0, 1.2]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "box"
    entity_name=str(random.randint(0,30))
    ####### DATA INPUT END ##########

    argv = sys.argv[1:]

    # Start node
    rclpy.init()

    sdf_file_path = os.path.join(
        get_package_share_directory(package_description), "urdf",
        xacro_file)
    
    node = rclpy.create_node("entity_spawner")

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

   

    # Set data for request
    robot_desc = xacro.process_file(sdf_file_path)
    

    request = SpawnEntity.Request()
    request.name = robot_base_name + entity_name
    request.xml = robot_desc.toxml()
    request.robot_namespace = entity_name
    request.initial_pose.position.x = position[0]
    request.initial_pose.position.y = position[1]
    request.initial_pose.position.z = position[2]

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()