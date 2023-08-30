"""
spawn_turtlebot.py

Script used to spawn a turtlebot in a generic position
"""
import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

def main():
    """ Main for spwaning turtlebot node """
    ####### DATA INPUT ##########
    urdf_file = 'conveyor_belt.urdf'
    xacro_file = "model.sdf"
    #xacro_file = "box_bot.xacro"
    package_description = "ariac_gazebo"
    use_urdf = False
    # Position and orientation
    # [X, Y, Z]
    position = [2.5, 0.0, 1.0]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "battery"
    ####### DATA INPUT END ##########

    argv = sys.argv[1:]

    # Start node
    rclpy.init()

    sdf_file_path = os.path.join(
        get_package_share_directory(package_description), "models","battery",
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
    request = SpawnEntity.Request()
    request.name = robot_base_name+argv[0]
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = argv[0]
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