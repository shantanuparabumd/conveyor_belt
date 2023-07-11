#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
import argparse
# import the ROS2 python libraries
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from std_msgs.msg import String


class RobotDescPub(Node):

    def __init__(self, args):

        super().__init__('robot_desciption_pub')
        parser = argparse.ArgumentParser(
            description='Publish into the given topic ( robot_description by default ) the XML string describing you robot')
        parser.add_argument('-xml_string', required=True, type=str, metavar='XML_URDF_XACRO',
                            help='Stringified xml data from urdf or xacro describing the robot')
        parser.add_argument('-robot_description_topic', type=str, default='/robot_description', metavar='ROBOT_DESCRIPTION_TOPIC',
                            help='Name of topic where the robot rescription from xml data will be published')

        self.args = parser.parse_args(args[1:])

        rclpy.logging.set_logger_level(
            'robot_desciption_pub', rclpy.logging.LoggingSeverity.ERROR)

        # Data recieved
        # self.get_logger().info("XML ROBOT ==>"+self.args.xml_string)
        # self.get_logger().info("Topic to pubish ==>"+self.args.robot_description_topic)

        # create the publisher object
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(
            String, '/robot_description', latched_qos)

        # Send Data
        self.send(self.args.xml_string)

        # self.get_logger().info("FINISHED Robot DESCRIPTION PUBLISH")

    def send(self, xml_data):
        # self.get_logger().info("Publishing XML DATA....")
        self.cmd = String()
        self.cmd.data = xml_data
        self.publisher_.publish(self.cmd)
        # self.get_logger().info("Publishing XML DATA.......DONE")


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    # declare the node constructor
    robot_desciption_pub = RobotDescPub(args_without_ros)

    rclpy.spin(robot_desciption_pub)

    robot_desciption_pub.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
