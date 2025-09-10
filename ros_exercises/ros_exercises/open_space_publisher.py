#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import OpenSpace
import random
import math

class OpenSpacePublisher(Node):
    def __init__(self):
        super().__init__('open_space_publisher')
        self.declare_parameter("subscriber_topic", "fake_scan")
        self.declare_parameter("publisher_topic", "open_scan")

        self.subscription = self.create_subscription(
                LaserScan,
                self.get_parameter("subscriber_topic").value,
                self.listen, 10
                )
        self.publisher_ = self.create_publisher(OpenSpace, self.get_parameter("publisher_topic").value, 10)

    def listen(self, scan: LaserScan):
        max_distance = max(scan.ranges)
        max_index = scan.ranges.index(max_distance)
        max_angle = scan.angle_min + max_index * scan.angle_increment

        msg = OpenSpace()
        msg.angle = float(max_angle)
        msg.distance = float(max_distance)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OpenSpacePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

