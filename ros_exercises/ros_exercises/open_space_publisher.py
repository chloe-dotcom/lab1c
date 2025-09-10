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
        self.sub = self.create_subscription(LaserScan, 'fake_scan', self.listen, 10)
        
        self.publisher_ = self.create_publisher(OpenSpace, 'open_space', 10)

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

