#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import random
import math

class OpenSpacePublisher(Node):
    def __init__(self):
        super().__init__('open_space_publisher')
        self.sub = self.create_subscription(LaserScan, 'fake_scan', self.listen, 10)
        self.sub
        
        self.distance_publisher = self.create_publisher(Float32, 'open_space/distance', 10)
        self.angle_publisher = self.create_publisher(Float32, 'open_space/angle', 10)


    def listen(self, scan: LaserScan):
        max_distance = max(scan.ranges)
        max_index = scan.ranges.index(max_distance)
        max_angle = scan.angle_min + max_index * scan.angle_increment

        distance_msg = Float32()
        distance_msg.data = float(max_distance)
        self.distance_pub.publish(distance_msg)
        angle_msg = Float32()
        angle_msg.data = float(max_angle)
        self.angle_pub.publish(angle_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OpenSpacePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

