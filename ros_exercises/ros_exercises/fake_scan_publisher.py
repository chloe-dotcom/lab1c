#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import random
import math

class FakeScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.scan_publisher = self.create_publisher(LaserScan, 'fake_scan', 10)
        self.len_publisher = self.create_publisher(Float32, 'range_test', 10)
        timer = 0.05
        self.timer = self.create_timer(timer, self.callback)

        self.angle_min = -2.0/3.0 * math.pi
        self.angle_max = 2.0/3.0 * math.pi
        self.angle_increment = math.pi/300.0
        self.range_min = 1.0
        self.range_max = 10.0
        self.num_ranges = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1

    def callback(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "base_link" #need to specify
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [random.uniform(self.range_min, self.range_max) for _ in range(self.num_ranges)] 
        self.scan_publisher.publish(scan)

        length_msg =Float32()
        length_msg.data = float(len(scan.ranges))
        self.len_publisher.publish(length_msg)
        self.get_logger().info(f"{scan}") # optional

def main(args=None):
    rclpy.init(args=args)
    node = FakeScanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

