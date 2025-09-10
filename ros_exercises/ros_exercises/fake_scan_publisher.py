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
        self.declare_parameter("topic", "fake_scan")
        self.scan_publisher = self.create_publisher(LaserScan, self.get_parameter("topic").value, 10)
        self.declare_parameter("publish_rate", 0.05)


        self.len_publisher = self.create_publisher(Float32, 'range_test', 10)
        self.timer = self.create_timer(self.get_parameter("publish_rate").value, self.callback)

        self.declare_parameter("angle_min", -2.0/3.0 * math.pi)
        self.declare_parameter("angle_max", 2.0/3.0 * math.pi)
        self.declare_parameter("angle_increment", math.pi/300.0)
        self.declare_parameter("range_min", 1.0)
        self.declare_parameter("range_max", 10.0)
        self.num_ranges = int(round((self.get_parameter("angle_max").value - self.get_parameter("angle_min").value) / self.get_parameter("angle_increment").value)) + 1

    def callback(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "base_link" #need to specify
        scan.angle_min =self.get_parameter("angle_min").value
        scan.angle_max = self.get_parameter("angle_max").value
        scan.angle_increment = self.get_parameter("angle_increment").value
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = self.get_parameter("range_min").value
        scan.range_max = self.get_parameter("range_max").value
        scan.ranges = [random.uniform(self.get_parameter("range_min").value, self.get_parameter("range_max").value) for _ in range(self.num_ranges)] 
        
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

