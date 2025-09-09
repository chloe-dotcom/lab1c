#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(Float32, 'my_random_float', 10)
        self.timer = self.create_timer(0.05, self.publish_random_number)

    def publish_random_number(self):
        random_number = random.uniform(0.0, 10.0)
        msg = Float32()
        msg.data =random_number

        self.publisher_.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        simple_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

