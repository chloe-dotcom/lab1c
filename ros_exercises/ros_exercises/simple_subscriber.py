#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random
import math

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.sub = self.create_subscription(Float32, 'my_random_float', self.listen, 10)
        self.sub

        self.publisher_ = self.create_publisher(Float32, 'random_float_log', 10)

    def listen(self, msg):
        val = msg.data
        log_val = math.log(val)
        log_msg = Float32()
        log_msg.data = log_val
        self.publisher_.publish(log_msg)


def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    try:
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        simple_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

