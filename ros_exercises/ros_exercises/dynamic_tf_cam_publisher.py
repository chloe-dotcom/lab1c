#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf_transformations


def transform_matrix(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    trans = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    trans[0, 3] = t.x
    trans[1, 3] = t.y
    trans[2, 3] = t.z
    return trans


def transform_from_matrix(T, parent, child):
    msg = TransformStamped()
    msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    translation = tf_transformations.translation_from_matrix(T)
    q = tf_transformations.quaternion_from_matrix(T)

    msg.transform.translation.x = float(translation[0])
    msg.transform.translation.y = float(translation[1])
    msg.transform.translation.z = float(translation[2])
    msg.transform.rotation.x = float(q[0])
    msg.transform.rotation.w = float(q[3])
    msg.transform.rotation.y = float(q[1])
    msg.transform.rotation.z = float(q[2])

    return msg


class DynamicTfCamPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_cam_publisher')

        self.T_base_left = np.array([
            [1, 0, 0, 0.0], #[R T 0 1]
            [0, 1, 0, 0.05],
            [0, 0, 1, 0.0],
            [0, 0, 0, 1.0]
        ])
        self.T_base_right = np.array([
            [1, 0, 0, 0.0],
            [0, 1, 0, -0.05],
            [0, 0, 1, 0.0],
            [0, 0, 0, 1.0]
        ])

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.callback)

    def callback(self):
        trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())

        T_odom_base = transform_matrix(trans)

        T_odom_left = T_odom_base @ self.T_base_left
        msg_left = transform_from_matrix(T_odom_left, 'odom', 'left_cam')
        self.tf_broadcaster.sendTransform(msg_left)
        T_odom_right = T_odom_base @ self.T_base_right
        T_left_right = np.linalg.inv(T_odom_left) @ T_odom_right
        msg_right = transform_from_matrix(T_left_right, 'left_cam', 'right_cam')
        self.tf_broadcaster.sendTransform(msg_right)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTfCamPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()