import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations

class StaticTFCamPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_cam_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)

        self.broadcast_static_transforms()

    def broadcast_static_transforms(self):
        left_tf = TransformStamped()
        left_tf.header.stamp = self.get_clock().now().to_msg()

        left_tf.child_frame_id = 'left_cam'
        left_tf.header.frame_id = 'base_link'
        
        left_tf.transform.translation.y = 0.05
        left_tf.transform.translation.z = 0.0
        left_tf.transform.translation.x = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, 0)
        left_tf.transform.rotation.x = q[0]
        left_tf.transform.rotation.y = q[1]
        left_tf.transform.rotation.z = q[2]
        left_tf.transform.rotation.w = q[3]
        right_tf = TransformStamped()
        right_tf.header.stamp = self.get_clock().now().to_msg()

        right_tf.header.frame_id = 'left_cam'
        right_tf.child_frame_id = 'right_cam'

        right_tf.transform.translation.x = 0.0
        right_tf.transform.translation.z = 0.0
        right_tf.transform.translation.y = -0.10


        q2 = tf_transformations.quaternion_from_euler(0, 0, 0)
        right_tf.transform.rotation.x = q2[0]
        right_tf.transform.rotation.y = q2[1]
        right_tf.transform.rotation.z = q2[2]
        right_tf.transform.rotation.w = q2[3]

        self.broadcaster.sendTransform([left_tf, right_tf])
        self.get_logger().info("Static camera transforms published.")

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFCamPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()