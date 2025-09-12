import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf_transformations

class BaseLinkTFPublisher(Node):
    def __init__(self):
        super().__init__('base_link_tf_pub')

        #listener
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.callback)

        self.left_baselink = np.array([
            [1, 0, 0, -0.05],  
            [0, 1, 0, 0.0],
            [0, 0, 1, 0.0],
            [0, 0, 0, 1]
        ])

    def callback(self):
        trans = self.tf_buf.lookup_transform('odom', 'left_cam', rclpy.time.Time())
        
        translation = [trans.transform.translation.x,
                        trans.transform.translation.y,
                        trans.transform.translation.z]
        rotation = [trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w]
        odom_to_left = tf_transformations.concatenate_matrices(
            tf_transformations.translation_matrix(translation),
            tf_transformations.quaternion_matrix(rotation)
        )

        o_baselink2 = np.dot(odom_to_left, self.left_baselink)

        trans_from_matrix = tf_transformations.translation_from_matrix(o_baselink2)
        quat = tf_transformations.quaternion_from_matrix(o_baselink2)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link_2'

        msg.transform.translation.x = trans_from_matrix[0]
        msg.transform.translation.y = trans_from_matrix[1]
        msg.transform.translation.z = trans_from_matrix[2]
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(msg)

def main(args=None):

    rclpy.init(args=args)
    node = BaseLinkTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()