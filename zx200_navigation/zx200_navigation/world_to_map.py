#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OffsetOdometryNode(Node):

    def __init__(self):
        super().__init__('offset_odometry_node')

        # 引く値（オフセット）
        # self.offset_x = 21395.178
        # self.offset_y = 14034.450
        # self.offset_z = 28.552
        self.offset_x = 0
        self.offset_y = 0
        self.offset_z = 0

        # Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/zx200/global_pose_odom',      # 入力トピック名
            self.odom_callback,
            10)

        # Publisher
        self.publisher = self.create_publisher(
            Odometry,
            '/zx200/global_pose_odom_map',     # 出力トピック名
            10)

        self.get_logger().info("Offset Odometry Node Started")

    def odom_callback(self, msg: Odometry):

        # コピー
        new_msg = Odometry()

        # header / child_frame_id はそのまま
        new_msg.header = msg.header
        new_msg.child_frame_id = msg.child_frame_id

        # ===== 座標を引く =====
        new_msg.pose.pose.position.x = msg.pose.pose.position.x - self.offset_x
        new_msg.pose.pose.position.y = msg.pose.pose.position.y - self.offset_y
        new_msg.pose.pose.position.z = msg.pose.pose.position.z - self.offset_z

        # orientation はそのまま
        new_msg.pose.pose.orientation = msg.pose.pose.orientation

        # twist そのまま
        new_msg.twist = msg.twist

        # covariance そのまま
        new_msg.pose.covariance = msg.pose.covariance
        new_msg.twist.covariance = msg.twist.covariance

        # publish
        self.publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OffsetOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()