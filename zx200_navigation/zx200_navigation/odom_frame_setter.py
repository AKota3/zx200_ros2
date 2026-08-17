#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class OdomFrameSetter(Node):

    def __init__(self):
        super().__init__('odom_frame_setter')

        # Parameters
        self.declare_parameter('input_topic', '/input_odom')
        self.declare_parameter('output_topic', '/output_odom')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')

        self.input_topic = self.get_parameter(
            'input_topic').get_parameter_value().string_value

        self.output_topic = self.get_parameter(
            'output_topic').get_parameter_value().string_value

        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value

        self.child_frame_id = self.get_parameter(
            'child_frame_id').get_parameter_value().string_value

        self.sub = self.create_subscription(
            Odometry,
            self.input_topic,
            self.odom_callback,
            10)

        self.pub = self.create_publisher(
            Odometry,
            self.output_topic,
            10)

        self.get_logger().info(
            f'{self.input_topic} -> {self.output_topic}')
        self.get_logger().info(
            f'frame_id={self.frame_id}, child_frame_id={self.child_frame_id}')

    def odom_callback(self, msg):

        odom = Odometry()

        # ヘッダ
        odom.header = msg.header
        odom.header.frame_id = self.frame_id

        # child_frame_id
        odom.child_frame_id = self.child_frame_id

        # 位置・姿勢
        odom.pose = msg.pose

        # 速度
        odom.twist = msg.twist

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)

    node = OdomFrameSetter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()