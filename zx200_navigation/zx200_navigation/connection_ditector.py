#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class SafetyNode(Node):

    def __init__(self):
        super().__init__("safety_node")

        self.declare_parameter('input_pose_topic', "/zx200/global_pose")
        self.declare_parameter('input_velosity_topic', "/nav2_cmd_vel")
        self.declare_parameter('output_velosity_topic', "/zx200/cmd_vel")
        # "odometry" または "pose_stamped"
        self.declare_parameter('pose_msg_type', "pose_stamped")

        self.input_pose_topic = self.get_parameter('input_pose_topic').get_parameter_value().string_value
        self.input_velosity_topic = self.get_parameter('input_velosity_topic').get_parameter_value().string_value
        self.output_velosity_topic= self.get_parameter('output_velosity_topic').get_parameter_value().string_value
        self.pose_msg_type = self.get_parameter('pose_msg_type').get_parameter_value().string_value

        # パラメータ
        self.declare_parameter("timeout", 0.5)
        self.timeout = self.get_parameter("timeout").value

        # 最後にglobal_poseを受信した時刻
        self.last_pose_time = self.get_clock().now()

        # 最後に受信したcmd_vel
        self.last_cmd = Twist()

        # Subscriber
        if self.pose_msg_type == "odometry":
            self.pose_sub = self.create_subscription(
                Odometry,
                self.input_pose_topic,
                self.pose_callback,
                10)
        elif self.pose_msg_type == "pose_stamped":
            self.pose_sub = self.create_subscription(
                PoseStamped,
                self.input_pose_topic,
                self.pose_callback,
                10)
        else:
            self.get_logger().error("Invalid pose_msg_type. Use 'odometry' or 'pose_stamped'.")
            raise ValueError("pose_msg_type must be 'odometry' or 'pose_stamped'")

        self.cmd_sub = self.create_subscription(
            Twist,
            self.input_velosity_topic,
            self.cmd_callback,
            10)

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            self.output_velosity_topic,
            10)

        # 100Hz
        self.timer = self.create_timer(
            0.01,
            self.timer_callback)

        self.get_logger().info("Safety Node Started")

    def pose_callback(self, msg):
        self.last_pose_time = self.get_clock().now()

    def cmd_callback(self, msg):
        self.last_cmd = msg

    def timer_callback(self):

        now = self.get_clock().now()

        dt = (
            now - self.last_pose_time
        ).nanoseconds / 1e9

        if dt > self.timeout:

            stop = Twist()

            stop.linear.x = 0.0
            stop.linear.y = 0.0
            stop.linear.z = 0.0

            stop.angular.x = 0.0
            stop.angular.y = 0.0
            stop.angular.z = 0.0

            self.cmd_pub.publish(stop)

            self.get_logger().warn(
                "Localization timeout! Stop robot."
            )

        else:

            self.cmd_pub.publish(self.last_cmd)


def main(args=None):

    rclpy.init(args=args)

    node = SafetyNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()