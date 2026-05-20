#!/usr/bin/env python3
"""
Publishes initial pose to AMCL repeatedly until AMCL confirms localization
(map→odom transform becomes available).
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('delay', 2.0)
        self.declare_parameter('robot_namespace', 'bcr_bot')

        self.robot_namespace = self.get_parameter('robot_namespace').value

        topic = '/initialpose' if not self.robot_namespace else f'/{self.robot_namespace}/initialpose'
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._publish_until_localized()

    def _build_pose_msg(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = self.get_parameter('x').value
        msg.pose.pose.position.y = self.get_parameter('y').value
        msg.pose.pose.position.z = self.get_parameter('z').value

        yaw = self.get_parameter('yaw').value
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        return msg

    def _is_localized(self):
        odom_frame = f'{self.robot_namespace}/odom' if self.robot_namespace else 'odom'
        try:
            # Use Time() so tf2 returns the latest available transform
            # regardless of sim time state (works even when Gazebo is paused)
            return self.tf_buffer.can_transform(
                'map', odom_frame, rclpy.time.Time(), timeout=Duration(seconds=0)
            )
        except Exception:
            return False

    def _publish_until_localized(self):
        ns = self.robot_namespace
        odom_frame = f'{ns}/odom' if ns else 'odom'
        topic_display = f'/{ns}/initialpose' if ns else '/initialpose'
        self.get_logger().info(f'Publishing initial pose to {topic_display} until map->{odom_frame} is available...')

        deadline = time.time() + 60.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)

            if self._is_localized():
                self.get_logger().info(f'Localization confirmed: map→{odom_frame} is available.')
                return

            msg = self._build_pose_msg()
            self.publisher.publish(msg)
            self.get_logger().info(
                f'Published initial pose: x={msg.pose.pose.position.x:.2f}, '
                f'y={msg.pose.pose.position.y:.2f}, yaw={self.get_parameter("yaw").value:.4f}',
                throttle_duration_sec=3.0,
            )

        self.get_logger().warn('Timed out (60 s) waiting for AMCL localization.')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = InitialPosePublisher()
        node.destroy_node()
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()