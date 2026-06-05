#!/usr/bin/env python3
"""
Sets initial pose on AMCL via service, retrying until AMCL confirms
localization by publishing amcl_pose.
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SetInitialPose
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('delay', 2.0)
        self.declare_parameter('robot_namespace', 'bcr_bot')

        ns = self.get_parameter('robot_namespace').value
        self.robot_namespace = ns

        srv_name = f'/{ns}/set_initial_pose' if ns else '/set_initial_pose'
        self.client = self.create_client(SetInitialPose, srv_name)

        self._amcl_pose_received = False
        amcl_topic = f'/{ns}/amcl_pose' if ns else '/amcl_pose'
        self.create_subscription(
            PoseWithCovarianceStamped, amcl_topic,
            self._amcl_pose_cb, 10
        )

        self._set_until_localized()

    def _amcl_pose_cb(self, msg):
        self._amcl_pose_received = True

    def _build_pose_msg(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
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

    def _wait_for_sim_clock(self, timeout=30.0):
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > 0:
                return True
        return False

    def _set_until_localized(self):
        ns = self.robot_namespace
        srv_name = f'/{ns}/set_initial_pose' if ns else '/set_initial_pose'

        self.get_logger().info('Waiting for sim clock...')
        if not self._wait_for_sim_clock():
            self.get_logger().warn('Sim clock never started.')
            return

        self.get_logger().info(f'Waiting for {srv_name} service...')
        deadline_srv = time.time() + 120.0
        while rclpy.ok() and not self.client.service_is_ready() and time.time() < deadline_srv:
            rclpy.spin_once(self, timeout_sec=0.5)
            self.get_logger().info(f'Still waiting for {srv_name}...', throttle_duration_sec=5.0)
        if not self.client.service_is_ready():
            self.get_logger().error(f'Service {srv_name} not available after 120s.')
            return

        delay = self.get_parameter('delay').value
        if delay > 0:
            self.get_logger().info(f'Waiting {delay}s for simulation to stabilize...')
            deadline_delay = time.time() + delay
            while rclpy.ok() and time.time() < deadline_delay:
                rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info('Sending initial pose until AMCL publishes amcl_pose...')
        deadline = time.time() + 120.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)

            if self._amcl_pose_received:
                self.get_logger().info('Localization confirmed: amcl_pose received.')
                return

            req = SetInitialPose.Request()
            req.pose = self._build_pose_msg()
            future = self.client.call_async(req)

            deadline_inner = time.time() + 2.0
            while rclpy.ok() and not future.done() and time.time() < deadline_inner:
                rclpy.spin_once(self, timeout_sec=0.1)

            x = req.pose.pose.pose.position.x
            y = req.pose.pose.pose.position.y
            yaw = self.get_parameter('yaw').value
            self.get_logger().info(
                f'Sent initial pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.4f}',
                throttle_duration_sec=3.0,
            )

        self.get_logger().warn('Timed out (120s) waiting for AMCL to publish amcl_pose.')


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
