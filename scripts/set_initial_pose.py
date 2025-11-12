#!/usr/bin/env python3
"""
Script para publicar la pose inicial de AMCL automáticamente.
Esto resuelve el warning: "AMCL cannot publish a pose or update the transform.
Please set the initial pose..."
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # Crear publicador para la pose inicial
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # Parámetros configurables
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('delay', 2.0)  # Segundos de espera antes de publicar

        # Esperar un poco para asegurar que AMCL esté listo
        delay = self.get_parameter('delay').value
        self.get_logger().info(f'Esperando {delay} segundos antes de publicar la pose inicial...')
        time.sleep(delay)

        # Publicar la pose inicial
        self.publish_initial_pose()

    def publish_initial_pose(self):
        """Publica la pose inicial para AMCL."""
        msg = PoseWithCovarianceStamped()

        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Pose
        msg.pose.pose.position.x = self.get_parameter('x').value
        msg.pose.pose.position.y = self.get_parameter('y').value
        msg.pose.pose.position.z = self.get_parameter('z').value

        # Orientación (quaternion desde yaw)
        yaw = self.get_parameter('yaw').value
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0 if yaw == 0.0 else yaw / abs(yaw) * 0.7071068  # sin(yaw/2)
        msg.pose.pose.orientation.w = 1.0 if yaw == 0.0 else 0.7071068  # cos(yaw/2)

        # Covarianza (valores típicos para una pose inicial conocida)
        msg.pose.covariance[0] = 0.25   # x
        msg.pose.covariance[7] = 0.25   # y
        msg.pose.covariance[35] = 0.068  # yaw

        # Publicar múltiples veces para asegurar que se reciba
        for i in range(5):
            self.publisher.publish(msg)
            self.get_logger().info(f'Pose inicial publicada ({i+1}/5): x={msg.pose.pose.position.x}, '
                                   f'y={msg.pose.pose.position.y}, yaw={self.get_parameter("yaw").value}')
            time.sleep(0.2)

        self.get_logger().info('Pose inicial establecida correctamente!')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = InitialPosePublisher()
        # El nodo se apaga después de publicar
        node.destroy_node()
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
