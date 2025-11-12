#!/usr/bin/env python3
"""
Script para activar los nodos de navegación manualmente.
Espera un delay y luego activa todos los nodos de Nav2.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time


class NavigationActivator(Node):
    def __init__(self):
        super().__init__('navigation_activator')

        # Parámetros
        self.declare_parameter('delay', 7.0)
        delay = self.get_parameter('delay').value

        self.get_logger().info(f'Esperando {delay} segundos antes de activar los nodos de navegación...')
        time.sleep(delay)

        self.get_logger().info('Activando nodos de navegación...')
        self.activate_navigation()

    def activate_navigation(self):
        """Activa los nodos de navegación llamando al servicio del lifecycle manager."""
        # Primero verificar si ya está activo
        is_active_client = self.create_client(Empty, '/lifecycle_manager_navigation/is_active')

        if is_active_client.wait_for_service(timeout_sec=2.0):
            request = Empty.Request()
            future = is_active_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            # Si ya está activo, no hacer nada
            if future.result() is not None:
                self.get_logger().info('✓ Los nodos de navegación ya están activos!')
                self.get_logger().info('El sistema Nav2 está listo para recibir goals.')
                return

        # Si no está activo, intentar activar
        client = self.create_client(Empty, '/lifecycle_manager_navigation/manage_nodes')

        # Esperar a que el servicio esté disponible
        timeout_sec = 10.0
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f'Servicio /lifecycle_manager_navigation/manage_nodes no disponible después de {timeout_sec}s')
            self.get_logger().warn('Puede que el autostart ya haya activado los nodos.')
            return

        # Llamar al servicio
        request = Empty.Request()
        future = client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() is not None:
            self.get_logger().info('✓ Nodos de navegación activados correctamente!')
            self.get_logger().info('El sistema Nav2 está listo para recibir goals.')
        else:
            self.get_logger().warn('El lifecycle manager puede que ya esté procesando el autostart.')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = NavigationActivator()
        # El nodo se apaga después de activar
        node.destroy_node()
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
