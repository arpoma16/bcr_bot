#!/usr/bin/env python3
"""Adapt Nav2 to a lifted cart, following lift_state (run inside the robot namespace).

  scan -> scan_filtered (sensor_msgs/LaserScan)
      Plate down: passed through unchanged.
      Plate up: readings that fall inside the cart box are set to +inf, so the cart legs,
      which travel with the robot, are not seen as obstacles. AMCL, the costmaps and the
      collision monitor read scan_filtered (config/nav2_factory_params.yaml).
  local_costmap/footprint, global_costmap/footprint (geometry_msgs/Polygon)
      Plate up: cart footprint. Plate down: robot footprint. The collision monitor follows the
      costmap's published_footprint, so it changes too.

The robot is never exactly centred under the cart (localization error ~0.1 m), so when the
plate goes up the cart pose relative to the robot is measured from its legs in the last raw
scan (cart_legs.py) and the filter box and the footprint are placed there. If the legs are
not found, the cart is assumed centred.
"""

import math

import rclpy
from geometry_msgs.msg import Point32, Polygon
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

import cart_legs


def _rectangle(half_x, half_y):
    return [(half_x, half_y), (half_x, -half_y), (-half_x, -half_y), (-half_x, half_y)]


class CartCarryAdapter(Node):
    def __init__(self):
        super().__init__('cart_carry_adapter')
        p = self.declare_parameter
        self.base_frame = p('base_frame', 'base_link').value
        # Half extents [m]. Robot: as in nav2_factory_params.yaml (0.90 x 0.64).
        # Cart: indra models/lift_cart is 1.20 x 1.00; +2 cm so the legs are inside.
        self.robot_footprint = _rectangle(p('robot_half_x', 0.45).value, p('robot_half_y', 0.32).value)
        self.cart_footprint = _rectangle(p('cart_half_x', 0.62).value, p('cart_half_y', 0.52).value)
        # Scan readings inside this box (cart frame, cart + margin) are dropped while carrying.
        self.filter_half_x = p('filter_half_x', 0.67).value
        self.filter_half_y = p('filter_half_y', 0.57).value
        self.leg_model = cart_legs.leg_model(p('leg_x', 0.58).value, p('leg_y', 0.48).value)
        self.leg_search_radius = p('leg_search_radius', 0.2).value
        self.republish_period = p('footprint_republish_period', 2.0).value

        self.carrying = False
        self.cart_pose = (0.0, 0.0, 0.0)   # cart in base_frame while carrying
        self._last_raw_scan = None
        self._laser_offset = {}            # frame_id -> (x, y, yaw) of the laser in base_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Reliable publisher: matches both best-effort (Nav2) and reliable subscribers.
        self.scan_pub = self.create_publisher(LaserScan, 'scan_filtered', 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, qos_profile_sensor_data)
        self.footprint_pubs = [self.create_publisher(Polygon, f'{c}/footprint', 10)
                               for c in ('local_costmap', 'global_costmap')]
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'lift_state', self._lift_state_cb, latched)
        # Costmaps that start (or restart) after a change would miss it: repeat while carrying.
        self.create_timer(self.republish_period, self._republish)

    # --------------------------------------------------------------- footprint

    def _lift_state_cb(self, msg):
        if msg.data == self.carrying:
            return
        if msg.data:
            self.cart_pose = self._measure_cart()
        self.carrying = msg.data
        self._publish_footprint()
        if self.carrying:
            x, y, yaw = self.cart_pose
            self.get_logger().info(
                f'carrying a cart at ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f} deg) from the '
                'robot: cart footprint, cart legs filtered from the scan')
        else:
            self.get_logger().info('not carrying: robot footprint, scan unfiltered')

    def _measure_cart(self):
        if self._last_raw_scan is None:
            self.get_logger().warn('no scan to locate the cart: assuming it is centred')
            return (0.0, 0.0, 0.0)
        offset = self._offset(self._last_raw_scan.header.frame_id)
        if offset is None:
            return (0.0, 0.0, 0.0)
        found, legs = cart_legs.fit_cart(cart_legs.scan_points(self._last_raw_scan, offset),
                                         (0.0, 0.0, 0.0), self.leg_model, self.leg_search_radius,
                                         min_legs=3)
        if found is None:
            self.get_logger().warn(f'cart legs not found ({legs} of 4): assuming it is centred')
            return (0.0, 0.0, 0.0)
        return found

    def _publish_footprint(self):
        if self.carrying:
            points = [cart_legs.transform(self.cart_pose, pt) for pt in self.cart_footprint]
        else:
            points = self.robot_footprint
        msg = Polygon(points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in points])
        for pub in self.footprint_pubs:
            pub.publish(msg)

    def _republish(self):
        if self.carrying:
            self._publish_footprint()

    # -------------------------------------------------------------------- scan

    def _offset(self, frame_id):
        if frame_id not in self._laser_offset:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, frame_id, Time(), timeout=Duration(seconds=0.0))
            except TransformException as exc:
                self.get_logger().warn(f'no TF {self.base_frame} <- {frame_id} yet: {exc}',
                                       throttle_duration_sec=5.0)
                return None
            # The laser is fixed on the robot: cache it.
            self._laser_offset[frame_id] = cart_legs.laser_offset_from_tf(tf)
        return self._laser_offset[frame_id]

    def _scan_cb(self, scan):
        if not self.carrying:
            self._last_raw_scan = scan
            self.scan_pub.publish(scan)
            return
        offset = self._offset(scan.header.frame_id)
        if offset is None:
            # Without the laser pose the legs cannot be removed; do not feed them to Nav2.
            return
        ox, oy, oyaw = offset
        cx, cy, cyaw = self.cart_pose
        c, s = math.cos(cyaw), math.sin(cyaw)
        ranges = list(scan.ranges)
        for i, r in enumerate(ranges):
            if not math.isfinite(r):
                continue
            a = scan.angle_min + i * scan.angle_increment + oyaw
            dx = ox + r * math.cos(a) - cx
            dy = oy + r * math.sin(a) - cy
            # Point in the cart frame.
            if abs(c * dx + s * dy) <= self.filter_half_x and abs(-s * dx + c * dy) <= self.filter_half_y:
                ranges[i] = math.inf
        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = CartCarryAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
