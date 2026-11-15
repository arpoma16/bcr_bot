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

Box mode (SetBool service `carry_box_mode`): for a load that is NOT a cart (a box on the plate,
indra_fleet_adapter pick_box / drop_box). Set it while the plate is down and before lift_up; then
no cart is measured, the footprint stays the robot's and only the scan returns inside the robot
(+ box_filter_margin) are dropped. It clears itself when the plate goes down, cannot be armed with
the plate up (so a real cart is never lifted in box mode) and cannot be dropped under a load.
Default off: the cart behaviour above is unchanged.

The robot is never exactly centred under the cart (localization error ~0.1 m), so when the
plate goes up the cart pose relative to the robot is measured from its legs in the last raw
scan (cart_legs.py) and the filter box and the footprint are placed there. If the legs are
not found, the cart is assumed centred.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Point32, Polygon
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

import cart_legs


def _rectangle(half_x, half_y):
    return [(half_x, half_y), (half_x, -half_y), (-half_x, -half_y), (-half_x, half_y)]


class CartCarryAdapter(Node):
    box_mode = False        # class default: off unless the carry_box_mode service armed it

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
        # Box mode: scan returns within the robot footprint plus this margin are dropped.
        self.box_filter_margin = p('box_filter_margin', 0.05).value
        self.box_mode = False
        self.leg_model = cart_legs.leg_model(p('leg_x', 0.58).value, p('leg_y', 0.48).value)
        self.leg_search_radius = p('leg_search_radius', 0.2).value
        self.republish_period = p('footprint_republish_period', 2.0).value
        # Only the handler's guarded translations may borrow the robot footprint.
        self.straight_lease_timeout = p('straight_lease_timeout', 0.6).value
        self._straight_until = 0.0
        self._cart_measured = False

        self.carrying = False
        self.cart_pose = (0.0, 0.0, 0.0)   # cart in base_frame while carrying
        self._last_raw_scan = None
        self._last_raw_received = 0.0
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
        self.create_service(SetBool, 'cart_straight_footprint', self._straight_footprint)
        self.create_service(SetBool, 'carry_box_mode', self._carry_box_mode)
        self.create_timer(0.1, self._expire_straight)

    # --------------------------------------------------------------- footprint

    def _lift_state_cb(self, msg):
        if msg.data == self.carrying:
            return
        self._straight_until = 0.0
        if msg.data:
            if self.box_mode:
                self.cart_pose = (0.0, 0.0, 0.0)    # no cart: nothing to measure
                self._cart_measured = False
            else:
                self.cart_pose = self._measure_cart()
        else:
            self.box_mode = False                   # box mode never outlives its load
        self.carrying = msg.data
        self._publish_footprint()
        if self.carrying and self.box_mode:
            self.get_logger().info('carrying a box: robot footprint, returns inside the robot '
                                   'filtered from the scan')
        elif self.carrying:
            x, y, yaw = self.cart_pose
            self.get_logger().info(
                f'carrying a cart at ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f} deg) from the '
                'robot: cart footprint, cart legs filtered from the scan')
        else:
            self.get_logger().info('not carrying: robot footprint, scan unfiltered')

    def _measure_cart(self):
        self._cart_measured = False
        if self._last_raw_scan is None or time.monotonic() - self._last_raw_received > 0.3:
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
        self._cart_measured = True
        return found

    def _carry_box_mode(self, request, response):
        if request.data:
            if self.carrying:
                response.success = False
                response.message = 'the platform is already up: box mode must be set before lifting'
            else:
                self.box_mode = True
                response.success = True
                response.message = 'box mode on: the robot footprint is kept while carrying'
        elif self.carrying and self.box_mode:
            response.success = False
            response.message = 'a box is on the platform: box mode clears when it goes down'
        else:
            self.box_mode = False
            response.success = True
            response.message = 'box mode off'
        return response

    def _straight_footprint(self, request, response):
        if not request.data:
            self._straight_until = 0.0
        elif self.carrying and not self._cart_measured:
            response.success = False
            response.message = 'cannot guard a cart whose pose was not measured'
            return response
        else:
            self._straight_until = time.monotonic() + self.straight_lease_timeout
        self._publish_footprint()
        response.success = True
        response.message = json.dumps({'carrying': self.carrying,
                                       'measured': self._cart_measured,
                                       'cart_pose': self.cart_pose if self.carrying else [0., 0., 0.],
                                       'half_x': max(x for x, _ in (
                                           self.cart_footprint if self.carrying else self.robot_footprint)),
                                       'half_y': max(y for _, y in (
                                           self.cart_footprint if self.carrying else self.robot_footprint)),
                                       'robot_half_x': max(x for x, _ in self.robot_footprint),
                                       'robot_half_y': max(y for _, y in self.robot_footprint)})
        return response

    def _expire_straight(self):
        if self._straight_until and time.monotonic() >= self._straight_until:
            self._straight_until = 0.0
            self._publish_footprint()

    def _publish_footprint(self):
        if self.carrying and not self.box_mode and time.monotonic() >= self._straight_until:
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
            self._last_raw_received = time.monotonic()
            self.scan_pub.publish(scan)
            return
        offset = self._offset(scan.header.frame_id)
        if offset is None:
            # Without the laser pose the legs cannot be removed; do not feed them to Nav2.
            return
        ox, oy, oyaw = offset
        cx, cy, cyaw = self.cart_pose
        c, s = math.cos(cyaw), math.sin(cyaw)
        if self.box_mode:
            half_x = max(x for x, _ in self.robot_footprint) + self.box_filter_margin
            half_y = max(y for _, y in self.robot_footprint) + self.box_filter_margin
        else:
            half_x, half_y = self.filter_half_x, self.filter_half_y
        ranges = list(scan.ranges)
        for i, r in enumerate(ranges):
            if not math.isfinite(r):
                continue
            a = scan.angle_min + i * scan.angle_increment + oyaw
            dx = ox + r * math.cos(a) - cx
            dy = oy + r * math.sin(a) - cy
            # Point in the cart frame.
            if abs(c * dx + s * dy) <= half_x and abs(-s * dx + c * dy) <= half_y:
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
