#!/usr/bin/env python3
"""Pick and drop a lift cart with Nav2 + the lift (run inside the robot namespace).

  pick_cart (nav2_msgs/action/DockRobot)
      dock_pose = cart centre and heading (frame map; the robot drives in along the cart's +x).
      use_dock_id must be false (there is no cart database). dock_type is ignored.
      1. NAV_TO_STAGING_POSE: NavigateToPose to staging_distance behind the cart, facing it
         (skipped with navigate_to_staging_pose=false).
      2. INITIAL_PERCEPTION / CONTROLLING: AMCL does not update while the robot stands still
         (update_min_d / update_min_a), so after Nav2 stops it can be off by 0.35 m and 9.5 deg
         (measured): first localization_updates calls to request_nomotion_update (0.18 m / 0.6 deg
         at worst after them), so the cart guess falls within leg_search_radius.
         Nav2 stops within xy_goal_tolerance (0.25 m) and the legs leave ~0.10 m per side, so the
         cart pose is measured from its legs in the scan (cart_legs.py) and the robot is corrected on the
         cart axis (spin + drive_on_heading + spin) until within lateral_tolerance /
         yaw_tolerance, checked again at pre_entry_distance; then drive_on_heading under the
         cart and lift_up.
  drop_cart (nav2_msgs/action/UndockRobot)
      lift_down, then backup by backout_distance to leave the cart where it is.

Error codes reuse DockRobot/UndockRobot: DOCK_NOT_IN_DB (dock_id given), DOCK_NOT_VALID (bad
frame, already/not carrying, busy), FAILED_TO_STAGE (navigation), FAILED_TO_DETECT_DOCK
(legs not found in the scan), FAILED_TO_CONTROL
(alignment, drive, lift, backup). One maneuver at a time; cancelling stops the running Nav2
action. While the cart is up, cart_carry_adapter.py adapts Nav2 (footprint, scan).
"""

import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import BackUp, DockRobot, DriveOnHeading, NavigateToPose, Spin, UndockRobot
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

import cart_legs

DOCK = DockRobot.Result
DOCK_FB = DockRobot.Feedback


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _duration_msg(seconds):
    sec = int(seconds)
    return DurationMsg(sec=sec, nanosec=int((seconds - sec) * 1e9))


class Canceled(Exception):
    pass


class Failed(Exception):
    def __init__(self, code, msg):
        super().__init__(msg)
        self.code = code


class CartHandler(Node):
    def __init__(self):
        super().__init__('cart_handler')
        p = self.declare_parameter
        self.global_frame = p('global_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        self.staging_distance = p('staging_distance', 1.8).value      # behind the cart centre [m]
        self.correction_lookahead = p('correction_lookahead', 0.5).value
        self.min_correction_clearance = p('min_correction_clearance', 1.3).value
        # Accepted when the lateral offset predicted at the cart centre (offset + distance *
        # sin(heading error)) is within lateral_tolerance and the heading within yaw_tolerance.
        self.lateral_tolerance = p('lateral_tolerance', 0.04).value   # [m]
        self.yaw_tolerance = p('yaw_tolerance', math.radians(3.0)).value
        self.max_corrections = p('max_corrections', 3).value
        self.approach_speed = p('approach_speed', 0.15).value
        self.backout_distance = p('backout_distance', 1.3).value
        self.backout_speed = p('backout_speed', 0.15).value
        self.lift_timeout = p('lift_timeout', 15.0).value
        self.server_timeout = p('server_timeout', 5.0).value
        # Last check before entering, close enough for several scan points per leg and far
        # enough to spin in place without sweeping the legs (front corner radius ~0.57 m).
        self.pre_entry_distance = p('pre_entry_distance', 1.3).value
        # Cart legs (indra models/lift_cart) and their detection in the raw scan.
        self.leg_model = cart_legs.leg_model(p('leg_x', 0.58).value, p('leg_y', 0.48).value)
        self.leg_search_radius = p('leg_search_radius', 0.25).value
        self.min_legs = p('min_legs', 2).value
        # Forced AMCL updates before looking for the legs (0 disables).
        self.localization_updates = p('localization_updates', 10).value
        self.localization_period = p('localization_period', 0.3).value   # [s] between updates

        self.lifted = False
        self._scan = None              # (LaserScan, monotonic receive time)
        self._laser_offset = None
        self._busy = threading.Lock()
        group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=group)
        self.drive = ActionClient(self, DriveOnHeading, 'drive_on_heading', callback_group=group)
        self.spin = ActionClient(self, Spin, 'spin', callback_group=group)
        self.backup = ActionClient(self, BackUp, 'backup', callback_group=group)
        self.lift_up = self.create_client(Trigger, 'lift_up', callback_group=group)
        self.lift_down = self.create_client(Trigger, 'lift_down', callback_group=group)
        self.nomotion = self.create_client(Empty, 'request_nomotion_update', callback_group=group)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'lift_state', self._lift_state_cb, latched,
                                 callback_group=group)
        # Raw scan: the legs must be visible (scan_filtered hides them while carrying).
        self.create_subscription(LaserScan, 'scan', self._scan_cb, qos_profile_sensor_data,
                                 callback_group=group)

        for name, action_type, execute in (('pick_cart', DockRobot, self._pick),
                                           ('drop_cart', UndockRobot, self._drop)):
            ActionServer(self, action_type, name, execute_callback=execute,
                         goal_callback=self._goal_cb, cancel_callback=lambda _: CancelResponse.ACCEPT,
                         callback_group=group)
        self.get_logger().info('pick_cart / drop_cart ready')

    def _lift_state_cb(self, msg):
        self.lifted = msg.data

    def _scan_cb(self, msg):
        self._scan = (msg, time.monotonic())

    def _detect_cart(self, cart_guess):
        """Cart pose (x, y, yaw) in the global frame measured from its legs in the scan."""
        t_request = time.monotonic()
        while self._scan is None or self._scan[1] <= t_request:
            if time.monotonic() - t_request > 2.0:
                raise Failed(DOCK.FAILED_TO_DETECT_DOCK, 'no fresh scan')
            time.sleep(0.02)
        scan = self._scan[0]
        if self._laser_offset is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, scan.header.frame_id, Time(),
                                                     timeout=Duration(seconds=1.0))
            except TransformException as exc:
                raise Failed(DOCK.FAILED_TO_DETECT_DOCK, f'no laser TF: {exc}')
            self._laser_offset = cart_legs.laser_offset_from_tf(tf)

        rx, ry, ryaw = self._robot_pose()
        c, s = math.cos(ryaw), math.sin(ryaw)
        dx, dy = cart_guess[0] - rx, cart_guess[1] - ry
        guess = (c * dx + s * dy, -s * dx + c * dy, _wrap(cart_guess[2] - ryaw))
        found, legs = cart_legs.fit_cart(cart_legs.scan_points(scan, self._laser_offset), guess,
                                         self.leg_model, self.leg_search_radius, self.min_legs)
        if found is None:
            raise Failed(DOCK.FAILED_TO_DETECT_DOCK,
                         f'cart legs not found near the given pose ({legs} of 4 legs)')
        fx, fy, fyaw = found
        cart = (rx + c * fx - s * fy, ry + s * fx + c * fy, _wrap(ryaw + fyaw))
        self.get_logger().info(
            f'cart detected from {legs} legs at ({cart[0]:.3f}, {cart[1]:.3f}, '
            f'{math.degrees(cart[2]):.1f} deg), {math.hypot(cart[0] - cart_guess[0], cart[1] - cart_guess[1]):.3f} m '
            f'from the previous estimate')
        return cart

    def _goal_cb(self, _goal):
        # One maneuver at a time (pick and drop share the robot).
        return GoalResponse.REJECT if self._busy.locked() else GoalResponse.ACCEPT

    # ------------------------------------------------------------------ helpers

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time(),
                                                 timeout=Duration(seconds=1.0))
        except TransformException as exc:
            raise Failed(DOCK.FAILED_TO_CONTROL, f'no robot pose: {exc}')
        t = tf.transform.translation
        return t.x, t.y, _yaw(tf.transform.rotation)

    def _wait(self, future, handle, timeout, on_cancel=None):
        deadline = time.monotonic() + timeout
        while not future.done():
            if handle.is_cancel_requested:
                if on_cancel:
                    on_cancel()
                raise Canceled()
            if time.monotonic() > deadline:
                return False
            time.sleep(0.05)
        return True

    def _run(self, client, goal, handle, timeout, code, what):
        """Send a Nav2 action goal and wait for it; Failed(code) unless it SUCCEEDS."""
        if not client.wait_for_server(timeout_sec=self.server_timeout):
            raise Failed(code, f'{what}: action server not available')
        send = client.send_goal_async(goal)
        if not self._wait(send, handle, self.server_timeout) or not send.result().accepted:
            raise Failed(code, f'{what}: goal rejected')
        nav_handle = send.result()
        result = nav_handle.get_result_async()
        if not self._wait(result, handle, timeout, on_cancel=nav_handle.cancel_goal_async):
            nav_handle.cancel_goal_async()
            raise Failed(code, f'{what}: timeout after {timeout:.0f} s')
        res = result.result()
        if res.status != 4:  # GoalStatus.STATUS_SUCCEEDED
            err = getattr(res.result, 'error_code', '?')
            raise Failed(code, f'{what}: failed (status {res.status}, error_code {err})')

    def _call(self, client, handle, code, what):
        if not client.wait_for_service(timeout_sec=self.server_timeout):
            raise Failed(code, f'{what}: service not available')
        future = client.call_async(Trigger.Request())
        if not self._wait(future, handle, self.lift_timeout):
            raise Failed(code, f'{what}: timeout')
        if not future.result().success:
            raise Failed(code, f'{what}: {future.result().message}')

    def _settle_localization(self, handle):
        """Let AMCL correct itself while the robot stands still (it only updates on motion)."""
        if self.localization_updates <= 0:
            return
        if not self.nomotion.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('request_nomotion_update not available: '
                                   'looking for the legs with the current AMCL pose')
            return
        before = self._robot_pose()
        for _ in range(self.localization_updates):
            if not self._wait(self.nomotion.call_async(Empty.Request()), handle, self.server_timeout):
                self.get_logger().warn('request_nomotion_update timed out')
                return
            deadline = time.monotonic() + self.localization_period
            while time.monotonic() < deadline:   # one filter update per scan
                if handle.is_cancel_requested:
                    raise Canceled()
                time.sleep(0.05)
        after = self._robot_pose()
        self.get_logger().info(
            f'localization settled: AMCL moved {math.hypot(after[0] - before[0], after[1] - before[1]):.3f} m, '
            f'{math.degrees(_wrap(after[2] - before[2])):+.1f} deg')

    def _spin_to(self, target_yaw, handle):
        _, _, yaw = self._robot_pose()
        delta = _wrap(target_yaw - yaw)
        if abs(delta) < 0.005:
            return
        goal = Spin.Goal(target_yaw=float(delta), time_allowance=_duration_msg(15.0))
        self._run(self.spin, goal, handle, 20.0, DOCK.FAILED_TO_CONTROL, 'spin')

    def _drive(self, distance, handle, what):
        goal = DriveOnHeading.Goal(speed=float(self.approach_speed),
                                   time_allowance=_duration_msg(distance / self.approach_speed + 10.0))
        goal.target.x = float(distance)
        self._run(self.drive, goal, handle, distance / self.approach_speed + 15.0,
                  DOCK.FAILED_TO_CONTROL, what)

    # --------------------------------------------------------------- pick_cart

    def _pick(self, handle):
        result = DockRobot.Result(success=False)
        with self._busy:
            t0 = time.monotonic()

            def feedback(state):
                handle.publish_feedback(DOCK_FB(state=state,
                                                docking_time=_duration_msg(time.monotonic() - t0)))
            try:
                self._do_pick(handle.request, handle, feedback)
            except Canceled:
                result.error_msg = 'canceled'
                handle.canceled()
                self.get_logger().warn('pick_cart canceled')
                return result
            except Failed as exc:
                result.error_code, result.error_msg = exc.code, str(exc)
                handle.abort()
                self.get_logger().error(f'pick_cart failed: {exc}')
                return result
            result.success = True
            handle.succeed()
            self.get_logger().info(f'pick_cart done in {time.monotonic() - t0:.1f} s')
            return result

    def _do_pick(self, req, handle, feedback):
        if req.use_dock_id:
            raise Failed(DOCK.DOCK_NOT_IN_DB,
                         'no cart database: set use_dock_id=false and give dock_pose')
        frame = req.dock_pose.header.frame_id or self.global_frame
        if frame != self.global_frame:
            raise Failed(DOCK.DOCK_NOT_VALID, f'dock_pose must be in "{self.global_frame}"')
        if self.lifted:
            raise Failed(DOCK.DOCK_NOT_VALID, 'already carrying a cart (lift is up)')

        cx, cy = req.dock_pose.pose.position.x, req.dock_pose.pose.position.y
        cyaw = _yaw(req.dock_pose.pose.orientation)
        ux, uy = math.cos(cyaw), math.sin(cyaw)
        self.get_logger().info(f'pick_cart at ({cx:.2f}, {cy:.2f}, {math.degrees(cyaw):.0f} deg)')

        if req.navigate_to_staging_pose:
            feedback(DOCK_FB.NAV_TO_STAGING_POSE)
            staging = PoseStamped()
            staging.header.frame_id = self.global_frame
            staging.header.stamp = self.get_clock().now().to_msg()
            staging.pose.position.x = cx - self.staging_distance * ux
            staging.pose.position.y = cy - self.staging_distance * uy
            staging.pose.orientation = req.dock_pose.pose.orientation
            self._run(self.nav, NavigateToPose.Goal(pose=staging), handle,
                      float(req.max_staging_time), DOCK.FAILED_TO_STAGE, 'navigate to staging pose')

        feedback(DOCK_FB.INITIAL_PERCEPTION)
        self._settle_localization(handle)

        cart = (cx, cy, cyaw)
        attempt = 0          # corrections made; getting closer to pre-entry does not count
        approached = False
        while True:
            # The cart pose comes from its legs in the scan, not from AMCL: even settled,
            # localization is off by up to ~0.18 m, more than the clearance to the legs.
            feedback(DOCK_FB.INITIAL_PERCEPTION)
            cart = self._detect_cart(cart)
            cx, cy, cyaw = cart
            ux, uy = math.cos(cyaw), math.sin(cyaw)
            feedback(DOCK_FB.CONTROLLING)
            rx, ry, ryaw = self._robot_pose()
            along = (rx - cx) * ux + (ry - cy) * uy          # < 0: behind the cart centre
            lateral = -(rx - cx) * uy + (ry - cy) * ux
            yaw_err = _wrap(ryaw - cyaw)
            # Where the robot ends after driving straight to the cart centre: what has to clear
            # the legs is that, not the heading alone.
            lateral_end = lateral - along * math.sin(yaw_err)
            self.get_logger().info(f'alignment {attempt}: along {along:.3f} m, '
                                   f'lateral {lateral:.3f} m (at the centre {lateral_end:.3f} m), '
                                   f'yaw {math.degrees(yaw_err):.1f} deg')
            if abs(lateral_end) <= self.lateral_tolerance and abs(yaw_err) <= self.yaw_tolerance:
                if along >= -(self.pre_entry_distance + 0.05):
                    break
                if approached:
                    raise Failed(DOCK.FAILED_TO_CONTROL,
                                 f'still {-along:.2f} m from the cart after the approach')
                # Aligned but far: get closer (more scan points per leg) and check again.
                self._drive(-along - self.pre_entry_distance, handle, 'approach to pre-entry')
                approached = True
                continue
            if attempt >= self.max_corrections:
                raise Failed(DOCK.FAILED_TO_CONTROL,
                             f'could not align with the cart (lateral {lateral:.3f} m, '
                             f'yaw {math.degrees(yaw_err):.1f} deg)')
            if abs(lateral) > self.lateral_tolerance:
                # Go to a point on the cart axis a bit ahead, staying clear of the legs;
                # back up first if there is no room for it.
                if along + self.correction_lookahead > -self.min_correction_clearance:
                    self._spin_to(cyaw, handle)
                    room = along + self.correction_lookahead + self.min_correction_clearance
                    goal = BackUp.Goal(speed=float(self.backout_speed),
                                       time_allowance=_duration_msg(room / self.backout_speed + 10.0))
                    goal.target.x = -float(room)
                    self._run(self.backup, goal, handle, room / self.backout_speed + 15.0,
                              DOCK.FAILED_TO_CONTROL, 'back up to realign')
                    rx, ry, ryaw = self._robot_pose()
                    along = (rx - cx) * ux + (ry - cy) * uy
                target_along = along + self.correction_lookahead
                qx, qy = cx + target_along * ux, cy + target_along * uy
                self._spin_to(math.atan2(qy - ry, qx - rx), handle)
                self._drive(math.hypot(qx - rx, qy - ry), handle, 'alignment drive')
            self._spin_to(cyaw, handle)
            attempt += 1

        rx, ry, _ = self._robot_pose()
        distance = -((rx - cx) * ux + (ry - cy) * uy)
        if distance <= 0.0:
            raise Failed(DOCK.FAILED_TO_CONTROL, 'robot is already past the cart centre')
        self._drive(distance, handle, 'drive under the cart')
        self._call(self.lift_up, handle, DOCK.FAILED_TO_CONTROL, 'lift_up')

    # --------------------------------------------------------------- drop_cart

    def _drop(self, handle):
        result = UndockRobot.Result(success=False)
        with self._busy:
            t0 = time.monotonic()
            try:
                if not self.lifted:
                    raise Failed(UndockRobot.Result.DOCK_NOT_VALID, 'not carrying a cart (lift is down)')
                self._call(self.lift_down, handle, UndockRobot.Result.FAILED_TO_CONTROL, 'lift_down')
                goal = BackUp.Goal(speed=float(self.backout_speed),
                                   time_allowance=_duration_msg(
                                       self.backout_distance / self.backout_speed + 10.0))
                goal.target.x = -float(self.backout_distance)
                self._run(self.backup, goal, handle,
                          float(handle.request.max_undocking_time),
                          UndockRobot.Result.FAILED_TO_CONTROL, 'back out of the cart')
            except Canceled:
                result.error_msg = 'canceled'
                handle.canceled()
                self.get_logger().warn('drop_cart canceled')
                return result
            except Failed as exc:
                result.error_code, result.error_msg = exc.code, str(exc)
                handle.abort()
                self.get_logger().error(f'drop_cart failed: {exc}')
                return result
            result.success = True
            handle.succeed()
            self.get_logger().info(f'drop_cart done in {time.monotonic() - t0:.1f} s')
            return result


def main(args=None):
    rclpy.init(args=args)
    node = CartHandler()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
