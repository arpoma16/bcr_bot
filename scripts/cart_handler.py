#!/usr/bin/env python3
"""Pick and drop a lift cart with Nav2 + the lift (run inside the robot namespace).

  pick_cart (nav2_msgs/action/DockRobot)
      dock_pose = cart centre and heading (frame map; the robot drives in along the cart's +x).
      use_dock_id must be false (there is no cart database). dock_type is ignored except for an
      optional "staging_distance=<metres>" (also in place_cart): the distance between the cart centre
      and the station's staging point, which replaces the staging_distance parameter for that
      request. Each station has its own (it is the nav graph's), so it is not a global constant.
      It must be at least min_staging_distance (the robot cannot stand closer than the rear legs).
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
         cart, lift_up, and reverse staging_distance on the entry axis.
  place_cart (nav2_msgs/action/DockRobot)
      dock_pose is the destination centre in map. With the lift raised, align at
      staging, drive straight to the centre, lower, and reverse to staging.
      Guarded translations temporarily lease the robot footprint from the carry
      adapter and independently monitor the raw scan and odometry deviation.
      Before each straight segment the measured cart offset from the axis must satisfy
      lateral_tolerance / yaw_tolerance (else FAILED_TO_CONTROL); during it the guard measures the
      lateral / yaw / envelope drift from the start pose and heading of the segment (the robot's own
      line, not the cart axis) against straight_lateral_limit, straight_yaw_limit and
      straight_envelope (README, ceilings 0.05 m / 3 deg / 0.06 m). It logs the start context,
      the maxima and a signed drift trace once per segment and reports the measured values on failure.
      place_cart accepts a carried cart up to place_carried_yaw_tolerance (default 3 deg) of yaw
      relative to the robot (the pick tolerance) and 0.25 m of offset.
  drop_cart (nav2_msgs/action/UndockRobot)
      lift_down, then backup by backout_distance to leave the cart where it is.

Error codes reuse DockRobot/UndockRobot: DOCK_NOT_IN_DB (dock_id given), DOCK_NOT_VALID (bad
frame, already/not carrying, busy), FAILED_TO_STAGE (navigation), FAILED_TO_DETECT_DOCK
(legs not found in the scan), FAILED_TO_CONTROL
(alignment, drive, lift, backup). One maneuver at a time; cancelling stops the running Nav2
action. While the cart is up, cart_carry_adapter.py adapts Nav2 (footprint, scan).
"""

import json
import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PolygonStamped, PoseStamped
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
from std_srvs.srv import Empty, SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

import cart_legs
import cart_geometry as geometry

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


class StopShort(Exception):
    """Internal: a forward segment reached the point where it must stop short of an obstacle."""


class PathBlocked(Exception):
    """Internal: the sweep ahead is occupied; _straight stops and waits for it to clear."""


class CartHandler(Node):
    PLACE_ALIGN_ROUNDS = 6      # measure/correct rounds of the place staging alignment

    def __init__(self):
        super().__init__('cart_handler')
        p = self.declare_parameter
        self.global_frame = p('global_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        self.staging_distance = p('staging_distance', 1.8).value      # behind the cart centre [m]
        # Closest staging a request may ask for: rear legs at leg_x from the cart centre + the
        # robot's front (0.45 m, nav2_factory_params footprint) + its 0.05 m padding.
        self.min_staging_distance = p('min_staging_distance', 1.08).value
        self._request_staging = None    # staging_distance of the running request (dock_type)
        self.correction_lookahead = p('correction_lookahead', 0.5).value
        self.min_correction_clearance = p('min_correction_clearance', 1.3).value
        # Accepted when the lateral offset predicted at the cart centre (offset + distance *
        # sin(heading error)) is within lateral_tolerance far from the cart (staging) and within
        # pick_lateral_tolerance at pre-entry, and the heading within yaw_tolerance.
        # Between the legs there are only ~4-9 cm per side once Nav2's footprint padding and its
        # 5 cm costmap cells are counted: with 2.4-2.8 cm at the centre (inside the old 4 cm) the
        # drive under the cart aborted with COLLISION_AHEAD (error 723) in two out of two runs.
        # 1.5 cm was too strict: the spin's +-0.7 deg alone is +-1.6 cm from pre-entry.
        self.pick_lateral_tolerance = p('pick_lateral_tolerance', 0.02).value   # [m]
        # lateral_tolerance also limits the cart's offset when a straight leg starts.
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
        # Reach of a ray to the centre of a leg of the cart just put down (legs are 4 cm boxes):
        # a return below the laser's range_min is accepted only if it points at such a leg.
        self.released_leg_margin = p('released_leg_margin', 0.10).value
        # Forced AMCL updates before looking for the legs (0 disables).
        self.localization_updates = p('localization_updates', 10).value
        self.localization_period = p('localization_period', 0.3).value   # [s] between updates
        self.motion_frame = p('motion_frame', self.base_frame.rsplit('/', 1)[0] + '/odom'
                              if '/' in self.base_frame else 'odom').value
        self.straight_lateral_limit = p('straight_lateral_limit', 0.03).value
        self.straight_yaw_limit = p('straight_yaw_limit', math.radians(2.)).value
        self.straight_envelope = p('straight_envelope', 0.04).value
        self.straight_lookahead = p('straight_lookahead', 0.75).value
        # The leading sweep covers only what is left of the segment plus this margin (never more
        # than straight_lookahead): an obstacle beyond the end point is not in the way, and the
        # robot leaves carts a few cm from the station edge, so a fixed 0.75 m blocked every place
        # next to a wall. 0.05 m covers the 3 cm end-point tolerance with some room.
        self.straight_end_margin = p('straight_end_margin', 0.05).value
        # Returns within this distance of the carried cart's own rectangle are its legs, not
        # obstacles: the outer leg returns sat 0.4 mm inside the rectangle (half_x 0.62), so 1 mm of
        # laser jitter at a leg made the guard cancel a reverse after 4 cm. The sweep still reaches
        # the same distance ahead, measured from the true cart edge.
        self.straight_cart_body_margin = p('straight_cart_body_margin', 0.03).value
        # An occupied sweep stops the motion at once (never debounced: the stop is the safety
        # action) and then holds up to straight_block_wait seconds IN TOTAL per segment for the path
        # to be clear for straight_clear_hold seconds in a row (a person walking by); then it resumes
        # what is left of the segment. Past the budget the segment fails. 0 = fail at once.
        self.straight_block_wait = p('straight_block_wait', 20.0).value
        self.straight_clear_hold = p('straight_clear_hold', 1.0).value
        self.straight_data_timeout = p('straight_data_timeout', 0.3).value
        self.straight_footprint_padding = p('straight_footprint_padding', 0.05).value
        self.place_tolerance_xy = p('place_tolerance_xy', 0.15).value
        self.place_tolerance_yaw = p('place_tolerance_yaw', math.radians(3.)).value
        # Max yaw of the carried cart relative to the robot that place_cart accepts. A pick
        # lifts the cart wherever it sits (aligned within the 3 deg pick tolerance) and
        # placement compensates the measured pose, so this matches the pick tolerance.
        self.place_carried_yaw_tolerance = p('place_carried_yaw_tolerance', math.radians(3.)).value
        # Staging alignment tolerances are in the map frame, so they must cover AMCL's noise (2-4 cm
        # at rest, more after a spin with the cart on) while staying under place_tolerance_xy, which
        # the final placement is still verified against. At 0.05 / 0.03 m the correction loop chased
        # noise: six ~150 deg spins in a row, each moving the robot 10-16 cm for a 5-9 cm error.
        self.place_staging_along_tolerance = p('place_staging_along_tolerance', 0.08).value
        # Map-frame staging uncertainty is distinct from relative drift in odom.
        self.place_staging_lateral_tolerance = p(
            'place_staging_lateral_tolerance', max(0.05, self.straight_lateral_limit)).value
        self.place_staging_compensation_limit = p('place_staging_compensation_limit', 0.10).value
        # place_cart's advance into the station may stop short of an obstacle beyond the target
        # (a table leg a few cm from where the cart's edge ends) when the cart would still end within
        # place_stop_short_limit of the target; it keeps straight_end_margin from the obstacle and
        # triggers the stop stop_short_brake early (cancel latency + braking). The placement is then
        # verified against place_tolerance_xy / place_tolerance_yaw as always.
        self.place_stop_short_limit = p('place_stop_short_limit', 0.10).value
        self.stop_short_brake = p('stop_short_brake', 0.02).value
        if (not math.isfinite(self.place_stop_short_limit)
                or not 0. < self.place_stop_short_limit < self.place_tolerance_xy
                or not math.isfinite(self.stop_short_brake) or not 0. <= self.stop_short_brake <= 0.05):
            raise ValueError('place_stop_short_limit must be in (0, place_tolerance_xy) and '
                             'stop_short_brake in [0, 0.05]')
        for value in (self.place_tolerance_xy, self.place_tolerance_yaw, self.pick_lateral_tolerance,
                      self.place_carried_yaw_tolerance, self.released_leg_margin,
                      self.place_staging_along_tolerance, self.place_staging_lateral_tolerance,
                      self.place_staging_compensation_limit):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('placement tolerances must be finite and positive')
        if max(self.place_staging_along_tolerance,
               self.place_staging_lateral_tolerance) >= self.place_tolerance_xy:
            raise ValueError('staging tolerances must stay below place_tolerance_xy')
        self._motion_fault = False
        self._cart_in_base = None       # last detection, relative to the robot (see _cart_relative)
        self._pending_goal = None
        self._pending_result = None
        self._footprints = {}

        self.lifted = False
        self._lift_state_seen = False
        self._lift_state_revision = 0
        self._lift_lock = threading.Lock()
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
        self.straight_footprint = self.create_client(SetBool, 'cart_straight_footprint',
                                                    callback_group=group)
        for costmap in ('local_costmap', 'global_costmap'):
            self.create_subscription(
                PolygonStamped, f'{costmap}/published_footprint',
                lambda msg, name=costmap: self._footprint_cb(name, msg), 10,
                callback_group=group)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'lift_state', self._lift_state_cb, latched,
                                 callback_group=group)
        # Raw scan: the legs must be visible (scan_filtered hides them while carrying).
        self.create_subscription(LaserScan, 'scan', self._scan_cb, qos_profile_sensor_data,
                                 callback_group=group)
        self.create_service(Trigger, '~/reset_fault', self._reset_fault, callback_group=group)

        for name, action_type, execute in (('pick_cart', DockRobot, self._pick),
                                           ('place_cart', DockRobot, self._place),
                                           ('drop_cart', UndockRobot, self._drop)):
            ActionServer(self, action_type, name, execute_callback=execute,
                         goal_callback=self._goal_cb, cancel_callback=lambda _: CancelResponse.ACCEPT,
                         callback_group=group)
        self.get_logger().info('pick_cart / place_cart / drop_cart ready')

    def _lift_state_cb(self, msg):
        with self._lift_lock:
            if not self._lift_state_seen or self.lifted != msg.data:
                self._lift_state_revision += 1
            self.lifted = msg.data
            self._lift_state_seen = True

    def _motion_terminal(self):
        if self._pending_goal is None:
            return True
        if not self._pending_goal.done():
            return False
        goal = self._pending_goal.result()
        if not goal.accepted:
            return True
        if self._pending_result is None:
            self._pending_result = goal.get_result_async()
        return (self._pending_result.done()
                and self._pending_result.result().status in (4, 5, 6))

    def _reset_fault(self, _request, response):
        response.success = False
        if not self._busy.acquire(blocking=False):
            response.message = 'cannot reset during a cart maneuver'
            return response
        try:
            with self._lift_lock:
                if not self._lift_state_seen:
                    response.message = 'lift_state has not been received'
                    return response
                revision = self._lift_state_revision
            if not self._motion_terminal():
                response.message = 'motion termination is not confirmed'
                return response
            if not self.straight_footprint.wait_for_service(timeout_sec=self.server_timeout):
                response.message = 'footprint service unavailable'
                return response
            # This confirms adapter lift state AND fresh normal footprints from both
            # costmaps, releasing any outstanding lease before clearing the latch.
            self._restore_footprint()
            with self._lift_lock:
                if revision != self._lift_state_revision:
                    response.message = 'lift_state changed during reset; retry after stabilization'
                    return response
                self._motion_fault = False
            response.success = True
            response.message = 'fault cleared; lift_state and both normal footprints confirmed'
            return response
        except Exception as exc:
            response.message = f'cannot confirm normal state: {exc}'
            return response
        finally:
            self._busy.release()

    def _scan_cb(self, msg):
        self._scan = (msg, time.monotonic())

    def _detect_cart(self, cart_guess):
        """Cart pose (x, y, yaw) in the global frame measured from its legs in the scan."""
        self._cart_in_base = None
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
        self._cart_in_base = found
        cart = (rx + c * fx - s * fy, ry + s * fx + c * fy, _wrap(ryaw + fyaw))
        self.get_logger().info(
            f'cart detected from {legs} legs at ({cart[0]:.3f}, {cart[1]:.3f}, '
            f'{math.degrees(cart[2]):.1f} deg), {math.hypot(cart[0] - cart_guess[0], cart[1] - cart_guess[1]):.3f} m '
            f'from the previous estimate')
        return cart

    def _goal_cb(self, _goal):
        # One maneuver at a time (pick and drop share the robot).
        return GoalResponse.REJECT if self._busy.locked() or self._motion_fault else GoalResponse.ACCEPT

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
        self._pending_goal, self._pending_result = send, None
        if not self._wait(send, handle, self.server_timeout) or not send.result().accepted:
            raise Failed(code, f'{what}: goal rejected')
        nav_handle = send.result()
        result = nav_handle.get_result_async()
        self._pending_result = result
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
        self._spin_by(target_yaw - yaw, handle)

    def _spin_by(self, delta, handle):
        delta = _wrap(delta)
        if abs(delta) < 0.005:
            return
        goal = Spin.Goal(target_yaw=float(delta), time_allowance=_duration_msg(15.0))
        self._run(self.spin, goal, handle, 20.0, DOCK.FAILED_TO_CONTROL, 'spin')

    def _cart_relative(self, cart):
        """Robot pose in the cart frame: (along, lateral, yaw error), along < 0 behind the centre.

        Taken from the detection itself (the legs in the scan, in the robot frame) when there is
        one: it involves no AMCL. Mixing the map pose of the cart (found with the AMCL pose of
        the scan) with the AMCL pose read afterwards made the robot jump 3-6 cm and up to 3 deg
        against the cart whenever AMCL updated in between, which now happens with any spin of
        0.05 rad (the pick chased those jumps)."""
        rel, self._cart_in_base = getattr(self, '_cart_in_base', None), None
        if rel is not None:
            fx, fy, fyaw = rel
            c, s = math.cos(fyaw), math.sin(fyaw)
            return -(fx * c + fy * s), fx * s - fy * c, _wrap(-fyaw)
        cx, cy, cyaw = cart
        rx, ry, ryaw = self._robot_pose()
        ux, uy = math.cos(cyaw), math.sin(cyaw)
        return ((rx - cx) * ux + (ry - cy) * uy, -(rx - cx) * uy + (ry - cy) * ux,
                _wrap(ryaw - cyaw))

    def _drive(self, distance, handle, what):
        goal = DriveOnHeading.Goal(speed=float(self.approach_speed),
                                   time_allowance=_duration_msg(distance / self.approach_speed + 10.0))
        goal.target.x = float(distance)
        self._run(self.drive, goal, handle, distance / self.approach_speed + 15.0,
                  DOCK.FAILED_TO_CONTROL, what)

    # ------------------------------------------------------ guarded translations

    def _footprint_cb(self, name, msg):
        pts = msg.polygon.points
        edges = sorted(math.hypot(a.x - b.x, a.y - b.y)
                       for a, b in zip(pts, list(pts[1:]) + list(pts[:1])))
        self._footprints[name] = (edges, time.monotonic())

    def _fresh_stamp(self, stamp):
        age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
        return -0.05 <= age <= self.straight_data_timeout

    def _motion_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.motion_frame, self.base_frame, Time())
        except TransformException as exc:
            raise Failed(DOCK.FAILED_TO_CONTROL, f'no motion TF: {exc}')
        if not self._fresh_stamp(tf.header.stamp):
            raise Failed(DOCK.FAILED_TO_CONTROL, 'stale motion TF')
        t = tf.transform.translation
        return t.x, t.y, _yaw(tf.transform.rotation)

    def _confirm_lift(self, expected, handle):
        deadline = time.monotonic() + self.server_timeout
        while self.lifted != expected:
            if handle.is_cancel_requested:
                raise Canceled()
            if time.monotonic() > deadline:
                raise Failed(DOCK.FAILED_TO_CONTROL, 'lift_state did not confirm the lift operation')
            time.sleep(0.02)

    def _lease_result(self, response, carrying):
        if not response.success:
            raise Failed(DOCK.FAILED_TO_CONTROL, f'footprint lease: {response.message}')
        try:
            data = json.loads(response.message)
            values = (*data['cart_pose'], data['half_x'], data['half_y'],
                      data['robot_half_x'], data['robot_half_y'])
            if (len(data['cart_pose']) != 3 or not all(math.isfinite(v) for v in values)
                    or min(values[3:]) <= 0 or data['carrying'] != carrying):
                raise ValueError('invalid geometry or inconsistent lift_state')
            return data
        except (KeyError, TypeError, ValueError) as exc:
            raise Failed(DOCK.FAILED_TO_CONTROL, f'invalid footprint acknowledgment: {exc}')

    def _restore_footprint(self):
        """Do not relinquish a maneuver until full collision protection returns."""
        started = time.monotonic()
        future = self.straight_footprint.call_async(SetBool.Request(data=False))
        while not future.done() and time.monotonic() - started <= self.straight_data_timeout:
            time.sleep(0.02)
        if not future.done():
            raise Failed(DOCK.FAILED_TO_CONTROL, 'footprint restoration acknowledgment timed out')
        data = self._lease_result(future.result(), self.lifted)
        hx, hy = data['half_x'], data['half_y']
        expected = sorted([2 * (hx + self.straight_footprint_padding)] * 2
                          + [2 * (hy + self.straight_footprint_padding)] * 2)
        deadline = time.monotonic() + 3.
        while True:
            ready = True
            for name in ('local_costmap', 'global_costmap'):
                edges, received = self._footprints.get(name, ([], 0.))
                if (received <= started or len(edges) != 4
                        or any(abs(a - b) > .015 for a, b in zip(edges, expected))):
                    ready = False
            if ready:
                return
            if time.monotonic() >= deadline:
                raise Failed(DOCK.FAILED_TO_CONTROL, 'costmaps did not restore the full footprint')
            time.sleep(0.02)

    def _close_returns_are_cart_legs(self, scan, laser, cart_in_base):
        """True if every scan return below range_min points at a leg of the cart on the floor.

        The laser reports nothing useful closer than range_min, so the guard normally treats
        such a return as an invalid scan. Backing out from a cart just put down passes between
        its legs, closer than that: those returns are expected, and only those. A return that
        does not point at a leg (within released_leg_margin of its centre) is still invalid.
        """
        ox, oy, oyaw = laser
        legs = [cart_legs.transform(cart_in_base, leg) for leg in self.leg_model]
        for i, r in enumerate(scan.ranges):
            if math.isnan(r) or r >= scan.range_min:
                continue
            a = scan.angle_min + i * scan.angle_increment + oyaw
            dx, dy = math.cos(a), math.sin(a)
            for lx, ly in legs:
                along = (lx - ox) * dx + (ly - oy) * dy
                off = abs((ly - oy) * dx - (lx - ox) * dy)
                if 0. <= along <= scan.range_min + self.released_leg_margin \
                        and off <= self.released_leg_margin:
                    break
            else:
                return False
        return True

    def _straight(self, distance, axis_yaw, handle, carrying, released_cart=None,
                  stop_short_within=None):
        """Nav2 supplies velocity control; an independent guard owns cart clearance.

        The footprint lease never authorizes rotation. All checks run before sending
        a goal and at 50 Hz while waiting, including during goal acceptance.
        released_cart: pose of the cart just put down, in the robot frame at the start of a
        segment made without it; the guard then tolerates the returns of its legs below the
        laser's range_min (see _close_returns_are_cart_legs).
        stop_short_within (forward segments only; place_cart's advance): when the sweep finds an
        obstacle beyond the target, instead of failing, advance until straight_end_margin from it
        (stopping stop_short_brake early) if the segment would then fall short of `distance` by at
        most this many metres; the caller verifies the resulting pose. A farther shortfall, or an
        obstacle already inside the margin, fails as always. None keeps the plain behaviour.
        """
        speed = self.approach_speed if distance > 0 else self.backout_speed
        if (stop_short_within is not None
                and (distance < 0 or not math.isfinite(stop_short_within) or stop_short_within <= 0
                     or not math.isfinite(self.stop_short_brake) or self.stop_short_brake < 0)):
            raise Failed(DOCK.FAILED_TO_CONTROL, 'invalid stop-short parameters')
        if (not math.isfinite(distance) or distance == 0 or not 0 < speed <= 0.15
                or not 0 < self.straight_data_timeout <= 0.3
                or not 0 < self.straight_lateral_limit <= 0.05
                or not 0 < self.straight_yaw_limit <= math.radians(3.)
                or not 0 < self.straight_envelope <= 0.06
                or not math.isfinite(self.straight_lookahead) or self.straight_lookahead < 0.75
                or not math.isfinite(self.straight_end_margin)
                or not 0.03 <= self.straight_end_margin <= self.straight_lookahead
                or not math.isfinite(self.straight_cart_body_margin)
                or not 0. <= self.straight_cart_body_margin <= 0.05
                or not math.isfinite(self.straight_block_wait)
                or not 0. <= self.straight_block_wait <= 60.
                or not math.isfinite(self.straight_clear_hold)
                or not 0.2 <= self.straight_clear_hold <= 5.):
            raise Failed(DOCK.FAILED_TO_CONTROL, 'invalid straight-motion safety parameters')
        self._confirm_lift(carrying, handle)
        if not self.straight_footprint.wait_for_service(timeout_sec=self.server_timeout):
            raise Failed(DOCK.FAILED_TO_CONTROL, 'footprint lease service unavailable')
        send = nav_handle = nav_result = None

        def stop_motion():
            """Cancel the goal in flight and confirm it stopped (else latch the fault)."""
            nonlocal send, nav_handle, nav_result
            if nav_handle is None and send is not None:
                if send.done():
                    nav_handle = send.result()
                else:
                    # Retain cancellation even when acceptance arrives after failure.
                    send.add_done_callback(lambda f: f.result().cancel_goal_async()
                                           if f.result().accepted else None)
                    self._motion_fault = True
            if nav_handle is not None and nav_handle.accepted:
                nav_handle.cancel_goal_async()
                terminal = nav_result or nav_handle.get_result_async()
                deadline = time.monotonic() + 2.
                while not terminal.done() and time.monotonic() < deadline:
                    time.sleep(0.02)
                if not terminal.done():
                    self._motion_fault = True
                    self.get_logger().error('motion stop unconfirmed; rejecting further cart goals')

        maxima = None
        trace = []       # (travelled, signed lateral drift, yaw drift) every ~0.1 m, logged at the end
        legs_seen = [0]  # scans with sub-range returns accepted as legs of the cart left behind
        stop_at = [None]    # travelled distance to stop at, short of an obstacle (stop_short_within)

        def metrics_text(lateral, yaw, envelope, travelled):
            return (f'lateral={abs(lateral):.3f}m yaw={abs(math.degrees(yaw)):.3f}° '
                    f'envelope={envelope:.3f}m '
                    f'limit={self.straight_lateral_limit:.3f}m/'
                    f'{math.degrees(self.straight_yaw_limit):.3f}°/'
                    f'{self.straight_envelope:.3f}m travelled={travelled:.3f}m')

        lease_start = time.monotonic()
        lease_future = self.straight_footprint.call_async(SetBool.Request(data=True))
        try:
            if not self._wait(lease_future, handle, self.straight_data_timeout):
                raise Failed(DOCK.FAILED_TO_CONTROL, 'footprint lease acknowledgment timed out')
            data = self._lease_result(lease_future.result(), carrying)
            lease_ok = time.monotonic()
            lease_future = None
            map_pose, origin = self._robot_pose(), self._motion_pose()
            axis = (origin[0], origin[1], origin[2] + _wrap(axis_yaw - map_pose[2]))
            # The cart put down stays where it is: fix it in the odom frame for the guard.
            cart_odom = (geometry.compose(origin, released_cart)
                         if released_cart is not None and not carrying else None)
            # Measure corner excursion of the actual cart, including its offset from
            # the robot, rather than assuming the robot lifted it at its centre.
            cart_at_start = data['cart_pose']
            initial_yaw = geometry.local_pose(origin, axis)[2]
            ox, oy, cart_yaw = cart_at_start
            initial_lateral = ox * math.sin(initial_yaw) + oy * math.cos(initial_yaw)
            initial_cart_yaw = _wrap(initial_yaw + cart_yaw)
            initial_exceeded = []
            if abs(initial_lateral) > self.lateral_tolerance:
                initial_exceeded.append('lateral')
            if abs(initial_cart_yaw) > self.yaw_tolerance:
                initial_exceeded.append('yaw')
            if initial_exceeded:
                raise Failed(
                    DOCK.FAILED_TO_CONTROL,
                    f'initial cart alignment outside tolerance: exceeded={"+".join(initial_exceeded)} '
                    f'lateral={abs(initial_lateral):.3f}m '
                    f'limit={self.lateral_tolerance:.3f}m '
                    f'yaw={abs(math.degrees(initial_cart_yaw)):.3f}° '
                    f'limit={math.degrees(self.yaw_tolerance):.3f}°')
            self.get_logger().info(
                f'straight-motion start: distance={distance:.3f}m '
                f'initial_yaw={math.degrees(initial_yaw):+.3f}° '
                f'initial_cart_yaw={math.degrees(initial_cart_yaw):+.3f}° '
                f'initial_lateral={initial_lateral:+.4f}m cart_offset=({ox:+.3f},{oy:+.3f})m '
                f'cart_yaw={math.degrees(cart_yaw):+.3f}°')
            maxima = {'lateral': 0., 'yaw': 0., 'envelope': 0., 'travelled': 0.}
            expected_edges = sorted([2 * (data['robot_half_x'] + self.straight_footprint_padding)] * 2
                                    + [2 * (data['robot_half_y'] + self.straight_footprint_padding)] * 2)

            def refresh():
                nonlocal lease_ok, lease_future
                now = time.monotonic()
                if lease_future is not None and lease_future.done():
                    if self._lease_result(lease_future.result(), carrying) != data:
                        raise Failed(DOCK.FAILED_TO_CONTROL, 'cart geometry changed during translation')
                    lease_ok = now
                    lease_future = None
                if now - lease_ok > self.straight_data_timeout:
                    raise Failed(DOCK.FAILED_TO_CONTROL, 'footprint lease lost')
                if lease_future is None and now - lease_ok >= 0.1:
                    lease_future = self.straight_footprint.call_async(SetBool.Request(data=True))

            def footprints_ready():
                for name in ('local_costmap', 'global_costmap'):
                    edges, received = self._footprints.get(name, ([], 0.))
                    if (received <= lease_start or time.monotonic() - received > 2.5
                            or len(edges) != 4
                            or any(abs(a - b) > 0.015 for a, b in zip(edges, expected_edges))):
                        return False
                return True

            def guard(check_footprint=True):
                if handle.is_cancel_requested:
                    raise Canceled()
                refresh()
                if self.lifted != carrying:
                    raise Failed(DOCK.FAILED_TO_CONTROL, 'lift state changed during translation')
                pose = self._motion_pose()
                # Drift is measured against the line the robot starts on (its own heading),
                # the same reference as the yaw drift. Against the cart axis, a constant start
                # heading error e would show up as travelled * sin(e) of "lateral drift" although
                # nothing drifted (measured: 1.1 deg over 1.8 m = 36 mm with yaw drift 0.000).
                # The start heading error itself is bounded by the initial alignment check above.
                relative = geometry.local_pose(pose, origin)
                yaw = relative[2]                       # heading change since the start
                # The cart centre traces an offset line if not perfectly centred.
                lateral = relative[1] + ox * math.sin(yaw) + oy * math.cos(yaw) - oy
                yaw_drift = yaw
                cart_error = (relative[0], lateral, yaw_drift)
                _, _, envelope = geometry.deviation_metrics(
                    cart_error, (0., 0., 0.), data['half_x'], data['half_y'])
                travelled = abs(relative[0])
                if not trace or travelled - trace[-1][0] >= 0.1:
                    trace.append((travelled, lateral, yaw_drift))
                maxima['lateral'] = max(maxima['lateral'], abs(lateral))
                maxima['yaw'] = max(maxima['yaw'], abs(yaw_drift))
                maxima['envelope'] = max(maxima['envelope'], envelope)
                maxima['travelled'] = max(maxima['travelled'], travelled)
                if not geometry.deviation_ok(cart_error, (0., 0., 0.),
                                              self.straight_lateral_limit, self.straight_yaw_limit,
                                              data['half_x'], data['half_y'], self.straight_envelope):
                    exceeded = []
                    if abs(lateral) > self.straight_lateral_limit:
                        exceeded.append('lateral')
                    if abs(yaw_drift) > self.straight_yaw_limit:
                        exceeded.append('yaw')
                    if envelope > self.straight_envelope:
                        exceeded.append('envelope')
                    raise Failed(
                        DOCK.FAILED_TO_CONTROL,
                        f'straight-motion deviation exceeded={"+".join(exceeded)} '
                        f'{metrics_text(lateral, yaw_drift, envelope, travelled)}')
                if (self._scan is None or time.monotonic() - self._scan[1] > self.straight_data_timeout
                        or not self._fresh_stamp(self._scan[0].header.stamp)):
                    raise Failed(DOCK.FAILED_TO_CONTROL, 'no fresh raw scan for straight motion')
                scan = self._scan[0]
                if (not scan.ranges or scan.angle_increment <= 0
                        or len(scan.ranges) * scan.angle_increment < 2 * math.pi - 0.05
                        or any(math.isnan(r) for r in scan.ranges)):
                    raise Failed(DOCK.FAILED_TO_CONTROL, 'invalid scan or incomplete angular coverage')
                try:
                    tf = self.tf_buffer.lookup_transform(self.base_frame, scan.header.frame_id, Time())
                except TransformException as exc:
                    raise Failed(DOCK.FAILED_TO_CONTROL, f'no laser TF for straight motion: {exc}')
                laser = cart_legs.laser_offset_from_tf(tf)
                if any(r < scan.range_min for r in scan.ranges):
                    # Below range_min the laser says nothing: invalid, unless it is the legs of
                    # the cart this segment is leaving (every such return must point at a leg).
                    if cart_odom is None or not self._close_returns_are_cart_legs(
                            scan, laser, geometry.local_pose(cart_odom, pose)):
                        raise Failed(DOCK.FAILED_TO_CONTROL, 'invalid scan or incomplete angular coverage')
                    legs_seen[0] += 1
                points = cart_legs.scan_points(scan, laser)
                # Sweep what is left of the segment plus the end margin, not a fixed distance.
                body = self.straight_cart_body_margin
                # Measured from the true cart edge: the strip starts `body` further out (the cart's
                # own legs are inside that margin), so it is `body` shorter to end at the same place.
                lookahead = max(0., min(self.straight_lookahead,
                                        max(0., abs(distance) - travelled) + self.straight_end_margin)
                                - body)
                blocker = geometry.sweep_blocker(points, cart_at_start, data['half_x'] + body,
                                                 data['half_y'] + body, 1 if distance > 0 else -1,
                                                 lookahead)
                if blocker is not None:
                    remaining = max(0., abs(distance) - travelled)
                    # Distance from the cart's leading edge to the obstacle, and how far the cart
                    # can still go keeping straight_end_margin from it.
                    gap = blocker[0] - data['half_x']
                    allowed = gap - self.straight_end_margin
                    shortfall = remaining - allowed
                    if (stop_short_within is None or distance < 0 or allowed < 0.
                            or shortfall > stop_short_within):
                        message = ('obstacle in straight-motion leading sweep at cart-frame '
                                   f'({blocker[0]:+.3f}, {blocker[1]:+.3f}) m, '
                                   f'travelled {travelled:.3f} m')
                        if self.straight_block_wait > 0:
                            raise PathBlocked(message)      # stop, hold, resume or fail (below)
                        raise Failed(DOCK.FAILED_TO_CONTROL, message)
                    target = travelled + allowed
                    if stop_at[0] is None:
                        self.get_logger().info(
                            f'straight-motion: obstacle {gap:.3f} m ahead of the cart edge at '
                            f'cart-frame ({blocker[0]:+.3f}, {blocker[1]:+.3f}) m; stopping '
                            f'{shortfall:.3f} m short of the target, {self.straight_end_margin:.3f} m '
                            'from it')
                    # Keep the most restrictive estimate: scan jitter must not push the stop closer.
                    stop_at[0] = target if stop_at[0] is None else min(stop_at[0], target)
                if stop_at[0] is not None and travelled >= stop_at[0] - self.stop_short_brake:
                    raise StopShort(f'stopped at {travelled:.3f} m of {abs(distance):.3f} m')
                if check_footprint and not footprints_ready():
                    raise Failed(DOCK.FAILED_TO_CONTROL, 'robot footprint acknowledgment lost')

            def monitored_wait(future, timeout):
                deadline = time.monotonic() + timeout
                while not future.done():
                    guard()
                    if time.monotonic() > deadline:
                        raise Failed(DOCK.FAILED_TO_CONTROL, 'straight-motion action timed out')
                    time.sleep(0.02)
                guard()

            def wait_until_clear(reason, used):
                """Hold (the goal is already cancelled) until the sweep has been clear for
                straight_clear_hold s in a row; `used` + this wait may not pass straight_block_wait.
                Every other guard check keeps running meanwhile. Returns the total waited."""
                started = time.monotonic()
                clear_since = None
                self.get_logger().info(
                    f'straight-motion: path blocked ({reason}); holding up to '
                    f'{self.straight_block_wait - used:.0f} s for it to clear')
                while True:
                    try:
                        guard()
                        now = time.monotonic()
                        if clear_since is None:
                            clear_since = now
                        if now - clear_since >= self.straight_clear_hold:
                            self.get_logger().info(
                                f'straight-motion: path clear after {now - started:.1f} s; resuming')
                            return used + now - started
                    except PathBlocked as still_blocked:
                        clear_since = None
                        reason = str(still_blocked)
                    if used + time.monotonic() - started > self.straight_block_wait:
                        raise Failed(DOCK.FAILED_TO_CONTROL,
                                     f'path still blocked after {self.straight_block_wait:.0f} s: {reason}')
                    time.sleep(0.02)

            client = self.drive if distance > 0 else self.backup
            waited, first = 0., True
            while True:
                try:
                    # Costmaps acknowledge the actual footprint, not merely service receipt.
                    deadline = time.monotonic() + 3.
                    while not footprints_ready():
                        guard(check_footprint=False)
                        if time.monotonic() > deadline:
                            raise Failed(DOCK.FAILED_TO_CONTROL,
                                         'costmaps did not acknowledge robot footprint')
                        time.sleep(0.02)
                    guard()
                    if not client.wait_for_server(timeout_sec=self.server_timeout):
                        raise Failed(DOCK.FAILED_TO_CONTROL, 'straight-motion action server unavailable')
                    guard()
                    # The whole segment at first; after a hold, what is left of it.
                    command = distance
                    if not first:
                        command = distance - geometry.local_pose(self._motion_pose(), origin)[0]
                        if abs(command) <= 0.03:
                            break           # it had already got there: the endpoint check below
                    first = False
                    timeout = abs(command) / speed + 15.
                    goal_type = DriveOnHeading if distance > 0 else BackUp
                    goal = goal_type.Goal(speed=float(speed), time_allowance=_duration_msg(timeout - 5.))
                    goal.target.x = float(command)
                    send = client.send_goal_async(goal)
                    self._pending_goal, self._pending_result = send, None
                    monitored_wait(send, self.server_timeout)
                    nav_handle = send.result()
                    if not nav_handle.accepted:
                        raise Failed(DOCK.FAILED_TO_CONTROL, 'straight-motion goal rejected')
                    nav_result = nav_handle.get_result_async()
                    self._pending_result = nav_result
                    monitored_wait(nav_result, timeout)
                    outcome = nav_result.result()
                    if outcome.status != 4:
                        raise Failed(DOCK.FAILED_TO_CONTROL,
                                     f'straight motion failed (status {outcome.status}, '
                                     f'error_code {getattr(outcome.result, "error_code", "?")})')
                    break
                except PathBlocked as blocked:
                    stop_motion()
                    send = nav_handle = nav_result = None
                    self._pending_goal = self._pending_result = None
                    # Diagnostics: where the robot stopped (odom along the segment, map pose), to
                    # compare with where it is after the hold and where the segment ends.
                    self.get_logger().info(
                        'straight-motion: stopped at odom along '
                        f'{geometry.local_pose(self._motion_pose(), origin)[0]:.3f} m of '
                        f'{distance:.3f} m, map pose {self._pose_text()}')
                    waited = wait_until_clear(str(blocked), waited)
                    here = geometry.local_pose(self._motion_pose(), origin)[0]
                    self.get_logger().info(
                        f'straight-motion: resuming with {distance - here:.3f} m left (odom along '
                        f'{here:.3f} m), map pose {self._pose_text()}')
            along, _, _ = geometry.local_pose(self._motion_pose(), origin)
            self.get_logger().info(
                f'straight-motion: ended at odom along {along:.3f} m of {distance:.3f} m, '
                f'map pose {self._pose_text()}')
            if abs(along - distance) > 0.03:
                raise Failed(DOCK.FAILED_TO_CONTROL, 'straight-motion endpoint outside 3 cm tolerance')
        except Exception as exc:
            stop_motion()
            if isinstance(exc, StopShort):
                # The goal was cancelled above and its stop confirmed (or the fault latched); the
                # caller verifies where the cart ended up, so no endpoint check here.
                self.get_logger().info(f'straight-motion: {exc}')
                return
            if isinstance(exc, (Failed, Canceled)):
                raise
            raise Failed(DOCK.FAILED_TO_CONTROL, f'straight-motion guard failed: {exc}') from exc
        finally:
            if maxima is not None:
                self.get_logger().info(
                    'straight-motion maxima: '
                    + metrics_text(maxima['lateral'], maxima['yaw'],
                                   maxima['envelope'], maxima['travelled']))
                if legs_seen[0]:
                    self.get_logger().info(
                        f'straight-motion: {legs_seen[0]} scans with returns below range_min '
                        'accepted as legs of the cart just put down')
                self.get_logger().info(
                    'straight-motion trace (travelled: signed lateral/yaw drift): '
                    + ' '.join(f'{t:.1f}m:{lat * 1000:+.0f}mm/{math.degrees(y):+.2f}°'
                               for t, lat, y in trace))
            try:
                self._restore_footprint()
            except Exception as exc:
                self._motion_fault = True
                raise Failed(DOCK.FAILED_TO_CONTROL, f'footprint restoration unconfirmed: {exc}') from exc

    # --------------------------------------------------------------- pick_cart

    def _pick(self, handle):
        return self._dock_action(handle, self._do_pick, 'pick_cart')

    def _place(self, handle):
        return self._dock_action(handle, self._do_place, 'place_cart')

    def _pose_text(self):
        x, y, yaw = self._robot_pose()
        return f'({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f} deg)'

    def _staging(self):
        """Staging distance of the running request: its dock_type's, else the parameter."""
        return getattr(self, '_request_staging', None) or self.staging_distance

    def _use_request_staging(self, req):
        """Read "staging_distance=<m>" from dock_type; no value keeps the parameter."""
        self._request_staging = None
        text = str(getattr(req, 'dock_type', '') or '').strip()
        if not text:
            return
        key, _, value = text.partition('=')
        try:
            distance = float(value)
        except ValueError:
            distance = math.nan
        if key.strip() != 'staging_distance' or not math.isfinite(distance):
            raise Failed(DOCK.DOCK_NOT_VALID,
                         f'dock_type must be "staging_distance=<metres>", got "{text}"')
        if distance < self.min_staging_distance:
            raise Failed(DOCK.DOCK_NOT_VALID,
                         f'staging_distance {distance:.2f} m is closer than the robot can stand '
                         f'to the cart legs (min_staging_distance {self.min_staging_distance:.2f} m)')
        self._request_staging = distance
        self.get_logger().info(f'staging_distance for this request: {distance:.2f} m')

    def _dock_action(self, handle, execute, name):
        result = DockRobot.Result(success=False)
        with self._busy:
            t0 = time.monotonic()

            def feedback(state):
                handle.publish_feedback(DOCK_FB(state=state,
                                                docking_time=_duration_msg(time.monotonic() - t0)))
            try:
                self._use_request_staging(handle.request)
                execute(handle.request, handle, feedback)
            except Canceled:
                result.error_msg = 'canceled'
                handle.canceled()
                self.get_logger().warn(f'{name} canceled')
                return result
            except Failed as exc:
                result.error_code, result.error_msg = exc.code, str(exc)
                handle.abort()
                self.get_logger().error(f'{name} failed: {exc}')
                return result
            finally:
                self._request_staging = None
            result.success = True
            handle.succeed()
            self.get_logger().info(f'{name} done in {time.monotonic() - t0:.1f} s')
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
        self.get_logger().info(f'pick_cart at ({cx:.2f}, {cy:.2f}, {math.degrees(cyaw):.0f} deg)')

        if req.navigate_to_staging_pose:
            feedback(DOCK_FB.NAV_TO_STAGING_POSE)
            staging = PoseStamped()
            staging.header.frame_id = self.global_frame
            staging.header.stamp = self.get_clock().now().to_msg()
            staging.pose.position.x, staging.pose.position.y, _ = geometry.staging_pose(
                (cx, cy, cyaw), self._staging())
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
            feedback(DOCK_FB.CONTROLLING)
            along, lateral, yaw_err = self._cart_relative(cart)      # along < 0: behind the centre
            # Where the robot ends after driving straight to the cart centre: what has to clear
            # the legs is that, not the heading alone.
            lateral_end = lateral - along * math.sin(yaw_err)
            self.get_logger().info(f'alignment {attempt}: along {along:.3f} m, '
                                   f'lateral {lateral:.3f} m (at the centre {lateral_end:.3f} m), '
                                   f'yaw {math.degrees(yaw_err):.1f} deg')
            # Far from the cart only a coarse check makes sense: the spin leaves +-0.7 deg, which is
            # +-2.4 cm at the centre from 2 m but +-1.6 cm from 1.3 m, so the strict limit (the one
            # that keeps the footprint off the legs) is demanded at pre-entry, not at staging.
            near = along >= -(self.pre_entry_distance + 0.05)
            lateral_limit = self.pick_lateral_tolerance if near else self.lateral_tolerance
            if abs(lateral_end) <= lateral_limit and abs(yaw_err) <= self.yaw_tolerance:
                if near:
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
            # Everything below is relative to the cart axis (the robot heading is `heading` in the
            # cart frame): no absolute yaw is read again, so AMCL cannot move the target meanwhile.
            heading = yaw_err
            if abs(lateral) > lateral_limit:
                # Go to a point on the cart axis a bit ahead, staying clear of the legs;
                # back up first if there is no room for it.
                if along + self.correction_lookahead > -self.min_correction_clearance:
                    self._spin_by(-heading, handle)
                    heading = 0.
                    room = along + self.correction_lookahead + self.min_correction_clearance
                    goal = BackUp.Goal(speed=float(self.backout_speed),
                                       time_allowance=_duration_msg(room / self.backout_speed + 10.0))
                    goal.target.x = -float(room)
                    self._run(self.backup, goal, handle, room / self.backout_speed + 15.0,
                              DOCK.FAILED_TO_CONTROL, 'back up to realign')
                bearing = math.atan2(-lateral, self.correction_lookahead)
                self._spin_by(bearing - heading, handle)
                heading = bearing
                self._drive(math.hypot(self.correction_lookahead, lateral), handle, 'alignment drive')
            self._spin_by(-heading, handle)          # back onto the cart axis
            attempt += 1

        distance = -along
        if distance <= 0.0:
            raise Failed(DOCK.FAILED_TO_CONTROL, 'robot is already past the cart centre')
        self._drive(distance, handle, 'drive under the cart')
        self._call(self.lift_up, handle, DOCK.FAILED_TO_CONTROL, 'lift_up')
        self._confirm_lift(True, handle)
        self._straight(-self._staging(), cyaw, handle, True)

    # -------------------------------------------------------------- place_cart

    def _place_target(self, cart, handle):
        if not self.straight_footprint.wait_for_service(timeout_sec=self.server_timeout):
            raise Failed(DOCK.FAILED_TO_CONTROL, 'cart geometry service unavailable')
        future = self.straight_footprint.call_async(SetBool.Request(data=False))
        if not self._wait(future, handle, self.straight_data_timeout):
            raise Failed(DOCK.FAILED_TO_CONTROL, 'cart geometry acknowledgment timed out')
        data = self._lease_result(future.result(), True)
        local = data['cart_pose']
        if (not data.get('measured') or abs(local[2]) > self.place_carried_yaw_tolerance
                or math.hypot(local[0], local[1]) > .25):
            seen = (f'{math.hypot(local[0], local[1]):.3f} m / {math.degrees(abs(local[2])):.2f} deg'
                    if data.get('measured') else 'not measured')
            raise Failed(DOCK.DOCK_NOT_VALID,
                         'place requires a measured cart within 25 cm / '
                         f'{math.degrees(self.place_carried_yaw_tolerance):.1f} deg of robot '
                         f'(cart: {seen})')
        target = geometry.placement_robot_pose(cart, local)
        nominal = geometry.staging_pose(cart, self._staging())
        corrected = geometry.staging_pose(target, self._staging())
        if (math.hypot(corrected[0] - nominal[0], corrected[1] - nominal[1])
                > self.place_staging_compensation_limit):
            raise Failed(DOCK.DOCK_NOT_VALID, 'cart offset exceeds staging compensation limit')
        self._placement_geometry = local
        return target

    def _verify_placement(self, cart):
        actual = geometry.compose(self._robot_pose(), self._placement_geometry)
        if (math.hypot(actual[0] - cart[0], actual[1] - cart[1]) > self.place_tolerance_xy
                or abs(_wrap(actual[2] - cart[2])) > self.place_tolerance_yaw):
            raise Failed(DOCK.FAILED_TO_CONTROL, 'cart outside destination tolerance; refusing to lower')

    def _align_place(self, cart, handle):
        staging = geometry.staging_pose(cart, self._staging())
        # Small staging corrections retain the full cart footprint. Never use the
        # leased footprint to rotate, or drive a lateral correction inside a station.
        limits = (f'limits {self.place_staging_along_tolerance:.3f} m along / '
                  f'{self.place_staging_lateral_tolerance:.3f} m lateral')
        # AMCL only updates when the robot moves, so after Nav2 stops (and after a spin) its
        # estimate lags: measured 17-19 cm / up to 8 deg off right after arriving, 4-12 cm / 0.3 deg
        # after 10 updates at rest. These tolerances are a few cm, so settle before comparing.
        # The check that counts is the one made already facing the cart (the final spin shifted the
        # pose by up to 6 cm after a check made while facing the staging point). So every round
        # settles and measures; a measurement within tolerance while not yet facing the cart only
        # triggers the final spin, and the next round measures again in that condition.
        # An error ALONG the axis, with the robot already facing it and the lateral error in
        # tolerance, is corrected by a short guarded straight move without turning around: the
        # spin toward the staging point and back was what moved the robot more than it corrected.
        # (No controller keeps a straight move on the line, so lateral errors still need the
        # spin + drive below.)
        facing, pose = False, (0., 0.)
        for attempt in range(self.PLACE_ALIGN_ROUNDS):
            self._settle_localization(handle)
            rx, ry, ryaw = self._robot_pose()
            pose = geometry.local_pose((rx, ry, cart[2]), staging)
            distance = math.hypot(pose[0], pose[1])
            self.get_logger().info(
                f'place alignment {attempt}{" (facing the cart)" if facing else ""}: '
                f'robot ({rx:.3f}, {ry:.3f}) staging ({staging[0]:.3f}, {staging[1]:.3f}), '
                f'along {pose[0]:+.3f} m, lateral {pose[1]:+.3f} m, {limits}')
            if distance > 0.5:
                raise Failed(DOCK.FAILED_TO_STAGE, 'place_cart must start within 0.5 m of staging')
            if (abs(pose[0]) <= self.place_staging_along_tolerance
                    and abs(pose[1]) <= self.place_staging_lateral_tolerance):
                if facing:
                    return
                self._spin_to(cart[2], handle)
                facing = True
                continue
            if (abs(pose[1]) <= self.place_staging_lateral_tolerance
                    and abs(_wrap(ryaw - cart[2])) <= self.yaw_tolerance):
                # pose[0] > 0: the robot is ahead of the staging (toward the cart): back up.
                self._straight(-pose[0], cart[2], handle, True)
                facing = True
                continue
            facing = False
            self._spin_to(math.atan2(staging[1] - ry, staging[0] - rx), handle)
            self._drive(distance, handle, 'align at destination staging')
        raise Failed(DOCK.FAILED_TO_STAGE,
                     'place_cart alignment left the staging position: '
                     f'along {pose[0]:+.3f} m, lateral {pose[1]:+.3f} m, {limits}')

    def _do_place(self, req, handle, feedback):
        if not self.lifted:
            raise Failed(DOCK.DOCK_NOT_VALID, 'not carrying a cart (lift is down)')
        if req.use_dock_id:
            raise Failed(DOCK.DOCK_NOT_IN_DB, 'no cart database: give dock_pose')
        if req.dock_pose.header.frame_id != self.global_frame:
            raise Failed(DOCK.DOCK_NOT_VALID, f'dock_pose must be in "{self.global_frame}"')
        pose = req.dock_pose.pose
        cart = pose.position.x, pose.position.y, _yaw(pose.orientation)
        q = pose.orientation
        if (not all(math.isfinite(v) for v in (*cart, q.x, q.y, q.z, q.w))
                or abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.) > 0.01):
            raise Failed(DOCK.DOCK_NOT_VALID, 'dock_pose must contain a finite pose and unit quaternion')
        feedback(DOCK_FB.CONTROLLING)
        target = self._place_target(cart, handle)
        self._align_place(target, handle)
        # The start check measures the CARRIED cart's yaw against this axis, so it is the cart's
        # destination axis, not the robot's target yaw (that one already includes the yaw the cart
        # was lifted with: 1.9 deg of it plus the spin's error read 3.4 deg and aborted the place).
        self._straight(self._staging(), cart[2], handle, True,
                       stop_short_within=self.place_stop_short_limit)
        self._verify_placement(cart)
        self._call(self.lift_down, handle, DOCK.FAILED_TO_CONTROL, 'lift_down')
        self._confirm_lift(False, handle)
        # The cart now stands where the robot carried it: its legs are what the robot leaves between.
        self._straight(-self._staging(), target[2], handle, False,
                       released_cart=self._placement_geometry)

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
