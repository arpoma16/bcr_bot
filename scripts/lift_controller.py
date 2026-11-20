#!/usr/bin/env python3
"""Lift platform services for one bcr_bot (run inside the robot namespace).

  lift_up / lift_down (std_srvs/Trigger)
      Ramp the setpoint on lift_cmd (std_msgs/Float64) at `speed` towards `up_height` / 0,
      then wait until joint_states reports the lift joint within `tolerance`.
      success=false if it does not get there within `timeout`, or if another lift motion is
      already running.
  lift_state (std_msgs/Bool, transient local)
      true when the plate is up (position above half the stroke).

The gz JointPositionController behind lift_cmd is stiff (it jumps 8 cm in ~0.25 s), so the
smooth motion comes from the ramp here.

joint_states is only listened to while it is needed: once at start (where the plate really is)
and during a lift_up / lift_down call. Between motions the node has no subscription to it,
because a steady stream makes a multi-threaded rclpy executor burn CPU even when nothing
happens (~60-90 % of a core per robot measured with the stream always on). The same goes for
/clock: the node runs WITHOUT use_sim_time (see bcr_bot_gz_spawn.launch.py) and gets the
simulation time from the header stamps of joint_states while it moves.
"""

import math
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger


class LiftController(Node):
    def __init__(self):
        super().__init__('lift_controller')
        self.joint_name = self.declare_parameter('joint_name', 'lift_joint').value
        # Lift-only state topic (gz.xacro). `joint_states` also works (all joints, other rate).
        self.state_topic = self.declare_parameter('state_topic', 'lift/position').value
        self.up_height = float(self.declare_parameter('up_height', 0.10).value)
        self.speed = float(self.declare_parameter('speed', 0.05).value)
        self.rate_hz = float(self.declare_parameter('rate', 50.0).value)
        self.tolerance = float(self.declare_parameter('tolerance', 0.003).value)
        self.timeout = float(self.declare_parameter('timeout', 5.0).value)

        self._position = None          # last measured lift position [m]
        self._setpoint = 0.0           # last commanded position [m]
        self._motion = threading.Lock()
        self._state_published = None
        self._readings = 0             # joint_states messages that carried the lift joint
        self._stamp = None             # simulation time [s] of the last one (its header stamp)
        self._state_sub = None         # open only at start and during a motion
        self._state_lock = threading.Lock()

        self._state_group = MutuallyExclusiveCallbackGroup()
        service_group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(Float64, 'lift_cmd', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(Bool, 'lift_state', latched)
        self.create_service(Trigger, 'lift_up', self._up_cb, callback_group=service_group)
        self.create_service(Trigger, 'lift_down', self._down_cb, callback_group=service_group)

        # First reading: where the plate really is. The timer closes the subscription as soon as
        # that reading has arrived (it cancels itself), so the idle node receives no stream.
        # Monotonic clock on purpose: the simulation time can be stopped (the launch starts paused).
        self._open_state()
        self._startup_timer = self.create_timer(
            1.0, self._close_after_first_reading, callback_group=self._state_group,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(
            f'Lift on joint "{self.joint_name}": up {self.up_height} m at {self.speed} m/s')

    # ------------------------------------------------------------------ state

    def _open_state(self):
        with self._state_lock:
            if self._state_sub is None:
                self._state_sub = self.create_subscription(
                    JointState, self.state_topic, self._joint_states_cb, 10,
                    callback_group=self._state_group)

    def _close_state(self):
        with self._state_lock:
            if self._state_sub is not None:
                self.destroy_subscription(self._state_sub)
                self._state_sub = None

    def _close_after_first_reading(self):
        if self._position is None:
            return                      # nothing read yet: keep listening
        self._startup_timer.cancel()
        if not self._motion.locked():   # a motion in progress closes it itself when it ends
            self._close_state()

    def _wait_for_reading(self, timeout=2.0):
        """Wait (wall clock) for a joint_states message that arrives after this call."""
        seen = self._readings
        deadline = time.monotonic() + timeout
        while self._readings == seen:
            if time.monotonic() > deadline:
                return False
            time.sleep(0.02)
        return True

    def _joint_states_cb(self, msg):
        try:
            self._position = msg.position[msg.name.index(self.joint_name)]
        except (ValueError, IndexError):
            return
        self._readings += 1
        stamp = msg.header.stamp
        if stamp.sec or stamp.nanosec:
            self._stamp = stamp.sec + stamp.nanosec * 1e-9
        if self._state_published is None:
            # First reading: start from where the plate really is.
            self._setpoint = self._position
            self._publish_state()

    def _publish_state(self):
        up = self._position is not None and self._position > self.up_height / 2.0
        if up != self._state_published:
            self.state_pub.publish(Bool(data=up))
            self._state_published = up

    # --------------------------------------------------------------- services

    def _up_cb(self, request, response):
        return self._move(self.up_height, 'up', response)

    def _down_cb(self, request, response):
        return self._move(0.0, 'down', response)

    def _move(self, target, label, response):
        if not self._motion.acquire(blocking=False):
            response.success = False
            response.message = 'lift busy: another lift motion is running'
            return response
        try:
            self._open_state()
            if self._position is not None and not self._wait_for_reading():
                response.success = False
                response.message = ('no joint_states received (simulation paused or the bridge '
                                    'is down?)')
            else:
                response.success, response.message = self._ramp_to(target, label)
        finally:
            self._close_state()
            self._motion.release()
            self._publish_state()
        # Two call sites: rclpy raises ValueError if ONE call site logs with different severities
        # (a failed lift followed by a good one used to kill the service callback, so the client
        # never got a response).
        if response.success:
            self.get_logger().info(f'lift_{label}: {response.message}')
        else:
            self.get_logger().warn(f'lift_{label}: {response.message}')
        return response

    def _ramp_to(self, target, label):
        if self._position is None:
            return False, f'no "{self.joint_name}" in joint_states yet'
        if abs(self._position - target) <= self.tolerance:
            self._setpoint = target
            return True, f'already {label} ({self._position:.3f} m)'

        start = self._position
        direction = math.copysign(1.0, target - start)
        ramp_time = abs(target - start) / self.speed
        # Simulation time drives the ramp through the header stamps of joint_states, which are the
        # simulation time (the node runs on wall time and does not subscribe to /clock: at 250 Hz
        # that subscription was most of its CPU). With unstamped messages it falls back to wall
        # time. A wall-clock guard keeps the call from hanging if the messages stop.
        stamped = self._stamp is not None
        clock = (lambda: self._stamp) if stamped else time.monotonic
        t0 = clock()
        wall_deadline = time.monotonic() + 3.0 * (ramp_time + self.timeout)
        period = 1.0 / self.rate_hz

        while rclpy.ok():
            elapsed = clock() - t0
            if elapsed < ramp_time:
                self._setpoint = start + direction * self.speed * elapsed
            else:
                self._setpoint = target
            self.cmd_pub.publish(Float64(data=self._setpoint))

            if elapsed >= ramp_time and abs(self._position - target) <= self.tolerance:
                return True, f'{label} at {self._position:.3f} m in {elapsed:.1f} s'
            if elapsed > ramp_time + self.timeout or time.monotonic() > wall_deadline:
                return False, (f'timeout going {label}: at {self._position:.3f} m, '
                               f'target {target:.3f} m (blocked or overloaded?)')
            time.sleep(period)
        return False, 'shutdown'


def main(args=None):
    rclpy.init(args=args)
    node = LiftController()
    executor = MultiThreadedExecutor(num_threads=3)
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
