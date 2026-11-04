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
"""

import math
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
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
        self.up_height = float(self.declare_parameter('up_height', 0.08).value)
        self.speed = float(self.declare_parameter('speed', 0.05).value)
        self.rate_hz = float(self.declare_parameter('rate', 50.0).value)
        self.tolerance = float(self.declare_parameter('tolerance', 0.003).value)
        self.timeout = float(self.declare_parameter('timeout', 5.0).value)

        self._position = None          # last measured lift position [m]
        self._setpoint = 0.0           # last commanded position [m]
        self._motion = threading.Lock()
        self._state_published = None

        state_group = MutuallyExclusiveCallbackGroup()
        service_group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(Float64, 'lift_cmd', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(Bool, 'lift_state', latched)
        self.create_subscription(JointState, 'joint_states', self._joint_states_cb, 10,
                                 callback_group=state_group)
        self.create_service(Trigger, 'lift_up', self._up_cb, callback_group=service_group)
        self.create_service(Trigger, 'lift_down', self._down_cb, callback_group=service_group)

        self.get_logger().info(
            f'Lift on joint "{self.joint_name}": up {self.up_height} m at {self.speed} m/s')

    # ------------------------------------------------------------------ state

    def _joint_states_cb(self, msg):
        try:
            self._position = msg.position[msg.name.index(self.joint_name)]
        except (ValueError, IndexError):
            return
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
            response.success, response.message = self._ramp_to(target, label)
        finally:
            self._motion.release()
            self._publish_state()
        level = self.get_logger().info if response.success else self.get_logger().warn
        level(f'lift_{label}: {response.message}')
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
        # Sim time drives the ramp; a wall-clock guard keeps the call from hanging if /clock stops.
        t0 = self.get_clock().now()
        wall_deadline = time.monotonic() + 3.0 * (ramp_time + self.timeout)
        period = 1.0 / self.rate_hz

        while rclpy.ok():
            elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9
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
