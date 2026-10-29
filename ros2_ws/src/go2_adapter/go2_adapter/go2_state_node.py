#!/usr/bin/env python3
"""Publish the Go2 -> hexapod feedback channel. Thin rclpy shell; no logic here.

Plan: phase1_go2_feedback_plan.md (D4, D5, D6, D8, D9; schema v1 in §3).

    /go2/odom          nav_msgs/Odometry       /utlidar/robot_odom, restamped, 20 Hz
    /go2/imu           sensor_msgs/Imu         /utlidar/imu, restamped, 50 Hz
    /go2/robot_status  DiagnosticArray         schema v1, 5 Hz on a TIMER

Every decision -- motion, faults, levels, the echo -- is in state_core.py and is
tested offline. This file subscribes, re-stamps, throttles and publishes.

READ-ONLY, MECHANICALLY: this node lives in go2_adapter, so the static safety test
(scripts/test_go2_adapter_no_publish_offline.py) checks that every publisher here
is a literal /go2/ topic. It subscribes to the robot and to the command bridge's
status; it never commands anything. Handset activity reaches it only through the
bridge's YIELDED state.

FRAMES (D6): every frame_id the robot reports is prefixed with "go2/". Both robots
call their frames odom/base_link; the prefix keeps them apart. No /tf is published.

STAMPS (D9): /go2/odom and /go2/imu carry the Go2's stamp moved onto this host's
clock by a per-topic minimum-delay estimator (clock_offset.py). Nothing is published
on either until its estimator has converged, and a clock step stops both and is
reported as a fault -- an uncorrected stamp is a wrong stamp. /go2/robot_status is
stamped with this host's clock at composition time.

COVARIANCE (D8): /utlidar/robot_odom reports all-zero covariance (v1.1.11). The
zeros pass through untouched and the status says odom_covariance_known: false.
Inventing numbers here would be a fabricated measurement.
"""
import copy
import json
import sys

from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from unitree_go.msg import LowState, SportModeState

from go2_adapter.clock_offset import CONVERGED, FAULTED, ClockOffsetEstimator
from go2_adapter.node_runtime import run
from go2_adapter.qos import GO2_INPUT_QOS, GO2_OUTPUT_QOS, GO2_STATUS_QOS
from go2_adapter.restamp import corrected, to_seconds
from go2_adapter.state_core import (DEFAULTS, Go2StateCore, Throttle)

HARDWARE_ID = 'B42N6000PBCD7AG9'    # the professor's Go2, from its own serial report
FRAME_PREFIX = 'go2/'


def prefixed(frame_id):
    # An empty frame is kept visibly empty rather than guessed at.
    return FRAME_PREFIX + (frame_id if frame_id else 'unset')


class Go2StateNode(Node):

    def __init__(self, **kwargs):
        super().__init__('go2_state_node', **kwargs)

        overrides = {k: float(self.declare_parameter(k, float(v)).value) for k, v in DEFAULTS.items()}
        odom_rate = float(self.declare_parameter('odom_rate_hz', 20.0).value)
        imu_rate = float(self.declare_parameter('imu_rate_hz', 50.0).value)
        status_rate = float(self.declare_parameter('status_rate_hz', 5.0).value)
        window = int(self.declare_parameter('clock_window', 100).value)
        step_s = float(self.declare_parameter('step_threshold_s', 0.5).value)
        step_n = int(self.declare_parameter('step_consecutive', 10).value)

        self._core = Go2StateCore(overrides)       # ConfigError is fatal, by design
        self._odom_throttle = Throttle(odom_rate)
        self._imu_throttle = Throttle(imu_rate)
        self._odom_clock = ClockOffsetEstimator(window, step_s, step_n)
        self._imu_clock = ClockOffsetEstimator(window, step_s, step_n)

        # Literal topics at every create_publisher: the safety test requires it.
        self._odom_pub = self.create_publisher(Odometry, '/go2/odom', GO2_OUTPUT_QOS)
        self._imu_pub = self.create_publisher(Imu, '/go2/imu', GO2_OUTPUT_QOS)
        self._status_pub = self.create_publisher(DiagnosticArray, '/go2/robot_status', GO2_STATUS_QOS)

        self.create_subscription(SportModeState, '/sportmodestate', self._on_sport, GO2_INPUT_QOS)
        self.create_subscription(LowState, '/lowstate', self._on_low, GO2_INPUT_QOS)
        self.create_subscription(Odometry, '/utlidar/robot_odom', self._on_odom, GO2_INPUT_QOS)
        self.create_subscription(Imu, '/utlidar/imu', self._on_imu, GO2_INPUT_QOS)
        self.create_subscription(String, '/go2/cmd_bridge/status', self._on_bridge, GO2_STATUS_QOS)
        self.create_timer(1.0 / status_rate, self._tick)

        self._last_summary = None
        self.n_published = {'odom': 0, 'imu': 0, 'status': 0}
        self.get_logger().info(
            'go2_state_node up: odom %.0f Hz, imu %.0f Hz, status %.0f Hz; motion params %s'
            % (odom_rate, imu_rate, status_rate,
               json.dumps({k: self._core.p[k] for k in ('v_mov', 'w_mov', 'd_mov', 'a_mov', 'window_s')})))

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------- inputs ---
    def _on_sport(self, m):
        self._core.on_sport(self._now(), m.error_code, m.velocity[0], m.velocity[1], m.yaw_speed,
                            m.position[0], m.position[1], m.imu_state.rpy[2])

    def _on_low(self, m):
        ms = m.motor_state
        # Indices 6 and 9 are RR_hip and RL_hip, as in go2_cmd_bridge and go2_thermal_watch.
        self._core.on_low(self._now(), [ms[i].mode for i in range(12)],
                          ms[6].temperature, ms[9].temperature)

    def _on_bridge(self, m):
        try:
            st = json.loads(m.data)
        except ValueError:
            st = None
        self._core.on_bridge(self._now(), st)

    def _restamped(self, msg, est, throttle):
        """Feed the estimator; return a re-stamped, re-framed copy, or None."""
        now = self._now()
        est.add(now, to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec))
        if est.offset is None or not throttle.allow(now):
            return None
        out = copy.deepcopy(msg)
        out.header.stamp.sec, out.header.stamp.nanosec = corrected(
            msg.header.stamp.sec, msg.header.stamp.nanosec, est.offset)
        out.header.frame_id = prefixed(msg.header.frame_id)
        return out

    def _on_odom(self, m):
        out = self._restamped(m, self._odom_clock, self._odom_throttle)
        if out is not None:
            out.child_frame_id = prefixed(m.child_frame_id)
            self._odom_pub.publish(out)
            self.n_published['odom'] += 1

    def _on_imu(self, m):
        out = self._restamped(m, self._imu_clock, self._imu_throttle)
        if out is not None:
            self._imu_pub.publish(out)
            self.n_published['imu'] += 1

    # --------------------------------------------------------------- tick ---
    def _clock_state(self):
        states = (self._odom_clock.state, self._imu_clock.state)
        if FAULTED in states:
            return FAULTED
        return CONVERGED if all(s == CONVERGED for s in states) else 'CONVERGING'

    def _tick(self):
        self._core.on_clock(self._odom_clock.offset, self._clock_state())
        now_msg = self.get_clock().now().to_msg()
        level, message, values = self._core.status(self._now())

        st = DiagnosticStatus()
        # `byte` is a 1-length bytes in Humble's rclpy and an int in older ones;
        # assigning the wrong kind raises. Ask the message rather than the distro.
        st.level = bytes([level]) if isinstance(DiagnosticStatus().level, bytes) else level
        st.name = 'go2'
        st.message = message
        st.hardware_id = HARDWARE_ID
        st.values = [KeyValue(key=k, value=v) for k, v in values]
        arr = DiagnosticArray()
        arr.header.stamp = now_msg
        arr.status = [st]
        self._status_pub.publish(arr)
        self.n_published['status'] += 1

        summary = (level, dict(values)['motion'], dict(values)['fault'])
        if summary != self._last_summary:
            # TWO call sites on purpose. rclpy caches severity per call site and raises
            # "Logger severity cannot be changed between calls" if one line logs at both
            # -- the first version did, and died on its first fault -> healthy edge
            # (test_go2_state_integration_rosonly.py, 29 Oct).
            if level < 2:
                self.get_logger().info('status level %d: %s' % (level, message))
            else:
                self.get_logger().warning('status level %d: %s' % (level, message))
            self._last_summary = summary


def main(args=None):
    return run(Go2StateNode, args=args)


if __name__ == '__main__':
    sys.exit(main())
