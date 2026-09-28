#!/usr/bin/env python3
"""Publish the measured Go2->local clock offset. Thin rclpy shell; no logic here.

All the arithmetic lives in clock_offset.py, which imports no rclpy so the offline
suite can exercise the real estimator with no ROS, no container and no robot. This
file is I/O and nothing else -- if you find yourself adding a calculation here,
it belongs in the pure module where it can be tested.

WHAT THIS NODE IS FOR
---------------------
It is the instrument, not the calibration source. Until now the offset has been
read by hand with `ros2 topic delay` once per session and pasted into a log; this
node does the same measurement continuously and publishes it, so the value lands
in a bag alongside the data it applies to instead of in a text file beside it.

IT IS NOT THE OFFSET THE LIDAR ADAPTER USES -- see lidar_adapter_node.py. Each
re-stamping node runs its own estimator on its own input topic, deliberately.
"""
from geometry_msgs.msg import PoseStamped
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from go2_adapter.clock_offset import CONVERGED, FAULTED, ClockOffsetEstimator
from go2_adapter.node_runtime import run
from go2_adapter.qos import GO2_INPUT_QOS, GO2_STATUS_QOS
from go2_adapter.restamp import to_seconds

# Published topic names appear as INLINE STRING LITERALS at every create_publisher
# call below, and not as module constants or parameters. This is not style: the
# static safety test (scripts/test_go2_adapter_no_publish_offline.py) requires a
# literal, because any indirection -- a name, a parameter, an f-string -- would let
# the topic be chosen at runtime and would defeat every other check in that test.
# A module-level constant is such an indirection, and the test correctly rejects
# one. Where the mounted yaml also names an output topic, the node compares the
# LIVE publisher's resolved topic_name against it, so there is a single source of
# truth rather than two strings that can drift.


class ClockOffsetNode(Node):

    def __init__(self, **kwargs):
        # **kwargs so tests can pass parameter_overrides= without going through the
        # command line or a global context. A node that can only be configured by
        # process arguments is a node that can only be tested by launching a process.
        super().__init__('go2_clock_offset_node', **kwargs)

        self.declare_parameter('source_topic', '/utlidar/robot_pose')
        self.declare_parameter('window', 100)
        self.declare_parameter('step_threshold_s', 0.5)
        self.declare_parameter('step_consecutive', 10)

        source_topic = self.get_parameter('source_topic').value
        window = int(self.get_parameter('window').value)

        self._est = ClockOffsetEstimator(
            window=window,
            step_threshold=float(self.get_parameter('step_threshold_s').value),
            step_consecutive=int(self.get_parameter('step_consecutive').value),
        )

        self._offset_pub = self.create_publisher(
            Float64, '/go2/clock_offset', GO2_STATUS_QOS)
        self._state_pub = self.create_publisher(
            String, '/go2/clock_offset_state', GO2_STATUS_QOS)
        self._sub = self.create_subscription(
            PoseStamped, source_topic, self._on_pose, GO2_INPUT_QOS)

        self._last_state = None

        # 18.75 Hz measured on /utlidar/robot_pose, so a 100-sample window converges
        # in ~5.3 s. Said out loud at startup because "no output yet" and "no data
        # arriving" look identical from outside, and the difference matters.
        self.get_logger().info(
            'listening on %s; %d samples to converge (~%.1f s at 18.75 Hz)'
            % (source_topic, window, window / 18.75))

    def _on_pose(self, msg):
        # Local arrival time. get_clock() is the system clock unless use_sim_time is
        # set, which it must not be here: the whole point is to compare the robot's
        # notion of time against this machine's.
        arrival_s = self.get_clock().now().nanoseconds / 1e9
        go2_stamp_s = to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec)

        state = self._est.add(arrival_s, go2_stamp_s)

        if state != self._last_state:
            self._announce(state)
            self._last_state = state

        self._state_pub.publish(String(data=state))
        offset = self._est.offset
        if offset is not None:
            self._offset_pub.publish(Float64(data=offset))

    def _announce(self, state):
        """Log a state transition once, at a severity that matches what happened."""
        if state == CONVERGED:
            # Worth an INFO with the number in it: this is the value a session log
            # wants, and it should be greppable out of the node's output.
            self.get_logger().info(
                'CONVERGED: offset = %.6f s (frozen; %d samples)'
                % (self._est.offset, self._est.n_samples))
        elif state == FAULTED:
            self.get_logger().error(
                'FAULTED: clock offset stepped by more than the threshold and stayed '
                'there. The robot probably rebooted. Latched on purpose -- re-arming '
                'automatically would produce a dataset whose stamps are self-consistent '
                'and wrong. Worst residual seen: %.6f s' % self._est.worst_residual)
        else:
            self.get_logger().info('state -> %s' % state)


def main(args=None):
    # Spin/teardown lives in node_runtime.run(); see that module for why SIGTERM
    # needs handling that SIGINT does not.
    return run(ClockOffsetNode, args=args)


if __name__ == '__main__':
    sys.exit(main())
