#!/usr/bin/env python3
"""Publish /go2/link_ok: is the Go2 still delivering data?

WHY A LIVENESS FLAG AT ALL
--------------------------
The hexapod side is the one that moves. Anything downstream that acts on Go2
perception needs to know when that perception stopped arriving, and it needs to
know it as a message it can subscribe to -- not by noticing that a topic went
quiet, which is exactly the kind of absence that software fails to notice.

WHY IT WATCHES THE POSE AND NOT THE CLOUD
-----------------------------------------
/utlidar/robot_pose is MEASURED at 18.75 Hz (18.69-18.81, sd 0.0006-0.0011 s over
a 100-sample window, session 3) and its payload is ~80 bytes. /utlidar/cloud is
the far more interesting topic and a far worse liveness probe: subscribing to it
costs a large multiple of the bandwidth to learn one boolean, and its actual
delivery rate has never been measured, so no defensible timeout could be chosen
for it. A cheap channel with a known, tight rate is the right instrument.

LIMIT, AND IT IS A REAL ONE: this reports that the LiDAR/odometry participant is
still publishing. It is NOT a statement about the robot's health, about the motors,
or about any other topic. A robot that has thermally shut down its motors keeps
publishing /utlidar/robot_pose -- session 7 measured exactly that, with all twelve
motors disabled and state still streaming. Tagged as inference that pose liveness
generalises to link liveness; it does not generalise to robot liveness at all.

WHY IT PUBLISHES CONTINUOUSLY
-----------------------------
On a timer, not on message arrival. A flag that is only published when data arrives
cannot report that data stopped arriving -- the failure mode silences the very
channel that is supposed to announce it.
"""
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from geometry_msgs.msg import PoseStamped

from go2_adapter.node_runtime import run
from go2_adapter.qos import GO2_INPUT_QOS, GO2_STATUS_QOS

# Inline literal at create_publisher; see clock_offset_node.py for why.


class LinkWatchdogNode(Node):

    def __init__(self, **kwargs):
        super().__init__('go2_link_watchdog_node', **kwargs)

        self.declare_parameter('source_topic', '/utlidar/robot_pose')
        self.declare_parameter('stale_timeout_s', 1.0)
        self.declare_parameter('output_topic', '/go2/link_ok')
        self.declare_parameter('publish_rate_hz', 5.0)

        source_topic = self.get_parameter('source_topic').value
        self._stale_timeout_s = float(self.get_parameter('stale_timeout_s').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if self._stale_timeout_s <= 0.0 or publish_rate_hz <= 0.0:
            raise ValueError(
                'stale_timeout_s and publish_rate_hz must both be > 0; got %r and %r'
                % (self._stale_timeout_s, publish_rate_hz))

        self._pub = self.create_publisher(Bool, '/go2/link_ok', GO2_STATUS_QOS)
        configured = self.get_parameter('output_topic').value
        if configured != self._pub.topic_name:
            raise ValueError(
                'config declares output_topic=%r but this node publishes to %r. '
                'Change the inline literal and the yaml together, or neither.'
                % (configured, self._pub.topic_name))
        self._sub = self.create_subscription(
            PoseStamped, source_topic, self._on_msg, GO2_INPUT_QOS)
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._tick)

        # None, not 0.0, and not "now". Starting at 0.0 would make the link look
        # infinitely stale, which is at least safe; starting at "now" would make it
        # look healthy for one timeout period before any data arrived, which is not.
        # None means "nothing has ever arrived" and is reported as its own state.
        self._last_msg_s = None
        self._last_ok = None
        self._n_msgs = 0

        # A timeout of 1.0 s against a measured 18.75 Hz stream tolerates ~19 missed
        # messages. Stated in those terms because "1 second" hides how loose it is.
        self.get_logger().info(
            'watching %s; stale after %.2f s (~%.0f missed messages at 18.75 Hz), '
            'publishing %s at %.1f Hz'
            % (source_topic, self._stale_timeout_s,
               self._stale_timeout_s * 18.75, self._pub.topic_name, publish_rate_hz))

    def _on_msg(self, _msg):
        # Arrival time, deliberately NOT header.stamp. The Go2's stamps are ~1396 s
        # behind this machine's clock, so comparing a stamp against local now() would
        # report every message as 23 minutes stale. Liveness is a question about
        # delivery, which is what arrival time measures.
        self._last_msg_s = self.get_clock().now().nanoseconds / 1e9
        self._n_msgs += 1

    def _tick(self):
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self._last_msg_s is None:
            ok = False
            age = None
        else:
            age = now_s - self._last_msg_s
            ok = age <= self._stale_timeout_s

        self._pub.publish(Bool(data=ok))

        if ok != self._last_ok:
            if ok:
                self.get_logger().info(
                    'link OK (%d messages received)' % self._n_msgs)
            elif self._last_msg_s is None:
                self.get_logger().warning(
                    'link NOT OK: no message has ever arrived on the source topic. '
                    'Check ROS_DOMAIN_ID and which interface the route uses -- an '
                    'empty stream and a domain mismatch are indistinguishable here.')
            else:
                self.get_logger().error(
                    'link LOST: last message %.2f s ago, timeout %.2f s '
                    '(%d received before the gap)'
                    % (age, self._stale_timeout_s, self._n_msgs))
            self._last_ok = ok


def main(args=None):
    # Spin/teardown lives in node_runtime.run(); see that module for why SIGTERM
    # needs handling that SIGINT does not.
    return run(LinkWatchdogNode, args=args)


if __name__ == '__main__':
    sys.exit(main())
