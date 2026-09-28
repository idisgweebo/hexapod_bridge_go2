#!/usr/bin/env python3
"""Re-stamp and re-frame /utlidar/cloud into /go2/lidar/points. Payload untouched.

/utlidar/cloud is already a STANDARD sensor_msgs/PointCloud2 (session 2 census,
inventory-go2_ros2.md line 718), which is the one piece of luck in this design: the
whole LiDAR path needs no unitree_* message types, so no cross-distro type building
is on the critical path. What the message lacks is a usable Header, and that is all
this node supplies.

TWO THINGS ARE WRONG WITH THE INCOMING HEADER
---------------------------------------------
1. The stamp is on the Go2's clock, which is ~1396 s (23 minutes) behind this
   machine. A default 10 s tf2 buffer rejects such a stamp outright -- it does not
   degrade, it drops. So every stamp is corrected by the measured offset.
2. The frame_id is UNKNOWN -- see below. It is set to a configured value.

WHY THIS NODE RUNS ITS OWN CLOCK ESTIMATOR
------------------------------------------
There is a go2_clock_offset_node that publishes an offset, and this node ignores
it. Two reasons:

  a) Correctness. The estimator takes the MINIMUM of (arrival - stamp), which is
     the true clock offset plus the minimum TRANSPORT delay on that topic. An 80-byte
     PoseStamped and a ~36 KB point cloud do not share a transport delay, so the
     pose-derived offset is the wrong constant for stamping clouds.
     Tagged as inference -- the difference is expected, not measured, and measuring
     it is an open item worth taking (it is the gap between two offsets this code
     already computes separately).
  b) Startup. Subscribing to the offset would make this node's output depend on
     another node's convergence, and produce a silent stall if that node is not
     running. Self-contained is easier to reason about at 15 Hz.

Cost: ~6.5 s to converge at the expected input rate instead of ~5.3 s, and no
output at all until then. That is the honest behaviour -- see below.

WHY NO OUTPUT BEFORE CONVERGENCE
--------------------------------
restamp.corrected() raises on a None offset rather than defaulting to zero. A cloud
stamped with an uncorrected Go2 time looks 23 minutes stale to every consumer, and
substituting 0 would hide the bug behind plausible-looking data. So clouds are
DROPPED until the offset converges, and the node says so. A gap in the output is
recoverable; a stream of wrongly-stamped clouds is not.
"""
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from go2_adapter.clock_offset import CONVERGED, FAULTED, ClockOffsetEstimator
from go2_adapter.node_runtime import run
from go2_adapter.qos import GO2_INPUT_QOS, GO2_OUTPUT_QOS
from go2_adapter.restamp import corrected, to_seconds

# The output topic is an inline literal at the create_publisher call, for the reason
# given in clock_offset_node.py. _check_output_topic() then compares the live
# publisher's resolved topic_name against the yaml, so the literal is the single
# source of truth and a mismatched config is a startup refusal.


class LidarAdapterNode(Node):

    def __init__(self, **kwargs):
        super().__init__('go2_lidar_adapter_node', **kwargs)

        self.declare_parameter('input_topic', '/utlidar/cloud')
        self.declare_parameter('output_topic', '/go2/lidar/points')
        self.declare_parameter('frame_id', 'go2_lidar_link')
        self.declare_parameter('max_rate_hz', 5.0)
        self.declare_parameter('window', 100)
        self.declare_parameter('step_threshold_s', 0.5)
        self.declare_parameter('step_consecutive', 10)

        input_topic = self.get_parameter('input_topic').value
        self._frame_id = self.get_parameter('frame_id').value

        max_rate_hz = float(self.get_parameter('max_rate_hz').value)
        if max_rate_hz <= 0.0:
            raise ValueError(
                'max_rate_hz must be > 0; got %r. A zero or negative rate would '
                'publish nothing while looking configured.' % max_rate_hz)
        self._min_period_s = 1.0 / max_rate_hz

        self._est = ClockOffsetEstimator(
            window=int(self.get_parameter('window').value),
            step_threshold=float(self.get_parameter('step_threshold_s').value),
            step_consecutive=int(self.get_parameter('step_consecutive').value),
        )

        self._pub = self.create_publisher(
            PointCloud2, '/go2/lidar/points', GO2_OUTPUT_QOS)
        # After the publisher exists, so the check compares the REAL resolved topic
        # rather than a second copy of the string.
        self._check_output_topic()
        self._sub = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, GO2_INPUT_QOS)

        self._last_pub_s = None
        self._n_in = 0
        self._n_out = 0
        self._n_dropped_throttle = 0
        self._n_dropped_no_offset = 0
        self._seen_frame_id = None
        self._warned_faulted = False
        self._last_report_n_in = 0
        self._last_report_s = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(
            '%s -> %s, frame_id %r, throttled to %.2f Hz'
            % (input_topic, self._pub.topic_name, self._frame_id, max_rate_hz))
        # Counters exist because a silent adapter and a working one look the same from
        # outside, and because /utlidar/cloud's real rate has never been measured --
        # the throttle's drop ratio is the cheapest instrument we have for it.
        self._report_timer = self.create_timer(10.0, self._report)

    def _check_output_topic(self):
        """Refuse to start if the mounted config disagrees with where we actually publish.

        The yaml is a mounted file, editable without rebuilding the image or re-running
        the safety test. If someone points output_topic somewhere else, the honest
        outcome is a refusal to start -- not a node that quietly ignores its own
        configuration, and not a publisher on a topic name nothing has checked.

        Compares the LIVE publisher's resolved topic_name, not a second copy of the
        string, so there is nothing here that can drift out of agreement with the
        literal passed to create_publisher.
        """
        actual = self._pub.topic_name
        configured = self.get_parameter('output_topic').value
        if configured != actual:
            raise ValueError(
                'config declares output_topic=%r but this node publishes to %r. The '
                'topic name is an inline literal in lidar_adapter_node.py on purpose: '
                'the static no-publish test can only verify literals. Change the '
                'literal and the yaml together, or neither.' % (configured, actual))

    def _on_cloud(self, msg):
        self._n_in += 1
        now_s = self.get_clock().now().nanoseconds / 1e9

        self._note_incoming_frame(msg.header.frame_id)

        # Feed the estimator on EVERY message, before the throttle. The throttle
        # controls our output rate; starving the estimator to a fifth of the samples
        # would quintuple convergence time for no benefit, and the minimum-delay
        # estimator specifically wants as many samples as it can get.
        state = self._est.add(
            now_s, to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec))

        if state == FAULTED:
            if not self._warned_faulted:
                self.get_logger().error(
                    'clock offset FAULTED (worst residual %.6f s) -- publishing STOPPED. '
                    'The Go2 most likely rebooted. This is latched: restart the node '
                    'deliberately rather than trusting a silently re-converged offset.'
                    % self._est.worst_residual)
                self._warned_faulted = True
            return

        offset = self._est.offset
        if offset is None:
            self._n_dropped_no_offset += 1
            return
        if state == CONVERGED and self._n_out == 0:
            self.get_logger().info(
                'clock offset CONVERGED at %.6f s after %d clouds; publishing now'
                % (offset, self._n_in))

        # Throttle on OUR publish times, not on message stamps. This is a rate limit
        # on what we emit, and it must hold even if the input arrives in bursts --
        # which the Go2's delivery measurably does on other topics (CLAUDE.md gotcha
        # 14: the IMU's rate is stable while its delivery is bursty).
        if self._last_pub_s is not None and (now_s - self._last_pub_s) < self._min_period_s:
            self._n_dropped_throttle += 1
            return
        self._last_pub_s = now_s

        # Rewrite the header in place and republish. The point payload -- fields,
        # width, height, is_bigendian, point_step, row_step, data -- is passed through
        # completely untouched. Reinterpreting the payload would need the frame and Z
        # convention that item O-1 exists to measure.
        sec, nanosec = corrected(msg.header.stamp.sec, msg.header.stamp.nanosec, offset)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)
        self._n_out += 1

    def _note_incoming_frame(self, frame_id):
        """Log the incoming frame_id once, because nobody has ever measured it.

        The config comments say "unitree types carry no frame_id at all", which is
        true of the unitree_* messages -- and /utlidar/cloud is NOT one of them, it is
        a standard PointCloud2 that may well carry a real frame. No cloud has ever
        been echoed to a file, so this is unmeasured either way.

        Overwriting a populated frame_id would discard information about the robot's
        own frame tree, so it gets logged loudly and exactly once. This costs nothing
        and closes the question on the next live run.
        """
        if self._seen_frame_id is not None:
            return
        self._seen_frame_id = frame_id
        if frame_id:
            self.get_logger().warning(
                'incoming cloud carries frame_id %r, and this node is OVERWRITING it '
                'with %r. That incoming value has never been recorded anywhere -- '
                'write it into the session log and into inventory-go2_ros2.md.'
                % (frame_id, self._frame_id))
        else:
            self.get_logger().info(
                'incoming cloud frame_id is empty, as assumed; setting %r. '
                'Record this too: it was previously unmeasured.' % self._frame_id)

    def _report(self):
        """Periodic counters, and the only rate instrument we have on this topic.

        /utlidar/cloud's delivery rate has never been measured -- the 15.37 Hz figure
        in circulation is LidarState.cloud_frequency, the LiDAR's self-report about
        itself, not an observation of the topic. Counting arrivals here is an
        independent measurement, so it is reported as one.

        The rate is computed over the ACTUAL interval between reports, from counter
        deltas. Dividing a cumulative count by the timer period would silently be
        wrong for the first window and after any scheduling hiccup, and would read as
        a measurement while being an assumption about the timer.
        """
        now_s = self.get_clock().now().nanoseconds / 1e9
        d_in = self._n_in - self._last_report_n_in
        dt = now_s - self._last_report_s
        self._last_report_n_in = self._n_in
        self._last_report_s = now_s

        if self._n_in == 0:
            self.get_logger().warning(
                'no clouds received on the input topic in %.1f s. A healthy endpoint '
                'graph does not imply traffic -- check ROS_DOMAIN_ID and the cable, '
                'not just `topic info`.' % dt)
            return

        # dt is wall time between two timer callbacks, so it is never exactly the
        # period; guard the division rather than assuming it.
        rate = (d_in / dt) if dt > 1e-6 else float('nan')
        self.get_logger().info(
            'in %d (+%d), out %d, dropped %d throttle + %d awaiting-offset; '
            'input %.2f Hz measured over %.1f s'
            % (self._n_in, d_in, self._n_out, self._n_dropped_throttle,
               self._n_dropped_no_offset, rate, dt))


def main(args=None):
    # Spin/teardown lives in node_runtime.run(); see that module for why SIGTERM
    # needs handling that SIGINT does not.
    return run(LidarAdapterNode, args=args)


if __name__ == '__main__':
    sys.exit(main())
