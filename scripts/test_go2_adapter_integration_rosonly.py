#!/usr/bin/env python3
"""END-TO-END test of go2_adapter. Needs ROS 2. Does NOT need the robot.

The offline suite proves the arithmetic in clock_offset.py and restamp.py. It cannot
prove that the NODES wire that arithmetic up correctly -- that the estimator is fed
the right two times in the right order, that the sign of the correction is right, or
that the header actually gets rewritten. A sign error here would pass every offline
test and produce clouds stamped 46 minutes in the WRONG direction.

So this test plays the robot. It publishes synthetic /utlidar/robot_pose and
/utlidar/cloud with stamps deliberately set ~1396 s in the past -- the offset
measured on 28 Sep 2026 -- and asserts on what comes out of /go2/.

RUN IT BEFORE THE ROBOT IS POWERED. It costs nothing and needs no cable.

WHY THE INJECTED OFFSET IS NOT A ROUND NUMBER
---------------------------------------------
1396.079 s is the real measured value (logs/go2/s8_clock_offset_A.txt). A round
number like 1000.0 could be produced by a plausible bug -- a truncation, an integer
division -- that a real-looking value would expose. The tolerance is tight for the
same reason: loose enough for scheduling jitter, tight enough that a wrong constant
cannot hide in it.
"""
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64, String

from go2_adapter.clock_offset_node import ClockOffsetNode
from go2_adapter.lidar_adapter_node import LidarAdapterNode
from go2_adapter.link_watchdog_node import LinkWatchdogNode
from go2_adapter.qos import GO2_INPUT_QOS

TRUE_OFFSET = 1396.079      # measured, 28 Sep 2026
WINDOW = 20                 # smaller than production's 100 to keep the test quick
OFFSET_TOL = 0.05           # generous vs. the offset's magnitude, tight vs. any bug
STAMP_TOL = 0.05

fails = []
checks = 0


def check(label, ok, detail=''):
    global checks
    checks += 1
    print('  %s %s%s' % ('pass' if ok else 'FAIL', label,
                         (' -- ' + detail) if detail else ''))
    if not ok:
        fails.append(label + ((' -- ' + detail) if detail else ''))


class FakeRobot(Node):
    """Publishes what the Go2 publishes, with the Go2's clock error baked in."""

    def __init__(self):
        super().__init__('fake_go2')
        self.pose_pub = self.create_publisher(
            PoseStamped, '/utlidar/robot_pose', GO2_INPUT_QOS)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/utlidar/cloud', GO2_INPUT_QOS)

    def _go2_stamp(self):
        """A stamp as the Go2 would produce it: local time minus the true offset."""
        t = self.get_clock().now().nanoseconds / 1e9 - TRUE_OFFSET
        sec = int(t)
        return sec, int(round((t - sec) * 1e9))

    def tick(self):
        sec, nanosec = self._go2_stamp()

        p = PoseStamped()
        p.header.stamp.sec, p.header.stamp.nanosec = sec, nanosec
        p.header.frame_id = 'odom'          # MEASURED: robot_pose really says odom
        self.pose_pub.publish(p)

        c = PointCloud2()
        c.header.stamp.sec, c.header.stamp.nanosec = sec, nanosec
        c.header.frame_id = ''              # UNMEASURED in reality; empty is the assumption
        c.height, c.width = 1, 1
        c.point_step, c.row_step = 4, 4
        c.data = [0, 0, 0, 0]
        c.is_dense = True
        self.cloud_pub.publish(c)


class Sink(Node):
    """Collects everything the adapter emits under /go2/."""

    def __init__(self):
        super().__init__('sink')
        self.offsets = []
        self.states = []
        self.clouds = []
        self.link_ok = []
        self.create_subscription(Float64, '/go2/clock_offset',
                                 lambda m: self.offsets.append(m.data), GO2_INPUT_QOS)
        self.create_subscription(String, '/go2/clock_offset_state',
                                 lambda m: self.states.append(m.data), GO2_INPUT_QOS)
        self.create_subscription(Bool, '/go2/link_ok',
                                 lambda m: self.link_ok.append(m.data), GO2_INPUT_QOS)
        self.create_subscription(
            PointCloud2, '/go2/lidar/points',
            lambda m: self.clouds.append(
                (m.header.stamp.sec + m.header.stamp.nanosec / 1e9,
                 m.header.frame_id,
                 self.get_clock().now().nanoseconds / 1e9)),
            GO2_INPUT_QOS)


def main():
    rclpy.init()

    # Supplied at construction, not set afterwards: both nodes build their estimator
    # in __init__, so a parameter set later would be read too late and the test would
    # quietly run against the production window instead of this one.
    overrides = [Parameter('window', Parameter.Type.INTEGER, WINDOW)]

    clock_node = ClockOffsetNode(parameter_overrides=overrides)
    lidar_node = LidarAdapterNode(parameter_overrides=overrides)
    watchdog = LinkWatchdogNode()
    robot = FakeRobot()
    sink = Sink()

    exe = rclpy.executors.SingleThreadedExecutor()
    for n in (clock_node, lidar_node, watchdog, robot, sink):
        exe.add_node(n)

    stop = threading.Event()

    def pump():
        # 200 Hz: enough samples to converge a 20-deep window quickly. Faster than the
        # real robot, which is fine -- the estimator is sample-counted, not time-based.
        while not stop.is_set():
            robot.tick()
            time.sleep(0.005)

    t = threading.Thread(target=pump, daemon=True)
    t.start()

    deadline = time.time() + 12.0
    while time.time() < deadline and len(sink.clouds) < 10:
        exe.spin_once(timeout_sec=0.05)
    stop.set()
    t.join(timeout=1.0)
    for _ in range(20):
        exe.spin_once(timeout_sec=0.02)

    print('1. the clock offset is recovered from the wire')
    check('offset was published at all', len(sink.offsets) > 0,
          '%d samples' % len(sink.offsets))
    if sink.offsets:
        got = sink.offsets[-1]
        check('offset matches the injected %.3f s' % TRUE_OFFSET,
              abs(got - TRUE_OFFSET) < OFFSET_TOL,
              'got %.6f, err %.6f s' % (got, got - TRUE_OFFSET))
        # A CONVERGED estimator is frozen. If the published value drifts, freezing is
        # broken -- which is the whole reason the estimator exists.
        check('offset is frozen, not drifting',
              max(sink.offsets) - min(sink.offsets) == 0.0,
              'spread %.9f s' % (max(sink.offsets) - min(sink.offsets)))
    check('state reached CONVERGED', 'CONVERGED' in sink.states)
    check('state never reached FAULTED on a steady clock',
          'FAULTED' not in sink.states)

    print('2. clouds come out re-stamped into LOCAL time')
    check('clouds were republished', len(sink.clouds) >= 10,
          '%d clouds' % len(sink.clouds))
    if sink.clouds:
        stamp, frame, arrival = sink.clouds[-1]
        check('output stamp is local-now, not Go2 time',
              abs(stamp - arrival) < STAMP_TOL,
              'stamp - arrival = %+.6f s' % (stamp - arrival))
        # The sign check is the point of this whole file: a flipped sign is ~2792 s out
        # and every offline test would still pass.
        check('correction has the RIGHT SIGN',
              abs(stamp - arrival) < TRUE_OFFSET / 2,
              'off by %+.3f s; a sign flip would be about %+.0f s'
              % (stamp - arrival, -2 * TRUE_OFFSET))
        check('frame_id was set', frame == 'go2_lidar_link', 'got %r' % frame)

    print('3. the throttle holds')
    if len(sink.clouds) >= 3:
        stamps = [c[2] for c in sink.clouds]
        gaps = [b - a for a, b in zip(stamps, stamps[1:])]
        # Input is 200 Hz, configured output is 5 Hz -> min gap 0.2 s. Allow a little
        # slack for executor scheduling, but not a factor of two.
        check('output gaps respect the 5 Hz limit', min(gaps) > 0.15,
              'min gap %.4f s over %d intervals' % (min(gaps), len(gaps)))

    print('4. the watchdog sees a live link')
    check('link_ok was published', len(sink.link_ok) > 0,
          '%d samples' % len(sink.link_ok))
    check('link_ok went True while data flowed', True in sink.link_ok)

    for n in (clock_node, lidar_node, watchdog, robot, sink):
        n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    print()
    if fails:
        print('FAILURES (%d of %d checks):' % (len(fails), checks))
        for f in fails:
            print('  -', f)
        return 1
    print('ALL %d INTEGRATION CHECKS PASS' % checks)
    return 0


if __name__ == '__main__':
    sys.exit(main())
