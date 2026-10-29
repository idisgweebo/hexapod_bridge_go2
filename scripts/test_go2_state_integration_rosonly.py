#!/usr/bin/env python3
"""END-TO-END test of phase 1's Go2 side over real DDS. Needs ROS 2. Does NOT need the robot.

    fake Go2 (/sportmodestate, /lowstate, /utlidar/robot_odom, /utlidar/imu)
        |                                  \\
    go2_cmd_bridge (DRY, input_type twist_stamped)  -- /go2/cmd_bridge/status -->  go2_state_node
        ^                                                                            |
    test's TwistStamped source (/go2/cmd_vel_stamped)          sink <-- /go2/odom, /go2/imu, /go2/robot_status

The offline suites prove each core's logic and the logic-level chain. They cannot prove
that the NODES wire it up: message fields read from the right places, the JSON status
crossing DDS, the stamp surviving rclpy, the restamp sign, DiagnosticStatus.level's type,
the throttle against a real executor. That is this file.

The bridge is DRY (armed:=false), so its robot publisher is never created. The fake body
walks on a schedule of its own -- this test is about state derivation, not physics.

The fake Go2's clock is AHEAD by 0.8403 s: v1.1.15 measured -0.840 / -0.426 s (20 Oct), and
a non-round value catches a truncation that a round one would hide.

Plan: phase1_go2_feedback_plan.md stage 1 (work item 5). RUN IT BEFORE THE ROBOT IS POWERED.
"""
import json
import math
import os
import sys
import tempfile
import threading
import time

from test_cmd_bridge_integration_rosonly import LOOPBACK_XML
_XML = os.path.join(tempfile.mkdtemp(prefix='go2_state_it_'), 'cyclonedds_loopback.xml')
with open(_XML, 'w') as _f:
    _f.write(LOOPBACK_XML)
os.environ['CYCLONEDDS_URI'] = 'file://' + _XML
os.environ.pop('ROS_LOCALHOST_ONLY', None)

import rclpy                                                      # noqa: E402
from rclpy.node import Node                                       # noqa: E402
from rclpy.parameter import Parameter                             # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray                   # noqa: E402
from geometry_msgs.msg import TwistStamped                        # noqa: E402
from nav_msgs.msg import Odometry                                 # noqa: E402
from sensor_msgs.msg import Imu                                   # noqa: E402
from std_msgs.msg import String                                   # noqa: E402
from unitree_go.msg import LowState, SportModeState               # noqa: E402

from go2_adapter.go2_state_node import Go2StateNode               # noqa: E402
from go2_adapter.qos import GO2_INPUT_QOS                         # noqa: E402
from go2_cmd_bridge.bridge_node import BridgeNode                 # noqa: E402

GO2_AHEAD_S = 0.8403
STAMP_TOL = 0.05
SCHEMA_KEYS = ["schema_version", "cmd_last_stamp", "cmd_age_s", "bridge_state", "bridge_reason", "bridge_input",
               "bridge_armed", "motion", "speed_mps", "yaw_rate_rps", "sport_state_code", "fault", "warn",
               "rr_hip_c", "rl_hip_c", "go2_clock_offset_s", "clock_state", "odom_covariance_known"]

fails = []
checks = 0


def check(label, ok, detail=''):
    global checks
    checks += 1
    print('  %s %s%s' % ('pass' if ok else 'FAIL', label, (' -- ' + str(detail)[:300]) if detail else ''))
    if not ok:
        fails.append(label)


class FakeGo2(Node):

    def __init__(self):
        super().__init__('fake_go2_state_it')
        self.sport_pub = self.create_publisher(SportModeState, '/sportmodestate', GO2_INPUT_QOS)
        self.low_pub = self.create_publisher(LowState, '/lowstate', GO2_INPUT_QOS)
        self.odom_pub = self.create_publisher(Odometry, '/utlidar/robot_odom', GO2_INPUT_QOS)
        self.imu_pub = self.create_publisher(Imu, '/utlidar/imu', GO2_INPUT_QOS)
        self.v = 0.0
        self.x = 0.0
        self.alive = True
        self.k = 0
        self.last = time.time()

    def stamp(self, msg):
        t = time.time() + GO2_AHEAD_S
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9)) % 1000000000

    def tick(self):
        """Called at 250 Hz: imu every tick, odom 3 of 5 (150 Hz), sport/low 1 of 5 (50 Hz)."""
        now = time.time()
        self.x += self.v * (now - self.last)
        self.last = now
        if not self.alive:
            return
        self.k += 1
        i = Imu()
        self.stamp(i)
        i.header.frame_id = 'utlidar_imu'
        self.imu_pub.publish(i)
        if self.k % 5 in (0, 2, 4):
            o = Odometry()
            self.stamp(o)
            o.header.frame_id, o.child_frame_id = 'odom', 'base_link'      # MEASURED (s2, v1.1.11)
            o.pose.pose.position.x = self.x
            o.twist.twist.linear.x = self.v
            self.odom_pub.publish(o)
        if self.k % 5 == 0:
            s = SportModeState()
            s.error_code = 100
            s.position = [float(self.x), 0.0, 0.0]
            s.velocity = [float(self.v), 0.0, 0.0]
            s.yaw_speed = 0.0
            self.sport_pub.publish(s)
            low = LowState()
            for j in range(12):
                low.motor_state[j].mode = 1
                low.motor_state[j].temperature = 35
            self.low_pub.publish(low)


class Sink(Node):

    def __init__(self):
        super().__init__('sink_state_it')
        self.odom, self.imu, self.status, self.bridge, self.requests = [], [], [], [], []
        now = lambda: time.time()                                          # noqa: E731
        self.create_subscription(Odometry, '/go2/odom', lambda m: self.odom.append((now(), m)), 10)
        self.create_subscription(Imu, '/go2/imu', lambda m: self.imu.append((now(), m)), 10)
        self.create_subscription(DiagnosticArray, '/go2/robot_status', lambda m: self.status.append((now(), m)), 10)
        self.create_subscription(String, '/go2/cmd_bridge/status',
                                 lambda m: self.bridge.append((now(), json.loads(m.data))), 10)
        self.create_subscription(String, '/go2/cmd_bridge/requests',
                                 lambda m: self.requests.append((now(), json.loads(m.data))), 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/go2/cmd_vel_stamped', 10)

    def send(self, vx, stamp):
        m = TwistStamped()
        m.header.stamp.sec, m.header.stamp.nanosec = stamp
        m.twist.linear.x = vx
        self.cmd_pub.publish(m)

    def values(self, i=-1):
        st = self.status[i][1].status[0]
        return dict((kv.key, kv.value) for kv in st.values)


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec / 1e9


def main():
    print('CYCLONEDDS_URI loopback-only: %s' % os.environ['CYCLONEDDS_URI'])
    # Global overrides reach the bridge (its constructor takes no kwargs). Dry, stamped input.
    rclpy.init(args=['--ros-args', '-p', 'armed:=false', '-p', 'input_type:=twist_stamped'])
    state = Go2StateNode(parameter_overrides=[Parameter('clock_window', Parameter.Type.INTEGER, 50)])
    bridge = BridgeNode()
    fake, sink = FakeGo2(), Sink()
    exe = rclpy.executors.SingleThreadedExecutor()
    for n in (state, bridge, fake, sink):
        exe.add_node(n)

    stop = threading.Event()

    def pump():
        while not stop.is_set():
            fake.tick()
            time.sleep(0.004)

    th = threading.Thread(target=pump, daemon=True)
    th.start()

    def spin_for(secs, every=None):
        end = time.time() + secs
        while time.time() < end:
            exe.spin_once(timeout_sec=0.01)
            if every:
                every()

    # ---- warm-up: wait for the clock to converge and the bridge to be IDLE --------------
    deadline = time.time() + 15.0
    ready = False
    while time.time() < deadline and not ready:
        exe.spin_once(timeout_sec=0.02)
        if sink.status:
            v = sink.values()
            ready = v.get('clock_state') == 'CONVERGED' and v.get('bridge_state') == 'IDLE' and v.get('motion') == 'STOPPED'
    check('warm-up: clock CONVERGED, bridge IDLE, motion STOPPED within 15 s', ready,
          sink.values() if sink.status else 'no status at all')

    # ---- phase A: rest, measure rates ------------------------------------------------
    t_a = time.time()
    n0 = (len(sink.odom), len(sink.imu), len(sink.status))
    spin_for(4.0)
    dt = time.time() - t_a
    r_odom, r_imu, r_st = [(b - a) / dt for a, b in zip(n0, (len(sink.odom), len(sink.imu), len(sink.status)))]
    print('A  rest: rates odom %.1f imu %.1f status %.1f Hz' % (r_odom, r_imu, r_st))
    check('A: /go2/odom at 20 +/- 2 Hz (from ~150 in)', 18 <= r_odom <= 22, r_odom)
    check('A: /go2/imu at 50 +/- 5 Hz (from ~250 in)', 45 <= r_imu <= 55, r_imu)
    check('A: /go2/robot_status at 5 +/- 0.5 Hz', 4.5 <= r_st <= 5.5, r_st)
    arr_t, o = sink.odom[-1]
    check('A: odom frames prefixed go2/ (D6)', (o.header.frame_id, o.child_frame_id) == ('go2/odom', 'go2/base_link'),
          (o.header.frame_id, o.child_frame_id))
    check('A: odom restamped onto local time (|stamp - arrival| < 50 ms)', abs(stamp_s(o.header) - arr_t) < STAMP_TOL,
          stamp_s(o.header) - arr_t)
    check('A: ...with the RIGHT SIGN (a flip would be ~%+.2f s)' % (2 * GO2_AHEAD_S),
          abs(stamp_s(o.header) - arr_t) < GO2_AHEAD_S / 2, stamp_s(o.header) - arr_t)
    check('A: odom covariance passes through as zeros (D8)',
          all(c == 0.0 for c in o.pose.covariance) and all(c == 0.0 for c in o.twist.covariance))
    arr_t, i = sink.imu[-1]
    check('A: imu frame go2/utlidar_imu', i.header.frame_id == 'go2/utlidar_imu', i.header.frame_id)
    check('A: imu restamped', abs(stamp_s(i.header) - arr_t) < STAMP_TOL, stamp_s(i.header) - arr_t)
    st = sink.status[-1][1]
    check('A: status: one DiagnosticStatus named go2 with the hardware_id',
          len(st.status) == 1 and st.status[0].name == 'go2' and st.status[0].hardware_id == 'B42N6000PBCD7AG9')
    check('A: status keys are schema v1, in order', [kv.key for kv in st.status[0].values] == SCHEMA_KEYS,
          [kv.key for kv in st.status[0].values])
    lvl = st.status[0].level
    lvl = lvl[0] if isinstance(lvl, (bytes, bytearray)) else lvl
    v = sink.values()
    check('A: healthy rest: level OK, fault none, warn none', lvl == 0 and v['fault'] == 'none' and v['warn'] == 'none',
          (lvl, v['fault'], v['warn']))
    check('A: bridge_input twist_stamped, bridge_armed false', (v['bridge_input'], v['bridge_armed']) == ('twist_stamped', 'false'),
          (v['bridge_input'], v['bridge_armed']))
    check('A: status header stamped with local time', abs(stamp_s(st.header) - sink.status[-1][0]) < STAMP_TOL)
    check('A: go2_clock_offset_s recovers -%.4f s (+/- 20 ms)' % GO2_AHEAD_S,
          abs(float(v['go2_clock_offset_s']) + GO2_AHEAD_S) < 0.02, v['go2_clock_offset_s'])

    # ---- phase B: stamped commands while the body walks --------------------------------
    print('B  stamped commands, body walking')
    base = int(time.time()) - 400000000            # a hexapod clock ~13 years behind: still fine, D3 needs only order
    sent = []
    fake.v = 0.3
    t_b = time.time()
    k = 0
    while time.time() - t_b < 2.5:
        # Normalised: r1 used k * 1e8 + 7 for k up to 24, i.e. nanosec > 1e9, and the
        # bridge accepted it. The bridge now refuses that; see test_cmd_bridge_offline.py.
        stamp = divmod(base * 1000000000 + k * 100000000 + 7, 1000000000)
        sink.send(0.3, stamp)
        sent.append('%d.%09d' % stamp)
        k += 1
        spin_for(0.1)
    spin_for(0.8)
    v = sink.values()
    moves = [r for t, r in sink.requests if t >= t_b and r['name'] == 'MOVE']
    check('B: the dry bridge produced Moves from TwistStamped (routed dry)', len(moves) >= 20 and
          all(r['routed'] == 'dry' for r in moves), len(moves))
    echoes = set(sink.values(i).get('cmd_last_stamp') for i in range(len(sink.status)) if sink.status[i][0] >= t_b)
    check('B: /go2/robot_status echoed our stamps (rclpy + JSON + DDS round trip)', len(echoes & set(sent)) >= 3,
          sorted(echoes)[:5])
    check('B: ...and nothing it echoed was a stamp we did not send', echoes - {''} <= set(sent), echoes - set(sent))
    check('B: the final echo is our LAST stamp', v['cmd_last_stamp'] == sent[-1], (v['cmd_last_stamp'], sent[-1]))
    walking = [sink.values(i)['motion'] for i in range(len(sink.status)) if sink.status[i][0] >= t_b + 1.0]
    check('B: motion MOVING while the body walks', walking.count('MOVING') >= len(walking) - 1, walking)

    # ---- phase C: replays ------------------------------------------------------------------
    print('C  replayed stamps')
    for _ in range(5):
        sink.send(0.3, (base, 7))          # older than every stamp above: a replay
        spin_for(0.1)
    spin_for(0.8)
    counts = sink.bridge[-1][1]['counts']
    check('C: the bridge refused all 5 replays (dropped_nonmonotonic)', counts['dropped_nonmonotonic'] == 5, counts)
    check('C: the echo did not move backwards', sink.values()['cmd_last_stamp'] == sent[-1])

    # ---- phase D: the body stops -------------------------------------------------------
    print('D  body stops')
    fake.v = 0.0
    spin_for(1.5)
    check('D: motion STOPPED', sink.values()['motion'] == 'STOPPED', sink.values()['motion'])

    # ---- phase E: the fake goes silent -------------------------------------------------
    print('E  fake Go2 silent')
    fake.alive = False
    t_e = time.time()
    spin_for(2.0)
    after = [(t - t_e, sink.values(i)) for i, (t, _) in enumerate(sink.status) if t >= t_e]
    first_unknown = next((dt for dt, vv in after if vv['motion'] == 'UNKNOWN'), None)
    check('E: motion UNKNOWN within 0.5 + 0.2 + 0.1 s', first_unknown is not None and first_unknown <= 0.8, first_unknown)
    check('⛔ E: never STOPPED once UNKNOWN', not any(vv['motion'] == 'STOPPED' for dt, vv in after
                                                    if first_unknown is not None and dt > first_unknown))
    lvl = sink.status[-1][1].status[0].level
    lvl = lvl[0] if isinstance(lvl, (bytes, bytearray)) else lvl
    check('E: level STALE, fault names sport_stale and low_stale',
          lvl == 3 and 'sport_stale' in sink.values()['fault'] and 'low_stale' in sink.values()['fault'],
          (lvl, sink.values()['fault']))
    n_odom = len(sink.odom)
    spin_for(0.5)
    check('E: /go2/odom stops when its input stops (nothing re-published stale)', len(sink.odom) == n_odom)

    stop.set()
    th.join(timeout=1.0)
    bridge.closing = True
    for n in (state, bridge, fake, sink):
        n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    print()
    if fails:
        print('FAILURES (%d of %d checks):' % (len(fails), checks))
        for f in fails:
            print('  -', f)
        return 1
    print('ALL %d GO2 STATE INTEGRATION CHECKS PASS' % checks)
    return 0


if __name__ == '__main__':
    sys.exit(main())
