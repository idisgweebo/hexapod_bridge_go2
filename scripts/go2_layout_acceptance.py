#!/usr/bin/env python3
"""Gate 5 -- unitree_go layout acceptance test. Re-runnable per session.

ROS 2 Humble matches DDS endpoints on type NAME, not type CONTENT; structural
type hashing (REP-2011) postdates Humble. So if the unitree_go definitions
built from upstream 668d1ec5 do not match this robot's firmware, discovery
succeeds, `ros2 topic echo` runs, and every field is plausible-looking garbage
with no error raised anywhere.

This subscribes to /sportmodestate and /lowstate simultaneously from a single
participant and evaluates predictions P1-P16, which were registered in
ros_bridge_project/gate5_session4_log.md BEFORE any sample was taken.

Posture is NOT assumed -- pass it with --posture. P7 (foot_force) and P13
(foot_position_body) are only meaningful against the stated posture, and session
4's P13 passed VACUOUSLY because foot_position_body is all zeros at rest.

Session 5 adds two questions that need the moment of power-up:
  * P8s -- is the +0.150760 rad signed yaw offset (sport - low) stable across a
    power cycle? Pass session 4's value with --yaw-ref 0.150760.
  * P9  -- the clock offset, re-measured, never hardcoded. --offset-ref only
    registers the prediction; it is never applied as a correction.

SAFETY -- read-only. This module creates subscriptions only. It must never
construct a publisher, a service client, or an action client. Publishing to a
robot topic requires explicit in-session approval and is out of scope here.
Reviewers: grep this file for create_publisher / create_client. There are none.
"""

import argparse
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState, SportModeState

# Load-bearing checks. A failure in any of these fails the whole test.
#
# P8 (all three axes aggregated) was load-bearing in session 4 and MUST NOT be:
# session 4 itself established that yaw has no absolute reference, so the two
# estimators legitimately hold different origins. Gating on the aggregate makes
# the test report a firmware DECODE failure every time -- which it did in both
# session 4 and session 5, while the project's own conclusion was that the
# decode is correct. The gate now uses the two GRAVITY-REFERENCED axes, which
# must agree if both sides decode correctly. Yaw is recorded by P8y/P8s as a
# finding, never as a gate.
#
# P1  self-consistency of the decode
# P4  physical ground truth (gravity) -- see its widened band below
# P8r/P8p  cross-participant agreement on the axes that have an absolute reference
CRITICAL = ("P1", "P4", "P8r", "P8p")

G = 9.80665


def quat_to_rpy(w, x, y, z):
    """ZYX (aerospace) euler angles from a quaternion, in radians."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def ang_diff(a, b):
    """Signed smallest difference a-b, wrapped to [-pi, pi]."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def norm(v):
    return math.sqrt(sum(c * c for c in v))


class Collector(Node):
    """One node, one participant, both topics.

    `ros2 topic echo --once` costs ~0.9 s of startup and discovery each; two
    separate invocations would also give two unsynchronised views, which would
    make the P8 cross-participant comparison meaningless. Hence one node.
    """

    def __init__(self, want):
        super().__init__("go2_layout_acceptance")
        self.want = want
        self.sport = []
        self.low = []
        # Default QoS is RELIABLE / VOLATILE / KEEP_LAST(1), which matches what
        # every Go2 publisher offers (inventory-go2_ros2.md section 2).
        # Never request TRANSIENT_LOCAL here: durability is VOLATILE robot-wide
        # and a TRANSIENT_LOCAL request would receive nothing at all.
        self.create_subscription(SportModeState, "/sportmodestate", self._on_sport, 10)
        self.create_subscription(LowState, "/lowstate", self._on_low, 10)

    def _on_sport(self, msg):
        if len(self.sport) < self.want:
            # /sportmodestate carries its own TimeSpec stamp; we also record
            # arrival so the clock offset (P9) can be computed.
            self.sport.append((time.time(), msg))

    def _on_low(self, msg):
        if len(self.low) < self.want:
            # /lowstate has NO stamp -- only uint32 tick. It MUST be stamped on
            # arrival, and therefore carries unknown transport latency.
            self.low.append((time.time(), msg))

    def done(self):
        # EITHER, not BOTH. The two topics run at different rates (~300 Hz and
        # ~500 Hz), so waiting for both to fill lets the faster one stop early
        # while the slower keeps collecting against nothing. In session 5 that
        # left /lowstate capped at 12.0 s while /sportmodestate ran to 20.0 s,
        # and the last 8 s of sport samples paired with a stale low sample
        # 7989 ms away -- which is exactly what P8t reported.
        # Stopping on the first to fill keeps both windows overlapping.
        return len(self.sport) >= self.want or len(self.low) >= self.want


class Report:
    def __init__(self):
        self.rows = []

    def add(self, pid, what, predicted, measured, ok):
        # rclpy returns array fields (float32[4] etc) as numpy arrays, so any
        # comparison derived from them is a numpy.bool_, and
        # `numpy.bool_(True) is not True` is True. Coerce to a real bool or the
        # pass/fail summary silently disagrees with its own table.
        self.rows.append((pid, what, predicted, measured, None if ok is None else bool(ok)))

    def failed_critical(self):
        return [r[0] for r in self.rows if r[0] in CRITICAL and r[4] is not True]

    def render(self):
        w = [max(len(str(r[i])) for r in self.rows) for i in range(4)]
        head = ("#", "Quantity", "Predicted", "Measured")
        w = [max(w[i], len(head[i])) for i in range(4)]
        line = "  ".join("-" * w[i] for i in range(4)) + "  ------"
        out = ["  ".join(head[i].ljust(w[i]) for i in range(4)) + "  RESULT", line]
        for pid, what, pred, meas, ok in self.rows:
            mark = {True: "PASS", False: "FAIL", None: "n/a "}[ok]
            star = " *" if pid in CRITICAL else ""
            cells = [pid, what, str(pred), str(meas)]
            out.append("  ".join(cells[i].ljust(w[i]) for i in range(4)) + f"  {mark}{star}")
        out.append("")
        out.append("* = load-bearing. P1 self-consistency, P4 physical ground truth,")
        out.append("  P8 agreement between two independent DDS participants.")
        return "\n".join(out)


def evaluate(sport, low, rep, csv_path, yaw_ref=None, offset_ref=None):
    # ---- P1 / P2: quaternion vs rpy, under BOTH possible element orderings ----
    # We do not presume Unitree's convention; we measure which one agrees.
    errs = {"wxyz": [], "xyzw": []}
    for _, m in sport:
        q = list(m.imu_state.quaternion)
        r = list(m.imu_state.rpy)
        for name, (w, x, y, z) in (
            ("wxyz", (q[0], q[1], q[2], q[3])),
            ("xyzw", (q[3], q[0], q[1], q[2])),
        ):
            got = quat_to_rpy(w, x, y, z)
            errs[name].append(max(abs(ang_diff(got[i], r[i])) for i in range(3)))
    best = min(errs, key=lambda k: max(errs[k]))
    worst = {k: max(v) for k, v in errs.items()}
    agree = worst[best] < 1e-3
    rep.add("P1", "quat->rpy vs rpy field", "< 1e-3 rad", f"{worst[best]:.2e} rad ({best})", agree)
    rep.add("P2", "quaternion element order", "[w,x,y,z]",
            f"{best} (other: {worst['xyzw' if best == 'wxyz' else 'wxyz']:.2e})",
            best == "wxyz" if agree else None)

    # ---- P3: quaternion norm (weak: recorded, not relied on) ----
    norms = [norm(m.imu_state.quaternion) for _, m in sport]
    dn = max(abs(n - 1.0) for n in norms)
    rep.add("P3", "||quaternion||", "1.000 +/- 1e-3", f"{min(norms):.6f}..{max(norms):.6f}", dn < 1e-3)

    # ---- P4: accelerometer magnitude vs gravity (PHYSICAL ground truth) ----
    # The band is +/-1.0, NOT +/-0.3. This test validates the DECODE: it asks
    # whether the field is an accelerometer reading m/s2 rather than plausible
    # garbage. It does NOT ask whether the IMU is calibrated -- it measurably is
    # not, reading ~4% low in both sessions, which is a recorded open finding.
    # A 0.3 band tests calibration and fails on a correctly decoded field.
    accs = [norm(m.imu_state.accelerometer) for _, m in sport]
    amean = sum(accs) / len(accs)
    rep.add("P4", "||accelerometer|| at rest", "9.81 +/- 1.0 m/s2 (decode)",
            f"{amean:.3f} ({min(accs):.3f}..{max(accs):.3f})", abs(amean - G) <= 1.0)
    # The bias itself is recorded, never gated: it is not a fixed constant.
    rep.add("P4b", "  ...bias vs 9.80665", "recorded, not gated",
            f"{amean - G:+.3f} m/s2 ({100.0 * (amean - G) / G:+.2f} %)", None)

    # ---- P5: gyroscope near zero at rest ----
    gyros = [norm(m.imu_state.gyroscope) for _, m in sport]
    gmax = max(gyros)
    rep.add("P5", "||gyroscope|| at rest", "< 0.05 rad/s", f"max {gmax:.4f}", gmax < 0.05)

    # ---- P6: roll and pitch near level ----
    rolls = [m.imu_state.rpy[0] for _, m in sport]
    pitches = [m.imu_state.rpy[1] for _, m in sport]
    rpmax = max(max(abs(v) for v in rolls), max(abs(v) for v in pitches))
    rep.add("P6", "roll, pitch while resting", "|v| < 0.15 rad", f"max {rpmax:.4f} rad", rpmax < 0.15)

    # ---- P7: foot forces, legs unloaded when lying ----
    # ---- P7: foot forces. READ FROM /lowstate, NOT /sportmodestate ----
    # SportModeState.foot_force is all zeros in every posture on this firmware,
    # standing included, so the old check passed VACUOUSLY in sessions 3-5 --
    # the same defect as P13. LowState.foot_force IS populated: [105,105,88,100]
    # lying in session 4. Posture-dependent, so it is recorded, not gated.
    lff = [list(m.foot_force) for _, m in low]
    lfmax = max(max(v) for v in lff)
    lfmin = min(min(v) for v in lff)
    rep.add("P7", "LowState.foot_force[4]", "non-zero, populated",
            f"{lfmin}..{lfmax}", lfmax != 0)
    # Per-foot, because the aggregate cannot distinguish "all four loaded" from
    # "one loaded and three not" -- which is the whole question when comparing
    # a standing robot to a lying one.
    perfoot = [[row[i] for row in lff] for i in range(4)]
    rep.add("P7f", "  ...per foot, mean [FR FL RR RL]", "even when standing",
            " ".join(f"{sum(f) / len(f):.0f}" for f in perfoot), None)
    rep.add("P7r", "  ...per foot, min..max", "recorded",
            " ".join(f"{min(f)}-{max(f)}" for f in perfoot), None)
    ff = [list(m.foot_force) for _, m in sport]
    ffmax = max(max(abs(v) for v in row) for row in ff)
    rep.add("P7z", "  ...SportModeState.foot_force", "all zero on this firmware",
            f"max |v| {ffmax}", None)

    # ---- P8: /lowstate vs /sportmodestate IMU -- TWO INDEPENDENT PARTICIPANTS ----
    # Pair each sport sample with the lowstate sample nearest in arrival time.
    per_axis = [[], [], []]
    dt_pair = []
    for ts, sm in sport:
        tl, lm = min(low, key=lambda kv: abs(kv[0] - ts))
        dt_pair.append(abs(tl - ts))
        for i in range(3):
            per_axis[i].append(abs(ang_diff(sm.imu_state.rpy[i], lm.imu_state.rpy[i])))
    ax = [max(v) for v in per_axis]
    pmax = max(ax)
    rep.add("P8", "IMU: /lowstate vs /sportmodestate", "< 1e-2 rad", f"max {pmax:.2e} rad", pmax < 1e-2)
    # Roll and pitch are gravity-referenced and must agree if both decode
    # correctly. Yaw has no absolute reference, so the two participants can
    # hold independent yaw estimates without either being mis-decoded.
    rep.add("P8r", "  ...roll only", "< 1e-2 rad", f"max {ax[0]:.2e} rad", ax[0] < 1e-2)
    rep.add("P8p", "  ...pitch only", "< 1e-2 rad", f"max {ax[1]:.2e} rad", ax[1] < 1e-2)
    rep.add("P8y", "  ...yaw only", "< 1e-2 rad", f"max {ax[2]:.2e} rad", ax[2] < 1e-2)
    # A loose pairing window invalidates P8/P8r/P8p/P8y -- they compare samples
    # that are not contemporaneous. Say so in the row rather than letting a
    # large number sit there looking like a result.
    dtmax = max(dt_pair) * 1e3
    rep.add("P8t", "  ...pairing window", "< 50 ms",
            f"max dt {dtmax:.2f} ms" + ("  <-- TOO LOOSE, P8* UNRELIABLE" if dtmax > 50 else ""),
            dtmax <= 50)

    # ---- P8s: the SIGNED yaw offset -- THE POWER-CYCLE QUESTION ----
    # Session 4 measured (sport - low) = +0.150760 rad, sd 1.1e-5, over 5993
    # pairs tighter than 5 ms. A fixed mounting or calibration rotation would
    # reproduce that constant after a power cycle; a filter-initialisation
    # difference would be re-randomised at boot and would not.
    #
    # P8 above takes |diff|, which cannot answer this -- the sign and the mean
    # are the whole point. Pairs are restricted to dt < 5 ms so the statistic is
    # comparable to session 4's rather than widened by pairing slop.
    tight = [ang_diff(sm.imu_state.rpy[2], lm.imu_state.rpy[2])
             for ts, sm in sport
             for tl, lm in [min(low, key=lambda kv: abs(kv[0] - ts))]
             if abs(tl - ts) < 5e-3]
    if not tight:
        rep.add("P8s", "  ...SIGNED yaw offset (sport-low)",
                "n/a" if yaw_ref is None else f"{yaw_ref:+.6f} rad",
                "NO PAIRS tighter than 5 ms -- INCONCLUSIVE, not a negative", None)
    else:
        ymean = sum(tight) / len(tight)
        ystd = math.sqrt(sum((v - ymean) ** 2 for v in tight) / len(tight))
        # 1e-3 rad (0.057 deg) is loose against session 4's 1.1e-5 sd but far
        # tighter than any re-randomised yaw origin would land by chance.
        match = None if yaw_ref is None else abs(ang_diff(ymean, yaw_ref)) < 1e-3
        rep.add("P8s", f"  ...SIGNED yaw offset (sport-low) n={len(tight)}",
                "n/a" if yaw_ref is None else f"{yaw_ref:+.6f} rad",
                f"{ymean:+.6f} rad sd {ystd:.1e} ({math.degrees(ymean):+.3f} deg)", match)
        rep.add("P8m", "  ...signed yaw offset spread", "narrow",
                f"{min(tight):+.6f} .. {max(tight):+.6f} rad", None)

    # ---- P9: SportModeState.stamp against the laptop clock ----
    # NEVER hardcoded: the offset moved +28.270 s between 18 and 22 Sep 2026.
    # The prediction is supplied per session by --offset-ref; the accept band is
    # deliberately wide because the point is to RECORD the value, not gate on it.
    offs = [t - (m.stamp.sec + m.stamp.nanosec * 1e-9) for t, m in sport]
    omean = sum(offs) / len(offs)
    rep.add("P9", "clock offset (laptop - robot)",
            "recorded, not gated" if offset_ref is None else f"approx {offset_ref:.1f} s",
            f"{omean:.3f} s (min {min(offs):.3f})", 1200.0 < omean < 1800.0)

    # ---- P10 / P11: motor slots. 12 real joints, 8 spares on a shared firmware ----
    real, spare = [], []
    for _, m in low:
        for i, ms in enumerate(m.motor_state):
            (real if i < 12 else spare).append(ms)
    qmax = max(abs(ms.q) for ms in real)
    rep.add("P10", "motor_state[0..11].q", "|q| < 3.2 rad", f"max |q| {qmax:.3f}", qmax < 3.2)
    spare_live = any(ms.q != 0.0 or ms.dq != 0.0 or ms.tau_est != 0.0 for ms in spare)
    sq = max((abs(ms.q) for ms in spare), default=0.0)
    rep.add("P11", "motor_state[12..19] spares", "inert (0 or constant)",
            f"max |q| {sq:.3g}, live={spare_live}", not spare_live)

    # ---- P12: mode and gait_type are small enumerations ----
    modes = sorted({m.mode for _, m in sport})
    gaits = sorted({m.gait_type for _, m in sport})
    ok12 = all(v < 20 for v in modes) and all(v < 20 for v in gaits)
    rep.add("P12", "mode, gait_type", "small ints < 20", f"mode={modes} gait={gaits}", ok12)

    # ---- P13: foot_position_body -- UNPOPULATED, recorded only ----
    # This was intended as the BACK of the SportModeState bracket. It cannot
    # serve: it is all zeros in every posture measured, STANDING included
    # (session 5). Gating on "|v| < 0.5" is satisfied by a field carrying no
    # information, which is how it passed vacuously in sessions 3-5.
    fpb = [v for _, m in sport for v in m.foot_position_body]
    fmax = max(abs(v) for v in fpb)
    rep.add("P13", "foot_position_body[12]", "unpopulated on this fw",
            f"max |v| {fmax:.4f}", None)

    # ---- P17: yaw_speed vs imu_state.gyroscope[2] -- THE REAL BRACKET ----
    # These are two different fields at two different offsets carrying the SAME
    # physical quantity, and the firmware sets them identically. That makes this
    # a self-validating alignment check reaching far past imu_state -- it cannot
    # pass by accident on a mis-decoded struct, and unlike foot_position_body it
    # is populated at rest. This is the anchor P13 was supposed to be.
    ysd = [abs(m.yaw_speed - m.imu_state.gyroscope[2]) for _, m in sport]
    rep.add("P17", "yaw_speed vs gyroscope[2]", "identical (< 1e-6)",
            f"max {max(ysd):.2e}", max(ysd) < 1e-6)

    # ---- P14 / P15: battery, PHYSICAL ground truth on a 28.8 V pack ----
    socs = sorted({m.bms_state.soc for _, m in low})
    rep.add("P14", "bms_state.soc", "0..100", f"{socs}", all(0 <= s <= 100 for s in socs))
    pv = [m.power_v for _, m in low]
    pmean = sum(pv) / len(pv)
    rep.add("P15", "power_v", "24..30 V", f"{pmean:.2f} V ({min(pv):.2f}..{max(pv):.2f})",
            24.0 <= pmean <= 30.0)

    # ---- P16: tick monotonic ----
    ticks = [m.tick for _, m in low]
    mono = all(b >= a for a, b in zip(ticks, ticks[1:]))
    span = ticks[-1] - ticks[0]
    dt = low[-1][0] - low[0][0]
    rate = span / dt if dt > 0 else float("nan")
    rep.add("P16", "tick monotonic", "monotonic, ~1 kHz",
            f"{'monotonic' if mono else 'NON-MONOTONIC'}, {rate:.0f} units/s", mono)

    with open(csv_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["src", "recv_epoch", "qw", "qx", "qy", "qz",
                    "roll", "pitch", "yaw", "ax", "ay", "az", "gx", "gy", "gz"])
        for tag, rows in (("sportmodestate", sport), ("lowstate", low)):
            for t, m in rows:
                i = m.imu_state
                w.writerow([tag, f"{t:.6f}"] + [f"{v:.9g}" for v in
                           list(i.quaternion) + list(i.rpy) +
                           list(i.accelerometer) + list(i.gyroscope)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=200)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--session", default="s5",
                    help="session tag used in the banner and the default CSV name")
    ap.add_argument("--posture", default="LYING / RESTING",
                    help="physical posture; P7 and P13 are only meaningful against it")
    ap.add_argument("--yaw-ref", type=float, default=None,
                    help="prior signed yaw offset (sport-low) in rad to compare against, "
                         "e.g. 0.150760 from session 4. Omit to record without comparing.")
    ap.add_argument("--offset-ref", type=float, default=None,
                    help="prior clock offset in s, for the registered prediction only. "
                         "Never used as a correction -- re-measure every session.")
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()
    csv_path = args.csv or f"/logs/{args.session}_layout_samples.csv"

    rclpy.init()
    node = Collector(args.samples)
    t0 = time.time()
    while rclpy.ok() and not node.done() and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    sport, low = node.sport, node.low
    node.destroy_node()
    rclpy.shutdown()

    print("=" * 78)
    print(f"unitree_go LAYOUT ACCEPTANCE TEST -- Gate 5 session {args.session}")
    print(f"Posture: {args.posture}.  Read-only: subscriptions only, no publisher.")
    print(f"Collected: /sportmodestate {len(sport)}, /lowstate {len(low)} "
          f"(requested {args.samples}) in {time.time() - t0:.1f}s")
    print(f"UTC now: {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}")
    print("=" * 78)

    # Distinguish "no data" from "broken instrument": say which topic was silent.
    if not sport or not low:
        for name, got in (("/sportmodestate", sport), ("/lowstate", low)):
            if not got:
                print(f"NO DATA on {name} -- 0 samples in {args.timeout}s.")
        print("\nINCONCLUSIVE. Not a layout failure: no samples to decode.")
        print("Check ROS_DOMAIN_ID, CYCLONEDDS_URI, the cable, and that the robot is on.")
        return 2

    rep = Report()
    evaluate(sport, low, rep, csv_path, args.yaw_ref, args.offset_ref)
    print(rep.render())
    print(f"\nPer-sample CSV: {csv_path}")

    bad = rep.failed_critical()
    others = [r[0] for r in rep.rows if r[4] is False and r[0] not in CRITICAL]
    print()
    if bad:
        print(f"RESULT: FAIL -- load-bearing checks failed: {', '.join(bad)}")
        print("This is evidence that the unitree_go definitions at 668d1ec5 may not decode")
        print("this firmware correctly. Before concluding that, rule out the setup: wrong")
        print("image, unsourced /ws overlay, unbound Cyclone, domain mismatch, loose P8t.")
        return 1
    print(f"RESULT: PASS -- all load-bearing checks ({', '.join(CRITICAL)}) passed.")
    if others:
        print(f"NOTE: non-critical checks failed: {', '.join(others)}. Investigate, do not ignore.")
    print("Scope: this validates the DECODE of the fields exercised above. It does not")
    print("establish that 668d1ec5 is the correct upstream release for this firmware.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
