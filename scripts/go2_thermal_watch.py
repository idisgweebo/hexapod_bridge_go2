#!/usr/bin/env python3
"""Gate 5 -- LIVE rear-hip thermal watchdog. READ-ONLY: subscribes, never publishes.

WHY THIS EXISTS
---------------
Session 7 stood the Go2 ~18 minutes during a software patch. RR_hip reached 65 C,
RL_hip 69-70 C, all torque dropped, and the robot thermally shut down. The warning
was in session 6's data (29 -> 47/49 C in ten minutes standing) AND in session 7's
own baseline capture (39.6 / 40.2 C) -- unread, both times.

gate5_session7_plan.md section 4b then made a live watch MANDATORY with a ~50 C
abort ceiling. But go2_unload_probe.py only writes temperature to its CSV; it never
prints it. The rule had no instrument. This is the instrument.

WHAT IT WATCHES
---------------
Session 6 measured the rear hips carrying ~3.7x the front hips (7.7-8.0 vs
2.1-2.2 N.m standing), and they were the two joints that got 19 C hotter while the
other ten stayed ~30 C. So RR_hip (index 6) and RL_hip (index 9) are the thermal
limit of this machine. All twelve are tracked; those two set the verdict.

SCOPE -- what this does NOT do
------------------------------
It does not stop the robot. It cannot: we never publish. It tells a human to sit
the robot with the handset. The abort is Doug's hands, every time.
"""
import argparse, sys, time

RR_HIP, RL_HIP = 6, 9
JOINT = ["FR_hip","FR_thigh","FR_calf","FL_hip","FL_thigh","FL_calf",
         "RR_hip","RR_thigh","RR_calf","RL_hip","RL_thigh","RL_calf"]

CEILING   = 50.0   # section 4b abort ceiling, degrees C
WARN      = 45.0   # start calling out loudly
NOTICE    = 40.0   # session 7's unread baseline was 39.6/40.2 -- notice it this time
SHUTDOWN  = 65.0   # what RR_hip actually reached when the robot collapsed


def classify(peak, ceiling=CEILING, warn=WARN, notice=NOTICE):
    """Return (tag, should_abort). Pure -- offline-testable without ROS."""
    if peak >= ceiling:
        return "ABORT", True
    if peak >= warn:
        return "WARN", False
    if peak >= notice:
        return "NOTICE", False
    return "ok", False


MIN_SPAN_S = 90.0  # see below -- 1 C quantisation makes short spans meaningless


def slope_per_min(hist, window_s=120.0, min_span_s=MIN_SPAN_S):
    """Degrees C per minute over the trailing window. None if too little data.
    hist is a list of (t, temp). Pure -- offline-testable.

    WHY min_span_s IS LARGE
    -----------------------
    LowState.motor_state[].temperature is an int8: the field is quantised to
    1 C. A single one-count tick across a 6 s span reads as +10 C/min, and the
    first live run of this script duly printed "+10.00 C/min, eta 1.6 min"
    while the robot lay still and barely warming. The rate is not a measurement
    until the span is long enough that one count of quantisation is small
    against the real change. At the ~2 C/min the rear hips actually climb while
    standing, 90 s spans a useful 3 C; one count is then a third of the signal
    rather than all of it.

    Reporting nothing is correct here. A number that is wrong in the alarming
    direction trains the operator to ignore the instrument."""
    if len(hist) < 2:
        return None
    t_end, v_end = hist[-1]
    cut = t_end - window_s
    older = [(t, v) for t, v in hist if t <= cut]
    t0, v0 = older[-1] if older else hist[0]
    dt = t_end - t0
    if dt < min_span_s:
        return None
    return (v_end - v0) * 60.0 / dt


def eta_to_ceiling(peak, rate, ceiling=CEILING):
    """Minutes until the ceiling at the current rate. None if not climbing. Pure."""
    if rate is None or rate <= 0.05:
        return None
    if peak >= ceiling:
        return 0.0
    return (ceiling - peak) / rate


def run_offline_selftest():
    ok = True
    def chk(name, got, want):
        nonlocal ok
        if got != want:
            ok = False
            print(f"  FAIL {name}: got {got!r}, want {want!r}")
        else:
            print(f"  pass {name}")

    print("offline self-test (no ROS, no robot):")
    chk("ambient 26 C is ok",        classify(26.0), ("ok", False))
    chk("39.9 below notice",         classify(39.9), ("ok", False))
    chk("s7 baseline 40.2 notices",  classify(40.2), ("NOTICE", False))
    chk("s6 ten-minute 49 warns",    classify(49.0), ("WARN", False))
    chk("ceiling 50.0 aborts",       classify(50.0), ("ABORT", True))
    chk("s7 collapse 65 aborts",     classify(65.0), ("ABORT", True))

    # a climb of 2 C/min, the session-6 rear-hip rate, sampled over 3 min
    hist = [(float(i), 26.0 + 2.0 * i / 60.0) for i in range(0, 181)]
    r = slope_per_min(hist)
    chk("slope recovers 2 C/min", r is not None and abs(r - 2.0) < 0.1, True)
    # THE DEFECT THE FIRST LIVE RUN EXPOSED: int8 temperature is quantised to
    # 1 C, so one count over a few seconds looked like +10 C/min and an eta of
    # 1.6 min while the robot lay still. Short spans must report nothing.
    quantised = [(0.0, 33.0), (3.0, 33.0), (6.0, 34.0)]
    chk("one count over 6 s reports NO rate (was +10 C/min)",
        slope_per_min(quantised), None)
    chk("...and therefore no eta", eta_to_ceiling(34.0, slope_per_min(quantised)), None)
    chk("60 s span still too short at 1 C resolution",
        slope_per_min([(0.0, 33.0), (60.0, 34.0)]), None)
    chk("92 s span is long enough to report",
        slope_per_min([(0.0, 33.0), (92.0, 34.0)]) is not None, True)
    chk("eta from 26 C at 2 C/min ~12 min",
        abs(eta_to_ceiling(26.0, 2.0) - 12.0) < 0.01, True)
    chk("flat gives no eta", eta_to_ceiling(30.0, 0.0), None)
    chk("cooling gives no eta", eta_to_ceiling(30.0, -1.5), None)
    chk("too little data -> no slope", slope_per_min([(0.0, 30.0)]), None)
    chk("short span -> no slope",
        slope_per_min([(0.0, 30.0), (2.0, 31.0)]), None)
    # a watchdog that never fires is the failure mode that matters
    chk("already at ceiling -> eta 0", eta_to_ceiling(50.0, 3.0), 0.0)
    print("SELFTEST:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true",
                    help="run pure-logic tests with no ROS and exit")
    ap.add_argument("--interval", type=float, default=2.0,
                    help="seconds between printed lines (default 2)")
    ap.add_argument("--duration", type=float, default=1800.0,
                    help="seconds to watch before exiting (default 1800)")
    ap.add_argument("--ceiling", type=float, default=CEILING)
    ap.add_argument("--label", default="", help="tag printed on every line")
    args = ap.parse_args()

    if args.selftest:
        return run_offline_selftest()

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from unitree_go.msg import LowState
    except Exception as e:
        print(f"ROS not available: {e}", file=sys.stderr)
        print("For logic checks with no robot, use --selftest.", file=sys.stderr)
        return 2

    class Watch(Node):
        def __init__(self):
            super().__init__("go2_thermal_watch")
            q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, depth=5)
            self.latest = None
            self.n = 0
            self.create_subscription(LowState, "/lowstate", self.cb, q)
        def cb(self, m):
            self.latest = [m.motor_state[i].temperature for i in range(12)]
            self.n += 1

    rclpy.init()
    node = Watch()
    t0 = time.time()
    hist_rr, hist_rl = [], []
    last_print = 0.0
    aborted = False
    tag = f"[{args.label}] " if args.label else ""

    print("=" * 78)
    print(f"{tag}REAR-HIP THERMAL WATCH -- read-only, ceiling {args.ceiling:.0f} C")
    print("Session 7 reference: RR_hip 65 C / RL_hip 69 C at shutdown.")
    print("This watchdog CANNOT stop the robot. On ABORT, sit it with the handset.")
    print("=" * 78)
    print(f"{'elapsed':>8s} {'RR_hip':>7s} {'RL_hip':>7s} {'hottest other':>16s} "
          f"{'C/min':>7s} {'eta->50C':>9s}  state")

    try:
        while rclpy.ok() and time.time() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.time() - t0
            if node.latest is None or now - last_print < args.interval:
                continue
            last_print = now
            temps = node.latest
            rr, rl = temps[RR_HIP], temps[RL_HIP]
            hist_rr.append((now, rr)); hist_rl.append((now, rl))
            others = [(v, JOINT[i]) for i, v in enumerate(temps)
                      if i not in (RR_HIP, RL_HIP)]
            ov, oname = max(others)
            peak = max(rr, rl)
            state, do_abort = classify(peak, args.ceiling, WARN, NOTICE)
            rate = max((x for x in (slope_per_min(hist_rr), slope_per_min(hist_rl))
                        if x is not None), default=None)
            eta = eta_to_ceiling(peak, rate, args.ceiling)
            rate_s = f"{rate:+.2f}" if rate is not None else "   --"
            eta_s = f"{eta:.1f} min" if eta is not None else "     --"
            print(f"{now:8.1f} {rr:7.1f} {rl:7.1f} {ov:9.1f} {oname:>6s} "
                  f"{rate_s:>7s} {eta_s:>9s}  {state}")
            if state == "WARN":
                print(f"         ^^ WARN: {peak:.1f} C. Finish the step and SIT THE ROBOT.")
            if do_abort and not aborted:
                aborted = True
                print("!" * 78)
                print(f"!! ABORT -- rear hip {peak:.1f} C >= ceiling {args.ceiling:.0f} C")
                print("!! SIT THE ROBOT NOW with the handset. Do not finish the capture.")
                print(f"!! Cooldown tau ~45-50 min. Do not stand it again until both")
                print(f"!! rear hips read under 35 C.")
                print("!" * 78)
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass

    print("-" * 78)
    if hist_rr:
        print(f"{tag}peak RR_hip {max(v for _, v in hist_rr):.1f} C, "
              f"peak RL_hip {max(v for _, v in hist_rl):.1f} C, "
              f"{node.n} LowState samples over {time.time()-t0:.0f}s")
        if aborted:
            print("ENDED WITH AN ABORT RAISED.")
    else:
        print("NO SAMPLES. Not a thermal finding -- check cable, domain, robot power.")
    node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
