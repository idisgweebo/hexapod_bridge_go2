#!/usr/bin/env python3
"""Go2 dance capture -- READ-ONLY: subscribes, never publishes. Not even a publisher object exists.

WHY THIS EXISTS
---------------
friday_dance_demo_plan.md (26 Oct 2026), step 1: Doug triggers the dance from the R3-1
handset with the Go2 on the cable; the laptop only listens. It answers PDN-0..PDN-6:
  PDN-0  does a handset dance appear on the wire as /api/sport/request api_id 1022/1023?
  PDN-1  which SportModeState.error_code does the dance run under?
  PDN-2  peak |v| / |yaw_speed| -- would go2_cmd_bridge's 0.3 m/s / 1.0 rad/s guards trip?
  PDN-3  net displacement and peak excursion (floor space for the demo)
  PDN-4  duration, and the state it ends in
  PDN-5  rear-hip temperature rise
  PDN-6  the /api/sport/response code, if PDN-0 holds
⚠ Subscribing to /api/sport/request is a READ. This script creates no publisher.

WHAT IT RECORDS (each to its own file, prefix --out)
----------------------------------------------------
  <out>_state.csv    SportModeState at ~50 Hz (every DECIMATE-th message) + newest LowState fields
  <out>_events.csv   EVERY /api/sport/request, /api/sport/response and /wirelesscontroller message
Requests and responses are also printed the instant they arrive.

  go2_dance_capture.py --out /logs/dance1            record until Ctrl+C (or --duration)
  go2_dance_capture.py --replay /logs/dance1         per-state-segment summary, NO ROS needed
  go2_dance_capture.py --selftest                    pure-logic checks, NO ROS needed
"""
import argparse, csv, json, math, os, sys, time

RR_HIP, RL_HIP = 6, 9
DECIMATE = 6          # /sportmodestate is ~300 Hz -> ~50 Hz rows
STALE_S = 1.0
GUARD_V, GUARD_W = 0.3, 1.0      # go2_cmd_bridge hard caps (bridge_core.py)
STATE_FIELDS = ["t_wall", "n_rx", "error_code", "mode", "gait_type", "progress", "body_height",
                "px", "py", "vx", "vy", "yaw_speed", "rr_hip_c", "rl_hip_c", "max_other_c",
                "motors_mode1", "low_age_s"]
EVENT_FIELDS = ["t_wall", "topic", "api_id", "name", "code", "parameter", "keys", "lx", "ly", "rx", "ry"]
POSTURE = {1001: "lying", 1002: "stand-lock", 100: "balance-stand", 2006: "standing-down"}
# unitree_ros2 668d1ec5 ros2_sport_client.h -- SOURCE CODE, not measured on this robot.
SPORT_API = {1001: "DAMP", 1002: "BALANCESTAND", 1003: "STOPMOVE", 1004: "STANDUP", 1005: "STANDDOWN",
             1006: "RECOVERYSTAND", 1007: "EULER", 1008: "MOVE", 1009: "SIT", 1010: "RISESIT",
             1015: "SPEEDLEVEL", 1016: "HELLO", 1017: "STRETCH", 1020: "CONTENT", 1022: "DANCE1",
             1023: "DANCE2", 1027: "SWITCHJOYSTICK", 1028: "POSE", 1029: "SCRAPE", 1030: "FRONTFLIP",
             1031: "FRONTJUMP", 1032: "FRONTPOUNCE", 1036: "HEART", 1061: "STATICWALK", 1062: "TROTRUN",
             1063: "ECONOMICGAIT", 2041: "LEFTFLIP", 2043: "BACKFLIP", 2044: "HANDSTAND", 2045: "FREEWALK",
             2046: "FREEBOUND", 2047: "FREEJUMP", 2048: "FREEAVOID", 2049: "CLASSICWALK",
             2050: "WALKUPRIGHT", 2051: "CROSSSTEP", 2054: "AUTORECOVERY_SET", 2055: "AUTORECOVERY_GET",
             2058: "SWITCHAVOIDMODE"}


def segments(rows):
    """Contiguous runs of the same (error_code, mode). Returns [((code, mode), [rows])]."""
    out = []
    for r in rows:
        k = (r["error_code"], r["mode"])
        if out and out[-1][0] == k:
            out[-1][1].append(r)
        else:
            out.append((k, [r]))
    return out


def seg_summary(key, rs):
    x0, y0 = rs[0]["px"], rs[0]["py"]
    exc = max(math.hypot(r["px"] - x0, r["py"] - y0) for r in rs)
    return {"code": key[0], "mode": key[1], "posture": POSTURE.get(key[0], "code %d" % key[0]),
            "t0": rs[0]["t_wall"], "dur_s": rs[-1]["t_wall"] - rs[0]["t_wall"], "n": len(rs),
            "v_peak": max(math.hypot(r["vx"], r["vy"]) for r in rs),
            "w_peak": max(abs(r["yaw_speed"]) for r in rs),
            "net_m": math.hypot(rs[-1]["px"] - x0, rs[-1]["py"] - y0), "exc_m": exc,
            "h_min": min(r["body_height"] for r in rs), "h_max": max(r["body_height"] for r in rs),
            "gaits": sorted({r["gait_type"] for r in rs}),
            "prog_max": max(r["progress"] for r in rs),
            "rr0": rs[0]["rr_hip_c"], "rr1": rs[-1]["rr_hip_c"],
            "rl0": rs[0]["rl_hip_c"], "rl1": rs[-1]["rl_hip_c"],
            "mode1_min": min(r["motors_mode1"] for r in rs)}


def report(rows, events, out=sys.stdout):
    p = lambda s="": print(s, file=out)
    p("EVENTS (requests/responses all; handset only when keys change)")
    last_keys = None
    for e in events:
        if e["topic"] == "/wirelesscontroller":
            if e["keys"] == last_keys:
                continue
            last_keys = e["keys"]
            p("  %s handset keys=%d" % (_ts(e["t_wall"]), e["keys"]))
        else:
            p("  %s %-22s api_id=%s %s code=%s param=%s" % (_ts(e["t_wall"]), e["topic"], e["api_id"],
              e["name"], e["code"], (e["parameter"] or "")[:60]))
    n_req = sum(e["topic"] == "/api/sport/request" for e in events)
    p("  -> %d /api/sport/request message(s) seen (PDN-0)" % n_req)
    p()
    p("SEGMENTS by (error_code, mode)  -- position is the robot's odom frame")
    p("%8s %6s %4s %14s %6s %6s %6s %6s %6s %5s %5s %9s %9s %s" % (
        "start", "code", "mode", "posture", "dur s", "v pk", "w pk", "net m", "exc m", "h min", "h max",
        "RR C", "RL C", "flags"))
    for key, rs in segments(rows):
        s = seg_summary(key, rs)
        flags = []
        if s["v_peak"] > GUARD_V: flags.append("v>%.1f" % GUARD_V)
        if s["w_peak"] > GUARD_W: flags.append("w>%.1f" % GUARD_W)
        if s["mode1_min"] < 12: flags.append("MOTOR mode!=1")
        p("%8s %6d %4d %14s %6.1f %6.2f %6.2f %6.2f %6.2f %5.3f %5.3f %4.0f->%-4.0f %4.0f->%-4.0f %s" % (
            _ts(s["t0"]), s["code"], s["mode"], s["posture"], s["dur_s"], s["v_peak"], s["w_peak"],
            s["net_m"], s["exc_m"], s["h_min"], s["h_max"], s["rr0"], s["rr1"], s["rl0"], s["rl1"],
            " ".join(flags)))
    if rows:
        x0, y0 = rows[0]["px"], rows[0]["py"]
        p()
        p("WHOLE CAPTURE: %.1f s, peak excursion from first sample %.2f m, net %.2f m, "
          "rear hips RR %.0f->%.0f RL %.0f->%.0f C" % (
              rows[-1]["t_wall"] - rows[0]["t_wall"],
              max(math.hypot(r["px"] - x0, r["py"] - y0) for r in rows),
              math.hypot(rows[-1]["px"] - x0, rows[-1]["py"] - y0),
              rows[0]["rr_hip_c"], rows[-1]["rr_hip_c"], rows[0]["rl_hip_c"], rows[-1]["rl_hip_c"]))


def _ts(t):
    return time.strftime("%H:%M:%S", time.gmtime(t)) + ("%.1f" % (t % 1))[1:]


def load(prefix):
    num = lambda v: float(v) if v not in ("", None) else None
    rows = []
    with open(prefix + "_state.csv") as f:
        for r in csv.DictReader(f):
            d = {k: float(v) for k, v in r.items()}
            for k in ("n_rx", "error_code", "mode", "gait_type", "motors_mode1"):
                d[k] = int(d[k])
            rows.append(d)
    events = []
    with open(prefix + "_events.csv") as f:
        for r in csv.DictReader(f):
            r["t_wall"] = float(r["t_wall"])
            r["keys"] = int(r["keys"]) if r["keys"] else None
            events.append(r)
    return rows, events


def run_selftest():
    fails = []
    def chk(name, got, want):
        ok = (abs(got - want) < 1e-6) if isinstance(want, float) else got == want
        print("%s %s: got %r want %r" % ("ok  " if ok else "FAIL", name, got, want))
        if not ok: fails.append(name)
    def row(t, code, mode=1, px=0.0, py=0.0, vx=0.0, w=0.0, rr=30.0, m1=12):
        return dict(t_wall=t, n_rx=0, error_code=code, mode=mode, gait_type=0, progress=0.0,
                    body_height=0.32, px=px, py=py, vx=vx, vy=0.0, yaw_speed=w, rr_hip_c=rr,
                    rl_hip_c=rr, max_other_c=30.0, motors_mode1=m1, low_age_s=0.0)
    rows = [row(0, 100), row(1, 100), row(2, 999, 7, 0.3, 0.4, 0.5, 1.5, 32),
            row(3, 999, 7, 0.0, 0.1, 0.1, -2.0, 33), row(4, 100, 1, 0.0, 0.1)]
    segs = segments(rows)
    chk("three segments (code/mode change splits)", len(segs), 3)
    s = seg_summary(*segs[1])
    chk("dance seg v_peak", s["v_peak"], 0.5)
    chk("dance seg w_peak is |min|", s["w_peak"], 2.0)
    chk("dance seg excursion from its own start", s["exc_m"], math.hypot(0.3, 0.3))
    chk("dance seg duration", s["dur_s"], 1.0)
    chk("rear hip start->end", (s["rr0"], s["rr1"]), (32.0, 33.0))
    import io
    buf = io.StringIO()
    ev = [dict(t_wall=2.0, topic="/api/sport/request", api_id=1022, name="DANCE1", code="", parameter="",
               keys=None, lx="", ly="", rx="", ry=""),
          dict(t_wall=1.9, topic="/wirelesscontroller", api_id="", name="", code="", parameter="",
               keys=0, lx="", ly="", rx="", ry=""),
          dict(t_wall=1.95, topic="/wirelesscontroller", api_id="", name="", code="", parameter="",
               keys=0, lx="", ly="", rx="", ry="")]
    report(rows, ev, out=buf)
    txt = buf.getvalue()
    chk("report flags guard trips", "v>0.3 w>1.0" in txt, True)
    chk("report counts requests", "1 /api/sport/request" in txt, True)
    chk("repeated handset keys collapsed", txt.count("handset keys=0"), 1)
    m1 = [row(0, 100, m1=11)]
    buf = io.StringIO(); report(m1, [], out=buf)
    chk("motor mode flag", "MOTOR mode!=1" in buf.getvalue(), True)
    chk("api name map", SPORT_API[1023], "DANCE2")
    print("SELFTEST %s (%d failed)" % ("PASS" if not fails else "FAIL", len(fails)))
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--replay", metavar="PREFIX")
    ap.add_argument("--out", metavar="PREFIX")
    ap.add_argument("--duration", type=float, default=1800.0)
    ap.add_argument("--interval", type=float, default=1.0, help="seconds between printed lines")
    args = ap.parse_args()
    if args.selftest:
        return run_selftest()
    if args.replay:
        report(*load(args.replay))
        return 0
    if not args.out:
        ap.error("--out is required to record")
    for suf in ("_state.csv", "_events.csv"):
        if os.path.exists(args.out + suf):
            print("REFUSING: %s exists -- never overwrite a capture." % (args.out + suf), file=sys.stderr)
            return 2
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from unitree_go.msg import LowState, SportModeState, WirelessController
        from unitree_api.msg import Request, Response
    except Exception as e:
        print("ROS not available: %s" % e, file=sys.stderr)
        return 2

    fs = open(args.out + "_state.csv", "w", newline="")
    fe = open(args.out + "_events.csv", "w", newline="")
    ws = csv.DictWriter(fs, fieldnames=STATE_FIELDS); ws.writeheader()
    we = csv.DictWriter(fe, fieldnames=EVENT_FIELDS); we.writeheader()

    class Rec(Node):
        def __init__(self):
            super().__init__("go2_dance_capture")
            q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)
            self.n = 0; self.low = None; self.t_low = None; self.t_sport = None; self.last = None
            self.n_req = 0; self.n_hs = 0
            self.create_subscription(SportModeState, "/sportmodestate", self.sport_cb, q)
            self.create_subscription(LowState, "/lowstate", self.low_cb, q)
            self.create_subscription(WirelessController, "/wirelesscontroller", self.hs_cb, q)
            self.create_subscription(Request, "/api/sport/request", self.req_cb, q)
            self.create_subscription(Response, "/api/sport/response", self.resp_cb, q)

        def low_cb(self, m):
            self.t_low = time.time()
            temps = [m.motor_state[i].temperature for i in range(12)]
            self.low = dict(rr_hip_c=float(temps[RR_HIP]), rl_hip_c=float(temps[RL_HIP]),
                            max_other_c=float(max(t for i, t in enumerate(temps) if i not in (RR_HIP, RL_HIP))),
                            motors_mode1=sum(1 for i in range(12) if m.motor_state[i].mode == 1))

        def sport_cb(self, m):
            self.n += 1; self.t_sport = now = time.time()
            if self.n % DECIMATE or self.low is None:
                return
            r = dict(t_wall=now, n_rx=self.n, error_code=int(m.error_code), mode=int(m.mode),
                     gait_type=int(m.gait_type), progress=float(m.progress), body_height=float(m.body_height),
                     px=float(m.position[0]), py=float(m.position[1]), vx=float(m.velocity[0]),
                     vy=float(m.velocity[1]), yaw_speed=float(m.yaw_speed), low_age_s=now - self.t_low,
                     **self.low)
            ws.writerow(r); self.last = r

        def hs_cb(self, m):
            self.n_hs += 1
            we.writerow(dict(t_wall=time.time(), topic="/wirelesscontroller", keys=int(m.keys),
                             lx=round(m.lx, 3), ly=round(m.ly, 3), rx=round(m.rx, 3), ry=round(m.ry, 3)))

        def req_cb(self, m):
            self.n_req += 1
            a = int(m.header.identity.api_id); t = time.time()
            we.writerow(dict(t_wall=t, topic="/api/sport/request", api_id=a, name=SPORT_API.get(a, "?"),
                             parameter=m.parameter))
            print(">>> %s REQUEST  api_id=%d %s param=%r" % (_ts(t), a, SPORT_API.get(a, "?"), m.parameter[:80]))
            fe.flush(); sys.stdout.flush()

        def resp_cb(self, m):
            a = int(m.header.identity.api_id); t = time.time()
            we.writerow(dict(t_wall=t, topic="/api/sport/response", api_id=a, name=SPORT_API.get(a, "?"),
                             code=int(m.header.status.code), parameter=m.data[:200]))
            print("<<< %s RESPONSE api_id=%d %s code=%d" % (_ts(t), a, SPORT_API.get(a, "?"),
                                                        m.header.status.code))
            fe.flush(); sys.stdout.flush()

    rclpy.init()
    node = Rec()
    t0 = time.time(); last_print = 0.0
    print("DANCE CAPTURE -- read-only, no publishers. -> %s_{state,events}.csv" % args.out)
    print("%10s %14s %4s %5s %6s %6s %6s %5s %5s %3s %5s %5s" % (
        "UTC", "posture", "mode", "gait", "h m", "v", "w", "x", "y", "m1", "RR C", "RL C"))
    try:
        while rclpy.ok() and time.time() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now - last_print < args.interval:
                continue
            last_print = now
            if node.t_sport is None or now - node.t_sport > STALE_S or node.last is None \
                    or now - node.t_low > STALE_S:
                print("%10s STALE -- /sportmodestate or /lowstate silent. Values UNKNOWN." % _ts(now))
                sys.stdout.flush(); continue
            r = node.last
            print("%10s %14s %4d %5d %6.3f %6.2f %6.2f %5.2f %5.2f %3d %5.0f %5.0f" % (
                _ts(now), POSTURE.get(r["error_code"], "code %d" % r["error_code"]), r["mode"],
                r["gait_type"], r["body_height"], math.hypot(r["vx"], r["vy"]), r["yaw_speed"],
                r["px"], r["py"], r["motors_mode1"], r["rr_hip_c"], r["rl_hip_c"]))
            sys.stdout.flush(); fs.flush(); fe.flush()
    except KeyboardInterrupt:
        pass
    fs.close(); fe.close()
    print("-" * 78)
    print("requests seen: %d, handset messages: %d" % (node.n_req, node.n_hs))
    report(*load(args.out))
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
