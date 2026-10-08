#!/usr/bin/env python3
"""Gate 7 -- scripted Twist source + recorder for the go2_cmd_bridge robot stages.

Plan: gate7_go2_cmdvel_bridge_plan.md (decision D2: a script, not teleop, for the
measured stages). Publishes ONLY /go2/cmd_vel -- never a robot topic. Whether that
moves the robot is decided by the bridge (armed or dry), which this script checks
before it streams anything.

    profile        what it streams                                    stage
    listen         nothing; records 10 s                              0, 1
    walk_03        0.3 m/s x 2.5 s, then STOPS PUBLISHING (watchdog)  1a (dry), 2, 5 (L2+A ~1 s in)
    walk_03_kill   0.3 m/s; SIGKILL the bridge 1.5 s in; record 5 s   3  (PV4)
    turn_10        +1.0 rad/s x 2 s, pause, -1.0 rad/s x 2 s          4

PREFLIGHT (nothing is published until all hold):
  * ROS_DOMAIN_ID exported as 0, Cyclone RMW
  * /go2/cmd_bridge/status received, its `armed` == --armed/--dry as given here,
    its state == --require-state (IDLE by default; WAITING for stage 0, robot lying)
  * /sportmodestate and /lowstate arriving

WHILE STREAMING: if the bridge reports ABORTED or YIELDED the stream stops at once
-- it would be ignored anyway, and a source that keeps talking to a latched bridge
makes the log lie about what the operator intended.
ON Ctrl+C or any error: three zero Twists (the bridge turns a zero Twist into one
StopMove, D3), then exit.

RECORDS run_dir/{events,sport,low,handset}.csv in go2_sport_probe's format, so its
analyse() applies unchanged; this file adds the bridge-specific report (watchdog
timing, travel after the last Move -- PV3 / PV4). --replay re-analyses with no ROS.
"""
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time

import go2_sport_probe as probe          # pure at import: analyse(), JOINT, helpers

CMD_TOPIC = "/go2/cmd_vel"
RATE_HZ = 10.0
VX_CAP, VYAW_CAP = 0.3, 1.0              # = the bridge's hard caps; a profile above them is a code error
BRIDGE_PROCESS = "go2_cmd_bridge_node"   # pkill -f pattern for walk_03_kill
STATUS_TIMEOUT_S = 5.0
SETTLE_MIN_S = 2.0
BRIDGE_CMD_TIMEOUT_S = 0.4               # the bridge's watchdog: Moves continue this long after the last Twist                       # every profile ends recording >= this after its last stream

# Steps: ("wait", s) | ("stream", vx, vyaw, s) | ("kill_bridge",)
PROFILES = {
    "listen": [("wait", 10.0)],
    "walk_03": [("wait", 1.0), ("stream", 0.3, 0.0, 2.5), ("wait", 3.0)],
    "walk_03_kill": [("wait", 1.0), ("stream", 0.3, 0.0, 1.5), ("kill_bridge",), ("wait", 5.0)],
    "turn_10": [("wait", 1.0), ("stream", 0.0, 1.0, 2.0), ("wait", 2.5),
                ("stream", 0.0, -1.0, 2.0), ("wait", 3.0)],
}


class ProfileError(ValueError):
    pass


def validate_profile(name):
    """Raise ProfileError for anything a profile must not contain. Pure."""
    if name not in PROFILES:
        raise ProfileError("unknown profile %r" % name)
    steps = PROFILES[name]
    for st in steps:
        if st[0] == "stream":
            _, vx, vyaw, secs = st
            for label, v, cap in (("vx", vx, VX_CAP), ("vyaw", vyaw, VYAW_CAP)):
                if not math.isfinite(v) or abs(v) > cap + 1e-9:
                    raise ProfileError("%s: %s=%r outside +/-%s" % (name, label, v, cap))
            if not (0 < secs <= 6.0):
                raise ProfileError("%s: stream of %r s (0 < s <= 6; the geofence is 1 m)" % (name, secs))
        elif st[0] == "wait":
            if not (0 < st[1] <= 30):
                raise ProfileError("%s: wait %r" % (name, st[1]))
        elif st[0] == "kill_bridge":
            if not name.endswith("_kill"):
                raise ProfileError("%s: kill_bridge only in a *_kill profile" % name)
        else:
            raise ProfileError("%s: unknown step %r" % (name, st))
    if steps[-1][0] != "wait" or steps[-1][1] < SETTLE_MIN_S:
        raise ProfileError("%s must end with a wait >= %.1f s to record the settle" % (name, SETTLE_MIN_S))
    # worst-case straight-line travel if the robot tracked the command fully, INCLUDING
    # the bridge's watchdog tail: it keeps sending Move for cmd_timeout after the last
    # Twist (measured: 4 Moves, integration test D).
    travel = sum(abs(s[1]) * (s[3] + BRIDGE_CMD_TIMEOUT_S) for s in steps if s[0] == "stream")
    if travel > 0.95:
        raise ProfileError("%s: up to %.2f m commanded -- inside the 1.0 m geofence with margin, please" % (name, travel))
    return steps


def describe(name):
    lines = ["profile %r" % name]
    t = 0.0
    for st in validate_profile(name):
        if st[0] == "wait":
            lines.append("  t~%5.1f  wait %.1f s" % (t, st[1])); t += st[1]
        elif st[0] == "stream":
            lines.append("  t~%5.1f  STREAM Twist vx=%.2f wz=%.2f at %.0f Hz for %.1f s (%d msgs)"
                         % (t, st[1], st[2], RATE_HZ, st[3], int(round(st[3] * RATE_HZ)))); t += st[3]
        else:
            lines.append("  t~%5.1f  ⛔ SIGKILL %s (no StopMove can be sent)" % (t, BRIDGE_PROCESS))
    lines.append("  ~%.1f s in total; publishes only %s" % (t, CMD_TOPIC))
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Analysis -- pure. probe.analyse() plus what is specific to the bridge.
# ---------------------------------------------------------------------------
def _rows(run_dir, name):
    p = os.path.join(run_dir, name)
    if not os.path.exists(p):
        return []
    with open(p, newline="") as fh:
        return list(csv.DictReader(fh))


def analyse_bridge(run_dir):
    out = [probe.analyse(run_dir), "", "--- bridge-specific (gate 7) ---"]
    ev = _rows(run_dir, "events.csv")
    sp = _rows(run_dir, "sport.csv")
    sent = [e for e in ev if e["kind"] == "sent"]
    dry = [e for e in ev if e["kind"].startswith("dry")]
    twists = [float(e["t"]) for e in ev if e["kind"] == "twist"]
    states = [(float(e["t"]), e["detail"]) for e in ev if e["kind"] == "bridge_state"]
    kills = [float(e["t"]) for e in ev if e["kind"] == "kill_bridge"]
    out.append("Twists published: %d   robot requests: %d   dry/dropped requests: %d"
               % (len(twists), len(sent), len(dry)))
    for t, d in states:
        out.append("  bridge state at +%.2f s: %s" % (t - (twists[0] if twists else t), d))
    moves = [float(e["t"]) for e in sent if probe.API_NAME.get(int(e["api_id"])) == "MOVE"]
    stops = [float(e["t"]) for e in sent if probe.API_NAME.get(int(e["api_id"])) == "STOPMOVE"]
    if len(moves) > 1:
        span = moves[-1] - moves[0]
        out.append("Move rate: %.2f Hz over %.2f s (%d Moves)" % ((len(moves) - 1) / span, span, len(moves)))
    if twists and moves:
        tail = [m for m in moves if m > twists[-1] + 0.02]
        out.append("watchdog tail: %d Move(s) after the last Twist" % len(tail))
    if twists and stops and not kills:
        first = [s for s in stops if s > twists[-1]]
        if first:
            out.append("PV3 -- watchdog StopMove %.3f s after the last Twist" % (first[0] - twists[-1]))
            out.append("      " + _travel_after(sp, first[0], "the StopMove", limit_cm=2.0))
    if kills:
        out.append("⛔ bridge SIGKILLed at t=%.3f" % kills[0])
        after = [m for m in moves if m > kills[0] + 0.02]
        out.append("  requests after the kill: %d (expect 0)" % len(after))
        if moves:
            out.append("PV4 -- " + _travel_after(sp, max(m for m in moves if m <= kills[0] + 0.02),
                                                 "the last Move", limit_cm=25.0, stop_s=1.5))
    return "\n".join(out)


def _travel_after(sp, t0, label, limit_cm, stop_s=None):
    """Body-frame travel from t0 to the end of the record, and when it stopped.
    'Stopped' = the last time position was > 1 cm from its final value -- position,
    not the velocity estimate (session 12)."""
    rows = [r for r in sp if float(r["t"]) >= t0]
    if len(rows) < 2:
        return "NO CONCLUSION: no /sportmodestate after %s" % label
    fwd, lat = probe._body_disp(rows[0], rows[-1])
    end = (float(rows[-1]["px"]), float(rows[-1]["py"]))
    moving = [float(r["t"]) for r in rows
              if math.hypot(float(r["px"]) - end[0], float(r["py"]) - end[1]) > 0.01]
    t_stop = (moving[-1] - t0) if moving else 0.0
    d = math.hypot(fwd, lat) * 100
    verdict = "PASS" if d <= limit_cm and (stop_s is None or t_stop <= stop_s) else "FAIL"
    lim = "<= %.0f cm" % limit_cm + ("" if stop_s is None else ", stopped within %.1f s" % stop_s)
    return ("travel after %s: %.1f cm (fwd %+.1f, left %+.1f), settled within 1 cm by +%.2f s "
            "-> %s (%s)" % (label, d, fwd * 100, lat * 100, t_stop, verdict, lim))


# ---------------------------------------------------------------------------
# Live
# ---------------------------------------------------------------------------
def run_live(args):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from unitree_go.msg import SportModeState, LowState, WirelessController
    from unitree_api.msg import Response

    steps = validate_profile(args.profile)
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    run_dir = os.path.join(args.out, "bridge_%s_%s" % (args.profile, stamp))
    os.makedirs(run_dir, exist_ok=True)
    print("run dir: " + run_dir)
    print(describe(args.profile))

    files = {}

    def writer(name, head):
        fh = open(os.path.join(run_dir, name), "w", newline="")
        w = csv.writer(fh)
        w.writerow(head)
        files[name] = fh
        return w

    w_ev = writer("events.csv", ["t", "utc", "kind", "api_id", "code", "detail"])
    w_sp = writer("sport.csv", ["t", "mode", "gait_type", "error_code", "body_height",
                                "px", "py", "pz", "vx", "vy", "vz", "yaw_speed", "yaw"])
    w_lo = writer("low.csv", ["t"] + ["q_" + n for n in probe.JOINT] + ["temp_" + n for n in probe.JOINT])
    w_hs = writer("handset.csv", ["t", "lx", "ly", "rx", "ry", "keys"])

    def event(kind, api_id="", code="", detail=""):
        t = time.time()
        w_ev.writerow(["%.6f" % t, time.strftime("%H:%M:%S", time.gmtime(t)), kind, api_id, code, detail])
        files["events.csv"].flush()
        if kind not in ("twist", "sent", "dry_dry"):
            print("[%s] %-12s %s %s %s" % (time.strftime("%H:%M:%SZ", time.gmtime(t)), kind, api_id, code, detail))

    st = {"status": None, "n_sport": 0, "n_low": 0, "state": None}

    class Profile(Node):
        def __init__(self):
            super().__init__("go2_cmd_vel_profile")
            # The only publisher in this file -- a /go2/ topic, never a robot topic.
            self.pub = self.create_publisher(Twist, "/go2/cmd_vel", 10)
            self.create_subscription(String, "/go2/cmd_bridge/status", self.on_status, 10)
            self.create_subscription(String, "/go2/cmd_bridge/requests", self.on_breq, 10)
            self.create_subscription(SportModeState, "/sportmodestate", self.on_sport, 10)
            self.create_subscription(LowState, "/lowstate", self.on_low, 10)
            self.create_subscription(WirelessController, "/wirelesscontroller", self.on_hs, 10)
            self.create_subscription(Response, "/api/sport/response", self.on_resp, 10)

        def on_status(self, m):
            s = json.loads(m.data)
            st["status"] = s
            if s.get("state") != st["state"]:
                st["state"] = s.get("state")
                event("bridge_state", detail="%s (%s)" % (s.get("state"), s.get("reason")))

        def on_breq(self, m):
            r = json.loads(m.data)
            kind = "sent" if r["routed"] == "robot" else "dry_" + r["routed"]
            event(kind, r["api_id"], "", r["parameter"] if r["parameter"] else r["name"])

        def on_sport(self, m):
            st["n_sport"] += 1
            if st["n_sport"] % 6 == 0:
                w_sp.writerow(["%.6f" % time.time(), m.mode, m.gait_type, m.error_code,
                               "%.5f" % m.body_height] + ["%.5f" % x for x in m.position]
                              + ["%.5f" % x for x in m.velocity] + ["%.5f" % m.yaw_speed,
                                                                     "%.6f" % m.imu_state.rpy[2]])

        def on_low(self, m):
            st["n_low"] += 1
            if st["n_low"] % 10 == 0:
                w_lo.writerow(["%.6f" % time.time()] + ["%.6f" % m.motor_state[i].q for i in range(12)]
                              + [m.motor_state[i].temperature for i in range(12)])

        def on_hs(self, m):
            w_hs.writerow(["%.6f" % time.time(), "%.3f" % m.lx, "%.3f" % m.ly, "%.3f" % m.rx,
                           "%.3f" % m.ry, m.keys])

        def on_resp(self, m):
            event("response", m.header.identity.api_id, m.header.status.code, m.data.replace("\n", " "))

    # rclpy's own SIGINT handler shuts the context down at once, so the zero Twists in
    # the except-branch below could never be published (integration P6, 8 Oct: the
    # abort was "RuntimeError: Unable to convert call argument" from spin_once). Same
    # fix as go2_cmd_bridge's main(): disable it (Humble) / override it (Foxy,
    # untested), and turn the signal into a KeyboardInterrupt raised in OUR loop,
    # with the context still alive.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    sig = {"n": None}

    def on_signal(signum, _frame):
        sig["n"] = signum

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    node = Profile()

    def spin_for(secs):
        t_end = time.time() + secs
        while time.time() < t_end:
            if sig["n"] is not None:
                raise KeyboardInterrupt("signal %d" % sig["n"])
            rclpy.spin_once(node, timeout_sec=0.01)

    def twist(vx, vyaw):
        t = Twist()
        t.linear.x, t.angular.z = float(vx), float(vyaw)
        node.pub.publish(t)
        event("twist", "", "", "%.3f %.3f" % (vx, vyaw))

    rc = 0
    streamed = False
    try:
        probs = probe.check_env(os.environ)
        t_end = time.time() + STATUS_TIMEOUT_S
        while time.time() < t_end and (st["status"] is None or st["n_sport"] == 0 or st["n_low"] == 0):
            rclpy.spin_once(node, timeout_sec=0.05)
        s = st["status"]
        if s is None:
            probs.append("no /go2/cmd_bridge/status in %.0f s -- is the bridge running?" % STATUS_TIMEOUT_S)
        else:
            if bool(s.get("armed")) != args.armed:
                probs.append("bridge armed=%s but this run says %s" % (s.get("armed"), "--armed" if args.armed else "--dry"))
            if s.get("state") != args.require_state:
                probs.append("bridge state %s (%s), need %s" % (s.get("state"), s.get("reason"), args.require_state))
        if st["n_sport"] == 0 or st["n_low"] == 0:
            probs.append("robot telemetry missing (sport %d, low %d)" % (st["n_sport"], st["n_low"]))
        event("preflight", detail=json.dumps({"status": s, "sport": st["n_sport"], "low": st["n_low"]}))
        if probs:
            for p in probs:
                event("refuse", detail=p)
            return 2

        for step in steps:
            if step[0] == "wait":
                spin_for(step[1])
            elif step[0] == "kill_bridge":
                event("kill_bridge", detail=BRIDGE_PROCESS)
                subprocess.run(["pkill", "-KILL", "-f", BRIDGE_PROCESS])
            else:
                _, vx, vyaw, secs = step
                streamed = True
                for _ in range(int(round(secs * RATE_HZ))):
                    if st["state"] in ("ABORTED", "YIELDED"):
                        event("stream_stopped", detail="bridge %s" % st["state"])
                        break
                    twist(vx, vyaw)
                    spin_for(1.0 / RATE_HZ)
        event("done", detail=args.profile)
    except BaseException as exc:          # Ctrl+C included
        event("abort", detail="%s: %s" % (type(exc).__name__, exc))
        rc = 1
        if streamed:
            sig["n"] = None               # let the flush below spin
            for _ in range(3):
                twist(0.0, 0.0)           # bridge: zero Twist -> one StopMove (D3)
                spin_for(0.05)
    finally:
        for fh in files.values():
            fh.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report = analyse_bridge(run_dir)
        with open(os.path.join(run_dir, "report.txt"), "w") as fh:
            fh.write(report + "\n")
        print("\n" + report)
    return rc


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("profile", nargs="?", choices=sorted(PROFILES))
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--armed", action="store_true", help="the bridge must report armed=true")
    mode.add_argument("--dry", action="store_true", help="the bridge must report armed=false")
    ap.add_argument("--require-state", default="IDLE", choices=["IDLE", "WAITING"])
    ap.add_argument("--out", default="/logs")
    ap.add_argument("--replay", metavar="RUN_DIR")
    ap.add_argument("--plan", action="store_true", help="print the profile and exit; no ROS")
    args = ap.parse_args(argv)
    if args.replay:
        print(analyse_bridge(args.replay))
        return 0
    if not args.profile:
        ap.error("a profile is required (or --replay)")
    if args.plan:
        print(describe(args.profile))
        return 0
    if not (args.armed or args.dry):
        ap.error("say which bridge you expect: --armed or --dry")
    return run_live(args)


if __name__ == "__main__":
    sys.exit(main())
