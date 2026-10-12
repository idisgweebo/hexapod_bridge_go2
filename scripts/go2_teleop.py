#!/usr/bin/env python3
"""Gate 7 §10 -- hold-to-move keyboard teleop for go2_cmd_bridge (option C).

Plan: gate7_go2_cmdvel_bridge_plan.md §10. Publishes ONLY /go2/cmd_vel -- never a robot
topic. Whether that moves the robot is the bridge's decision (armed or dry), which this
script checks before it sends anything.

    W / S   forward / back      0.3 m/s  (= the bridge's hard cap)
    A / D   turn left / right   1.0 rad/s
    Space   stop now            Q / Esc / close window   stop and quit

HOLD TO MOVE. A 10 Hz timer streams the command while a key is DOWN; auto-repeat is
never what keeps the robot moving (measured 8 Oct: the first repeat comes ~500 ms after
the press, longer than the bridge's 0.4 s watchdog -- §10c).

X11 auto-repeat delivers a KeyRelease+KeyPress pair per repeat, both with the SAME X
server time (211/211 pairs, §10c). A release is therefore held for DEBOUNCE_S and
cancelled if a press of the same key with the same X time arrives; otherwise it is real.
On a real release of the last motion key the window publishes a ZERO Twist at once (the
bridge turns a fresh zero into one StopMove on its next tick; silence would wait for the
0.4 s watchdog). Focus loss, Space, quit, Ctrl+C, a bridge YIELD/ABORT: stop at once.

Key-down STATE drives the command, never "repeats are still arriving": X auto-repeats
only the most recent key, so holding W and adding A silences W's repeats with no release.

PREFLIGHT (nothing is published until all hold): ROS_DOMAIN_ID exported as 0, Cyclone;
/go2/cmd_bridge/status received, its `armed` == --armed/--dry as given here, its state
IDLE or WAITING.  A bridge YIELDED or ABORTED latches this window too: restart both.

Records run_dir/{events,sport,low}.csv; the events include every key event with its X
time and role, every Twist, every bridge request -- enough to measure stop latency.
"""
import argparse
import csv
import json
import math
import os
import signal
import sys
import time

CMD_TOPIC = "/go2/cmd_vel"
RATE_HZ = 10.0
VX_CAP, VYAW_CAP = 0.3, 1.0         # = go2_cmd_bridge's hard caps
DEBOUNCE_S = 0.020                  # §10c: pairs arrive < 0.25 ms apart; taps are held >= 109 ms
POLL_MS = 10                        # the Tk timer: resolves releases and paces publishing
ZERO_BURST = 3                      # zero Twists after motion stops (one is enough; 3 for loss)
STATUS_TIMEOUT_S = 5.0
STATUS_STALE_S = 1.5                # bridge status is 2 Hz
OK_STATES = ("IDLE", "WAITING", "DRIVING")
LATCH_STATES = ("YIELDED", "ABORTED")

# key -> (vx sign, vyaw sign)
KEYMAP = {"w": (1, 0), "s": (-1, 0), "a": (0, 1), "d": (0, -1)}
STOP_KEYS = ("space",)
QUIT_KEYS = ("q", "escape")


class TeleopError(ValueError):
    pass


class TeleopCore:
    """Pure: key events in, Twists out. No ROS, no Tk; every time is passed in."""

    def __init__(self, vx=VX_CAP, vyaw=VYAW_CAP, debounce=DEBOUNCE_S, rate_hz=RATE_HZ):
        for label, v, cap in (("vx", vx, VX_CAP), ("vyaw", vyaw, VYAW_CAP)):
            if not math.isfinite(v) or not 0 < v <= cap + 1e-9:
                raise TeleopError("%s=%r outside (0, %s]" % (label, v, cap))
        if not 0 <= debounce <= 0.1:
            raise TeleopError("debounce %r outside [0, 0.1] s -- it adds to stop latency" % debounce)
        self.vx, self.vyaw, self.debounce, self.period = vx, vyaw, debounce, 1.0 / rate_hz
        self.down = set()
        self.pending = {}           # key -> (xtime, deadline)
        self.last_out = (0.0, 0.0)
        self.last_pub_t = None
        self.zeros_left = 0
        self.latched = None         # reason; once set, only zeros, then silence
        self.released = []          # keys whose release became real on the last tick
        self.orphan_rel = {}        # key -> xtime of a release seen while the key was NOT down
        self.counts = {"press": 0, "autorepeat": 0, "real_release": 0, "twist": 0, "zero": 0}

    # ---- events: each returns the role it assigned (for the log)
    def press(self, key, xtime, now):
        key = key.lower()
        if key not in KEYMAP:
            return None
        p = self.pending.pop(key, None)
        if p is not None and p[0] == xtime:
            self.counts["autorepeat"] += 1
            return "autorepeat"
        if key not in self.down and self.orphan_rel.pop(key, None) == xtime:
            # Auto-repeat of a key physically held THROUGH a stop (Space, focus loss): its
            # release half arrived while the key was up. Not a new press -- motion after a
            # stop needs a real key-down. (Offline test 8 caught this restarting in ~30 ms.)
            self.counts["held_through_stop"] = self.counts.get("held_through_stop", 0) + 1
            return "held_through_stop"
        if key in self.down:
            # a press with no pending release: an auto-repeat whose release we never saw,
            # or a re-press inside the debounce with a new X time -- either way, still down
            return "repress"
        self.down.add(key)
        self.counts["press"] += 1
        return "down"

    def release(self, key, xtime, now):
        key = key.lower()
        if key not in self.down:
            if key in KEYMAP:
                self.orphan_rel[key] = xtime
            return None
        self.pending[key] = (xtime, now + self.debounce)
        return "pending"

    def stop(self, reason=None, latch=False):
        """Space, focus loss, quit, signal, bridge latch: everything up, zeros follow."""
        # A stop can land between the two halves of an auto-repeat pair: keep the pending
        # release's X time so its press half is still recognised (offline test 8b).
        for k, (xt, _) in self.pending.items():
            self.orphan_rel[k] = xt
        self.down.clear()
        self.pending.clear()
        if latch and self.latched is None:
            self.latched = reason or "latched"

    def command(self):
        if self.latched:
            return (0.0, 0.0)
        sx = sum(KEYMAP[k][0] for k in self.down)
        sz = sum(KEYMAP[k][1] for k in self.down)
        return (self.vx * max(-1, min(1, sx)), self.vyaw * max(-1, min(1, sz)))

    def tick(self, now):
        """Resolve due releases; return (vx, vyaw) to publish now, or None."""
        released = []
        for k, (_, deadline) in list(self.pending.items()):
            if now >= deadline:
                del self.pending[k]
                self.down.discard(k)
                self.counts["real_release"] += 1
                released.append(k)
        cmd = self.command()
        due = self.last_pub_t is None or now - self.last_pub_t >= self.period - 1e-3
        out = None
        if cmd != (0.0, 0.0):
            if cmd != self.last_out or due:
                out = cmd
            self.zeros_left = ZERO_BURST
        elif self.zeros_left > 0 and (self.last_out != (0.0, 0.0) or due):
            out = (0.0, 0.0)
            self.zeros_left -= 1
        if out is not None:
            self.last_out, self.last_pub_t = out, now
            self.counts["zero" if out == (0.0, 0.0) else "twist"] += 1
        self.released = released
        return out


# ---------------------------------------------------------------------------
# Live: rclpy + Tk, one thread. If the GUI hangs, publishing stops with it and the
# bridge's watchdog stops the robot -- the deadman holds through a frozen window.
# ---------------------------------------------------------------------------
def run_live(args):
    import tkinter as tk
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import go2_sport_probe as probe          # pure at import: check_env, JOINT

    try:
        from unitree_go.msg import SportModeState, LowState
    except ImportError:
        SportModeState = LowState = None      # recorder off; the bridge still guards temps

    core = TeleopCore(vx=args.vx, vyaw=args.vyaw, debounce=args.debounce)
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    run_dir = os.path.join(args.out, "teleop_%s_%s" % ("armed" if args.armed else "dry", stamp))
    os.makedirs(run_dir, exist_ok=True)
    files = {}

    def writer(name, head):
        fh = open(os.path.join(run_dir, name), "w", newline="")
        w = csv.writer(fh)
        w.writerow(head)
        files[name] = fh
        return w

    w_ev = writer("events.csv", ["t", "utc", "kind", "key", "xtime", "role", "detail"])
    w_sp = writer("sport.csv", ["t", "error_code", "body_height", "px", "py", "vx", "yaw_speed", "yaw"])
    w_lo = writer("low.csv", ["t"] + ["temp_" + n for n in probe.JOINT])
    RR_HIP, RL_HIP = probe.JOINT.index("RR_hip"), probe.JOINT.index("RL_hip")

    def event(kind, key="", xtime="", role="", detail=""):
        t = time.time()
        w_ev.writerow(["%.6f" % t, time.strftime("%H:%M:%S", time.gmtime(t)), kind, key, xtime, role, detail])
        files["events.csv"].flush()
        if kind not in ("key", "twist", "sent", "dry_dry"):
            print("[%s] %-12s %s %s" % (time.strftime("%H:%M:%SZ", time.gmtime(t)), kind, role, detail))

    st = {"status": None, "status_t": 0.0, "state": None, "hips": None, "n_sport": 0, "n_low": 0}

    class Teleop(Node):
        def __init__(self):
            super().__init__("go2_teleop")
            # The only publisher in this file -- a /go2/ topic, never a robot topic.
            self.pub = self.create_publisher(Twist, "/go2/cmd_vel", 10)
            self.create_subscription(String, "/go2/cmd_bridge/status", self.on_status, 10)
            self.create_subscription(String, "/go2/cmd_bridge/requests", self.on_breq, 10)
            if SportModeState is not None:
                self.create_subscription(SportModeState, "/sportmodestate", self.on_sport, 10)
                self.create_subscription(LowState, "/lowstate", self.on_low, 10)

        def on_status(self, m):
            s = json.loads(m.data)
            st["status"], st["status_t"] = s, time.time()
            if s.get("state") != st["state"]:
                st["state"] = s.get("state")
                event("bridge_state", detail="%s (%s)" % (s.get("state"), s.get("reason")))

        def on_breq(self, m):
            r = json.loads(m.data)
            kind = "sent" if r["routed"] == "robot" else "dry_" + r["routed"]
            event(kind, role=r["name"], detail=r["parameter"] or "")

        def on_sport(self, m):
            st["n_sport"] += 1
            if st["n_sport"] % 6 == 0:
                w_sp.writerow(["%.6f" % time.time(), m.error_code, "%.5f" % m.body_height,
                               "%.5f" % m.position[0], "%.5f" % m.position[1], "%.5f" % m.velocity[0],
                               "%.5f" % m.yaw_speed, "%.6f" % m.imu_state.rpy[2]])

        def on_low(self, m):
            st["n_low"] += 1
            st["hips"] = (m.motor_state[RR_HIP].temperature, m.motor_state[RL_HIP].temperature)
            if st["n_low"] % 50 == 0:
                w_lo.writerow(["%.6f" % time.time()] + [m.motor_state[i].temperature for i in range(12)])

    # rclpy's SIGINT handler would shut the context before our zeros (cmd_vel_profile P6).
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    sig = {"n": None}
    signal.signal(signal.SIGINT, lambda n, f: sig.__setitem__("n", n))
    signal.signal(signal.SIGTERM, lambda n, f: sig.__setitem__("n", n))
    node = Teleop()

    def publish(out):
        t = Twist()
        t.linear.x, t.angular.z = float(out[0]), float(out[1])
        node.pub.publish(t)
        event("twist", detail="%.3f %.3f" % out)

    # ---- preflight: nothing is published before this passes
    probs = probe.check_env(os.environ)
    t_end = time.time() + STATUS_TIMEOUT_S
    while time.time() < t_end and st["status"] is None:
        rclpy.spin_once(node, timeout_sec=0.05)
    s = st["status"]
    if s is None:
        probs.append("no /go2/cmd_bridge/status in %.0f s -- is the bridge running?" % STATUS_TIMEOUT_S)
    else:
        if bool(s.get("armed")) != args.armed:
            probs.append("bridge armed=%s but this run says %s" % (s.get("armed"), "--armed" if args.armed else "--dry"))
        if s.get("state") not in ("IDLE", "WAITING"):
            probs.append("bridge state %s (%s), need IDLE or WAITING" % (s.get("state"), s.get("reason")))
    event("preflight", detail=json.dumps({"status": s, "args": vars(args)}))
    if probs:
        for p in probs:
            event("refuse", detail=p)
        node.destroy_node()
        rclpy.shutdown()
        for fh in files.values():
            fh.close()
        return 2

    # ---- the window
    root = tk.Tk()
    root.title("go2_teleop -- %s" % ("ARMED" if args.armed else "dry run"))
    big = ("TkDefaultFont", 16, "bold")
    mode_lbl = tk.Label(root, font=big, fg="white", bg="#b00020" if args.armed else "#555555",
                        text="ARMED -- the robot moves" if args.armed else "DRY RUN -- nothing reaches the robot")
    mode_lbl.pack(fill="x")
    cmd_lbl = tk.Label(root, font=("TkFixedFont", 14), pady=8)
    cmd_lbl.pack()
    br_lbl = tk.Label(root, font=("TkFixedFont", 11))
    br_lbl.pack()
    hip_lbl = tk.Label(root, font=("TkFixedFont", 11))
    hip_lbl.pack()
    tk.Label(root, justify="left", padx=20, pady=10, text=(
        "HOLD  W / S   forward / back   (%.1f m/s)\n"
        "HOLD  A / D   turn left / right (%.1f rad/s)\n"
        "Space  stop      Q / Esc  quit\n"
        "Click away = stop.  Handset L2+A always wins." % (core.vx, core.vyaw))).pack()
    state = {"quit": None, "n": 0}

    def on_press(e):
        ks = (e.keysym or "").lower()
        if ks in QUIT_KEYS:
            state["quit"] = "key " + ks
            return
        if ks in STOP_KEYS:
            core.stop()
            event("key", ks, e.time, "stop")
            return
        role = core.press(ks, e.time, time.monotonic())
        if role:        # autorepeat too: T0's PT0 counts pairs per hold (~33/s, cheap)
            event("key", ks, e.time, role)

    def on_release(e):
        ks = (e.keysym or "").lower()
        core.release(ks, e.time, time.monotonic())

    def on_focus_out(e):
        # unconditional: a spurious FocusOut costs a stop, a missed one costs a walk
        core.stop()
        event("focus_out", role="stop")

    def on_focus_in(e):
        event("focus_in")

    root.bind("<KeyPress>", on_press)
    root.bind("<KeyRelease>", on_release)
    root.bind("<FocusOut>", on_focus_out)
    root.bind("<FocusIn>", on_focus_in)
    root.protocol("WM_DELETE_WINDOW", lambda: state.__setitem__("quit", "window closed"))

    def poll():
        if sig["n"] is not None and not state["quit"]:
            state["quit"] = "signal %d" % sig["n"]
        rclpy.spin_once(node, timeout_sec=0)
        s = st["status"]
        now_w = time.time()
        if s and s.get("state") in LATCH_STATES and not core.latched:
            core.stop("bridge %s (%s)" % (s.get("state"), s.get("reason")), latch=True)
            event("latch", detail=core.latched)
        if s and bool(s.get("armed")) != args.armed and not core.latched:
            # the bridge was restarted in the other mode under us
            core.stop("bridge armed=%s, this window %s" % (s.get("armed"), "armed" if args.armed else "dry"),
                      latch=True)
            event("latch", detail=core.latched)
        if state["quit"]:
            core.stop()
        out = core.tick(time.monotonic())
        for k in core.released:
            event("key", k, "", "real_release")
        if out is not None:
            publish(out)
        state["n"] += 1
        if state["n"] % 10 == 0:
            stale = now_w - st["status_t"] > STATUS_STALE_S
            vx, vz = core.command()
            cmd_lbl.config(text="vx %+.2f m/s   vyaw %+.2f rad/s" % (vx, vz),
                           fg="#b00020" if (vx or vz) else "black")
            if core.latched:
                br_lbl.config(text="LATCHED: %s -- restart bridge and teleop" % core.latched, fg="#b00020")
            elif stale:
                br_lbl.config(text="bridge status STALE (%.1f s) -- the bridge may be gone"
                              % (now_w - st["status_t"]), fg="#b00020")
            else:
                br_lbl.config(text="bridge %s (%s)" % (s.get("state"), s.get("reason")), fg="black")
            h = st["hips"]
            hip_lbl.config(text="rear hips RR %s / RL %s C   (bridge aborts at 50)" % h if h else
                           "rear hips: no /lowstate", fg="#b00020" if h and max(h) >= 45 else "black")
        if state["quit"] and core.zeros_left == 0:
            root.destroy()
            return
        root.after(POLL_MS, poll)

    event("start", detail="armed=%s vx=%.2f vyaw=%.2f debounce=%.3f" % (args.armed, core.vx, core.vyaw, core.debounce))
    print("run dir: " + run_dir)
    root.after(POLL_MS, poll)
    root.focus_force()
    try:
        root.mainloop()
    finally:
        # however we got here: zeros out, while the context is still alive
        if core.last_out != (0.0, 0.0) or core.zeros_left:
            for _ in range(ZERO_BURST):
                publish((0.0, 0.0))
                rclpy.spin_once(node, timeout_sec=0.02)
        event("quit", detail="%s; counts %s" % (state["quit"], json.dumps(core.counts)))
        for fh in files.values():
            fh.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--armed", action="store_true", help="the bridge must be ARMED")
    g.add_argument("--dry", action="store_true", help="the bridge must be DRY")
    ap.add_argument("--vx", type=float, default=VX_CAP)
    ap.add_argument("--vyaw", type=float, default=VYAW_CAP)
    ap.add_argument("--debounce", type=float, default=DEBOUNCE_S)
    ap.add_argument("--out", default="/logs")
    a = ap.parse_args()
    try:
        TeleopCore(vx=a.vx, vyaw=a.vyaw, debounce=a.debounce)
    except TeleopError as e:
        print("REFUSING: %s" % e)
        return 2
    return run_live(a)


if __name__ == "__main__":
    sys.exit(main())
