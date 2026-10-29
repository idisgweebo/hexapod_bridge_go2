#!/usr/bin/env python3
"""Offline tests for go2_adapter/state_core.py -- NO ROS, NO container, NO robot.

Plan: phase1_go2_feedback_plan.md (§3 schema v1, D10, PFC-7). Run before any robot.

  1  params: unknown keys, non-positive values and warn >= fault refuse
  2  helpers: wrap_pi, fmt_stamp (the echo's wire form), Throttle's average rate
  3  D10 motion on synthetic walks, turns, sway, spikes and gaps
  4  status composition: staleness, faults, warnings, echo, the schema's key list
  5  the yaml block names exactly the node's parameters (a typo must not fall back)
  6  clean interpreter: state_core imports with no rclpy anywhere
"""
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, "..", "ros2_ws", "src", "go2_adapter"))
sys.path.insert(0, PKG)
from go2_adapter import state_core as sc                          # noqa: E402

n_checks = 0
fails = []


def check(label, ok, detail=""):
    global n_checks
    n_checks += 1
    print(("  ok   " if ok else "  FAIL ") + label + ("" if ok else "  -- " + str(detail)[:300]))
    if not ok:
        fails.append(label)


def raises(fn, exc=Exception):
    try:
        fn()
    except exc:
        return True
    return False


# --------------------------------------------------------------- 1 params ---
print("1    params")
check("defaults validate", sc.validate_params() == sc.DEFAULTS)
check("unknown parameter refuses", raises(lambda: sc.validate_params({"v_move": 0.1}), sc.ConfigError))
check("zero / negative / NaN refuse",
      all(raises(lambda v=v: sc.validate_params({"window_s": v}), sc.ConfigError) for v in (0, -1, float("nan"))))
check("hip_warn_c >= hip_fault_c refuses",
      raises(lambda: sc.validate_params({"hip_warn_c": 50.0}), sc.ConfigError))
check("thermal defaults match go2_cmd_bridge (45 arm, 50 abort)",
      sc.DEFAULTS["hip_warn_c"] == 45.0 and sc.DEFAULTS["hip_fault_c"] == 50.0)

# -------------------------------------------------------------- 2 helpers ---
print("2    helpers")
check("wrap_pi folds 3*pi/2 to -pi/2", abs(sc.wrap_pi(1.5 * math.pi) + 0.5 * math.pi) < 1e-12)
check("wrap_pi across the seam: (-3.1) - (3.1) is small", abs(sc.wrap_pi(-3.1 - 3.1)) < 0.1)
check("fmt_stamp pads nanoseconds to 9 digits", sc.fmt_stamp(100, 5) == "100.000000005")
check("fmt_stamp: distinct IDs never collide by rounding",
      sc.fmt_stamp(1761700000, 123456789) != sc.fmt_stamp(1761700000, 123456790))

th = sc.Throttle(20.0)
n = sum(th.allow(i / 150.0) for i in range(1500))             # 10 s of 150 Hz
check("Throttle: 150 Hz in, 20 Hz target -> 200 +/- 2 out over 10 s", 198 <= n <= 202, n)
th = sc.Throttle(20.0)
out = [t for t in (i / 150.0 for i in range(150)) if th.allow(t)]
gap_out = [t for t in (5.0 + i / 150.0 for i in range(150)) if th.allow(t)]
check("Throttle: after a 4 s input gap, no catch-up burst",
      all(b - a > 0.04 for a, b in zip(gap_out, gap_out[1:])), gap_out[:4])
check("Throttle refuses a zero rate", raises(lambda: sc.Throttle(0), sc.ConfigError))


# ------------------------------------------------------------- 3 motion ---
class Body(object):
    """A synthetic Go2 reporting SportModeState at 50 Hz (the s13 recorder rate)."""

    def __init__(self, params=None):
        self.m = sc.MotionDetector(sc.validate_params(params))
        self.t, self.x, self.y, self.yaw = 100.0, 0.0, 0.0, 0.0
        self.trace = []

    def run(self, secs, v=0.0, w=0.0, rep_v=None, rep_w=None, sway=0.0, dt=0.02):
        """Integrate true motion (v, w); report rep_v / rep_w as the estimator's
        velocity (default: the truth); add a +/-sway m oscillation in x."""
        end = self.t + secs
        k = 0
        while self.t < end - 1e-9:
            self.t += dt
            self.yaw += w * dt
            self.x += v * dt * math.cos(self.yaw)
            self.y += v * dt * math.sin(self.yaw)
            sx = sway * math.sin(2 * math.pi * 2.0 * k * dt)          # 2 Hz sway
            st = self.m.update(self.t, (v if rep_v is None else rep_v), 0.0,
                               (w if rep_w is None else rep_w), self.x + sx, self.y, self.yaw)
            self.trace.append((round(self.t, 3), st))
            k += 1
        return self.m.state

    def first(self, state, after):
        for t, s in self.trace:
            if t > after and s == state:
                return t
        return None


print("3    D10 motion")
b = Body()
check("fresh detector is UNKNOWN", b.m.state == sc.UNKNOWN)
b.run(0.2)
check("...and stays UNKNOWN for less than the window (0.2 s of rest)", b.m.state == sc.UNKNOWN)
b.run(0.3)
check("rest becomes STOPPED after the window", b.m.state == sc.STOPPED)
t0 = b.t
b.run(2.5, v=0.3)
tm = b.first(sc.MOVING, t0)
check("0.3 m/s walk -> MOVING", b.m.state == sc.MOVING)
check("...within 0.4 s of motion starting (PFC-8 budget is 0.8 s end to end)",
      tm is not None and tm - t0 <= 0.4, None if tm is None else tm - t0)
check("...and never STOPPED once MOVING during the walk",
      all(s == sc.MOVING for t, s in b.trace if tm is not None and t >= tm))
t1 = b.t
b.run(1.0)
ts = b.first(sc.STOPPED, t1)
check("walk ends -> STOPPED within 0.4 s", ts is not None and ts - t1 <= 0.4, None if ts is None else ts - t1)

b = Body()
b.run(1.0)
t0 = b.t
b.run(2.0, w=1.0)
check("turn in place at 1.0 rad/s -> MOVING (via yaw rate + heading change)", b.m.state == sc.MOVING)

b = Body()
b.run(1.0)
b.run(0.02, rep_v=0.637)          # s12: one 0.637 m/s sample on 7.6 mm of motion
b.run(1.0)
check("a single-sample velocity spike never reads MOVING",
      all(s != sc.MOVING for _, s in b.trace), [x for x in b.trace if x[1] == sc.MOVING][:3])

b = Body()
b.run(1.0)
b.run(2.0, rep_v=0.2)             # estimator says 0.2 m/s, body does not move
check("velocity WITHOUT displacement never reads MOVING", all(s != sc.MOVING for _, s in b.trace))

b = Body()
b.run(1.0)
# A yaw-rate blip plus a steady 4 cm/s position creep (1.2 cm per window, over d_mov),
# no real turn, reported speed 0. Only the pairing keeps this STOPPED.
# ❌ First version used a 2 Hz +/-2 cm sway instead: its displacement crosses zero every
# 0.25 s, so the candidate never held for the window and an UNPAIRED rule passed too
# (mutation run, 29 Oct). A test that its own mutant passes tests nothing.
b.run(2.0, v=0.04, rep_v=0.0, rep_w=0.3)
check("yaw rate cannot be vouched for by translation (pairing)",
      all(s != sc.MOVING for _, s in b.trace))

b = Body()
b.run(1.0)
b.run(2.0, v=0.3)
b.t += 0.5                         # a 0.5 s hole in /sportmodestate
b.run(0.02, v=0.3)
check("a gap longer than the window resets to UNKNOWN (no bridging across it)",
      b.m.state == sc.UNKNOWN, b.m.state)
b.run(1.0, v=0.3)
check("...and recovers to MOVING once data resumes", b.m.state == sc.MOVING)


# ------------------------------------------------------------- 4 status ---
def healthy(core, now, motion_walk=False, bridge=None, hips=(35, 34), code=100, modes=None):
    """Feed 1 s of healthy inputs ending at `now`."""
    x = 0.0
    for i in range(51):
        t = now - 1.0 + i * 0.02
        v = 0.3 if motion_walk else 0.0
        x += v * 0.02
        core.on_sport(t, code, v, 0.0, 0.0, x, 0.0, 0.0)
        core.on_low(t, modes or [1] * 12, hips[0], hips[1])
    core.on_bridge(now, bridge or {"state": "IDLE", "reason": "no fresh command", "input_type": "twist_stamped",
                                   "cmd_last_stamp": None, "cmd_age_s": None})
    core.on_clock(-0.84, "CONVERGED")


def vals(core, now):
    level, msg, v = core.status(now)
    return level, msg, dict(v), [k for k, _ in v]


print("4    status composition")
c = sc.Go2StateCore()
level, msg, v, keys = vals(c, 10.0)
check("no inputs at all: level STALE, motion UNKNOWN", level == sc.STALE and v["motion"] == sc.UNKNOWN, (level, v))
check("...with every stale input named in fault",
      all(f in v["fault"] for f in ("sport_stale", "low_stale", "bridge_stale")), v["fault"])
check("schema v1: the key list is exactly the documented one, in order",
      keys == ["schema_version", "cmd_last_stamp", "cmd_age_s", "bridge_state", "bridge_reason",
               "bridge_input", "bridge_armed", "motion", "speed_mps", "yaw_rate_rps", "sport_state_code", "fault", "warn",
               "rr_hip_c", "rl_hip_c", "go2_clock_offset_s", "clock_state", "odom_covariance_known"], keys)
check("every value is a string (KeyValue.value is a string)",
      all(isinstance(x, str) for x in v.values()))
check("odom_covariance_known is 'false' (D8)", v["odom_covariance_known"] == "false")

c = sc.Go2StateCore()
healthy(c, 50.0)
level, msg, v, _ = vals(c, 50.0)
check("healthy, standing, bridge IDLE, clock converged: level OK, fault none, STOPPED",
      level == sc.OK and v["fault"] == "none" and v["warn"] == "none" and v["motion"] == sc.STOPPED, (level, v))

c = sc.Go2StateCore()
healthy(c, 50.0, motion_walk=True)
check("healthy walk: MOVING", vals(c, 50.0)[2]["motion"] == sc.MOVING)
level, msg, v, _ = vals(c, 50.6)
check("⛔ the feed dies mid-walk: 0.6 s later motion is UNKNOWN, NEVER STOPPED",
      v["motion"] == sc.UNKNOWN and level == sc.STALE and "sport_stale" in v["fault"], (level, v))
check("...and speed/yaw/hip fields are blanked, not left at their last values",
      v["speed_mps"] == "" and v["rr_hip_c"] == "", v)

c = sc.Go2StateCore()
healthy(c, 50.0, modes=[1] * 6 + [0] + [1] * 5)
level, _, v, _ = vals(c, 50.0)
check("motor mode != 1 on RR_hip -> ERROR, named", level == sc.ERROR and v["fault"] == "motor_mode:RR_hip", v["fault"])
c = sc.Go2StateCore()
healthy(c, 50.0, modes=[1] * 11)
check("eleven motors reported -> fault, not a silent pass",
      "motor_mode:motor_count_11" in vals(c, 50.0)[2]["fault"])
c = sc.Go2StateCore()
healthy(c, 50.0, hips=(51, 40))
level, _, v, _ = vals(c, 50.0)
check("rear hip 51 C -> ERROR rear_hip_hot:51", level == sc.ERROR and v["fault"] == "rear_hip_hot:51", v)
c = sc.Go2StateCore()
healthy(c, 50.0, hips=(40, 46))
level, _, v, _ = vals(c, 50.0)
check("rear hip 46 C -> WARN only", level == sc.WARN and v["fault"] == "none" and v["warn"] == "rear_hip_warm:46", v)
c = sc.Go2StateCore()
healthy(c, 50.0, code=1234)
level, _, v, _ = vals(c, 50.0)
check("unseen sport code -> WARN 'unrecognised', never ERROR (rule 9)",
      level == sc.WARN and v["warn"] == "sport_code_unrecognised:1234" and v["fault"] == "none", v)
for code in (100, 1001, 1002, 2006):
    c = sc.Go2StateCore()
    healthy(c, 50.0, code=code)
    check("rule 9 code %d is known (no warning)" % code, vals(c, 50.0)[2]["warn"] == "none")

c = sc.Go2StateCore()
healthy(c, 50.0, bridge={"state": "ABORTED", "reason": "rear hip 50 C >= 50 C"})
level, _, v, _ = vals(c, 50.0)
check("bridge ABORTED -> ERROR with its reason", level == sc.ERROR and v["fault"] == "bridge_aborted:rear hip 50 C >= 50 C", v)
c = sc.Go2StateCore()
healthy(c, 50.0, bridge={"state": "YIELDED", "reason": "handset activity (keys=32)"})
check("bridge YIELDED -> fault bridge_yielded", vals(c, 50.0)[2]["fault"] == "bridge_yielded")
c = sc.Go2StateCore()
healthy(c, 50.0, code=1001, bridge={"state": "WAITING", "reason": "error_code 1001, not 100 (balance stand)"})
level, _, v, _ = vals(c, 50.0)
check("bridge WAITING (robot lying) -> WARN, not a fault",
      level == sc.WARN and v["fault"] == "none" and v["warn"].startswith("bridge_waiting:"), v)

c = sc.Go2StateCore()
healthy(c, 50.0, bridge={"state": "DRIVING", "reason": "fresh command", "input_type": "twist_stamped",
                         "cmd_last_stamp": [1761700000, 5], "cmd_age_s": 0.042})
v = vals(c, 50.0)[2]
check("echo: cmd_last_stamp in wire form, age to the ms",
      v["cmd_last_stamp"] == "1761700000.000000005" and v["cmd_age_s"] == "0.042", v)
check("bridge_state / bridge_input pass through", v["bridge_state"] == "DRIVING" and v["bridge_input"] == "twist_stamped")
check("bridge_armed is '' when the bridge status does not say", v["bridge_armed"] == "")
c2 = sc.Go2StateCore()
healthy(c2, 50.0, bridge={"state": "IDLE", "reason": "", "armed": True})
check("bridge_armed 'true' passes through", vals(c2, 50.0)[2]["bridge_armed"] == "true")
c2 = sc.Go2StateCore()
healthy(c2, 50.0, bridge={"state": "IDLE", "reason": "", "armed": False})
check("bridge_armed 'false' passes through", vals(c2, 50.0)[2]["bridge_armed"] == "false")
c.on_bridge(50.5, None)                                  # unparseable status
c.on_bridge(50.6, {"no": "state"})                       # parseable but not a bridge status
v = vals(c, 51.6)[2]
check("bad bridge messages do NOT refresh its freshness (stale 1.6 s after the last good one)",
      "bridge_stale" in v["fault"] and v["cmd_last_stamp"] == "" and c.bridge_bad == 2, (v["fault"], c.bridge_bad))

c = sc.Go2StateCore()
healthy(c, 50.0)
c.on_clock(None, "FAULTED")
level, _, v, _ = vals(c, 50.0)
check("clock step -> ERROR clock_step", level == sc.ERROR and v["fault"] == "clock_step", v)
c.on_clock(None, "CONVERGING")
check("clock converging -> WARN", vals(c, 50.0)[0] == sc.WARN)

# ---------------------------------------------------------- 5 yaml block ---
print("5    yaml")
NODE_ONLY = {"odom_rate_hz", "imu_rate_hz", "status_rate_hz", "clock_window", "step_threshold_s", "step_consecutive"}
yaml_path = os.path.join(PKG, "config", "go2_adapter.yaml")
text = open(yaml_path).read()
m = re.search(r"^go2_state_node:\n  ros__parameters:\n((?:    .*\n|\s*\n)+)", text, re.M)
keys = set(re.findall(r"^    (\w+):", m.group(1), re.M)) if m else set()
check("yaml has a go2_state_node block (the node name: a mismatch drops every parameter)", bool(m))
check("yaml keys == core DEFAULTS + the node's own parameters",
      keys == set(sc.DEFAULTS) | NODE_ONLY, sorted(keys ^ (set(sc.DEFAULTS) | NODE_ONLY)))
node_src = open(os.path.join(PKG, "go2_adapter", "go2_state_node.py")).read()
check("the node declares every node-only parameter",
      all("'%s'" % k in node_src for k in NODE_ONLY), [k for k in NODE_ONLY if "'%s'" % k not in node_src])
setup_src = open(os.path.join(PKG, "setup.py")).read()
check("setup.py: executable go2_state_node -> go2_state_node:main",
      "'go2_state_node = go2_adapter.go2_state_node:main'" in setup_src)

# ---------------------------------------------------- 6 clean interpreter ---
print("6    clean interpreter")
probe = ("import sys; sys.path.insert(0, %r)\n"
         "from go2_adapter import state_core\n"
         "assert 'rclpy' not in sys.modules\n"
         "print('CLEAN')\n" % PKG)
p = subprocess.run([sys.executable, "-E", "-s", "-c", probe], capture_output=True, text=True)
check("state_core imports without rclpy", p.returncode == 0 and "CLEAN" in p.stdout, p.stderr[-300:])

print()
print("%d checks, %d failure(s)" % (n_checks, len(fails)))
if n_checks < 50:
    print("FAIL: fewer checks ran than written -- refusing to report success")
    sys.exit(1)
print("ALL GO2 STATE OFFLINE TESTS PASS" if not fails else "FAILURES:\n  - " + "\n  - ".join(fails))
sys.exit(1 if fails else 0)
