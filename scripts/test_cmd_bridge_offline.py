#!/usr/bin/env python3
"""Offline tests for go2_cmd_bridge -- NO ROS, NO container, NO robot.

Plan: gate7_go2_cmdvel_bridge_plan.md §4.1. Run before the Go2 is powered.

  1  allowlist: only MOVE / STOPMOVE / STANDDOWN can be built; Move over the hard
     cap raises; StandUp, BalanceStand and Damp are unbuildable
  2  params: YAML can lower the clamps, never raise them; nonsense refuses
  3  the state machine on synthetic time, one rule per scenario
  4  package audit (AST): one robot publisher, literal; everything else under /go2/
  5  anti-vacuity: the auditor FAILS deliberately bad sources
  6  clean interpreter: the core imports with no rclpy anywhere
"""
import ast
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, "..", "ros2_ws", "src", "go2_cmd_bridge"))
sys.path.insert(0, PKG)
from go2_cmd_bridge import bridge_core as bc                      # noqa: E402

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


# ------------------------------------------------------------- 1 allowlist ---
print("1    allowlist")
check("allowlist is exactly MOVE, STOPMOVE, STANDDOWN with the vendor ids",
      bc.API == {"MOVE": 1008, "STOPMOVE": 1003, "STANDDOWN": 1005}, bc.API)
for bad in ("STANDUP", "BALANCESTAND", "DAMP", "AUTORECOVERY_GET", "move", ""):
    check("%r is unbuildable" % bad, raises(lambda: bc.build_request(bad), bc.RequestRejected))
check("MOVE at the hard cap builds",
      bc.build_request("MOVE", vx=0.3, vyaw=-1.0)[2] == json.dumps({"x": 0.3, "y": 0.0, "z": -1.0}))
check("MOVE over the vx hard cap RAISES (never clips)",
      raises(lambda: bc.build_request("MOVE", vx=0.31), bc.RequestRejected))
check("MOVE over the vyaw hard cap RAISES",
      raises(lambda: bc.build_request("MOVE", vyaw=-1.01), bc.RequestRejected))
check("MOVE with NaN raises", raises(lambda: bc.build_request("MOVE", vx=float("nan")), bc.RequestRejected))
check("STOPMOVE with a velocity raises", raises(lambda: bc.build_request("STOPMOVE", vx=0.1), bc.RequestRejected))
check("MOVE always carries y = 0 (no strafing)",
      json.loads(bc.build_request("MOVE", vx=0.1)[2])["y"] == 0.0)

# --------------------------------------------------------------- 2 params ---
print("2    params")
check("defaults validate", bc.validate_params() == bc.DEFAULTS)
check("lowering vx_max is allowed", bc.validate_params({"vx_max": 0.1})["vx_max"] == 0.1)
check("raising vx_max above the hard cap REFUSES",
      raises(lambda: bc.validate_params({"vx_max": 0.31}), bc.ConfigError))
check("raising vyaw_max above the hard cap REFUSES",
      raises(lambda: bc.validate_params({"vyaw_max": 1.5}), bc.ConfigError))
check("unknown parameter refuses (a typo must not silently fall back to a default)",
      raises(lambda: bc.validate_params({"vx_mx": 0.1}), bc.ConfigError))
check("zero / negative / NaN refuse",
      all(raises(lambda v=v: bc.validate_params({"cmd_timeout": v}), bc.ConfigError)
          for v in (0, -0.4, float("nan"))))
check("arm limit at or above abort limit refuses",
      raises(lambda: bc.validate_params({"hip_arm_max_c": 50}), bc.ConfigError))


# --------------------------------------------------------- 3 state machine ---
class Sim(object):
    """Drives a BridgeCore at the node's real cadence: telemetry at 50 Hz, tick at
    10 Hz. Records every request with its time."""

    def __init__(self, params=None, error_code=100, hips=(30, 30)):
        self.c = bc.BridgeCore(params)
        self.t = 1000.0
        self.error_code, self.hips = error_code, hips
        self.pos = [0.0, 0.0]
        self.v = (0.0, 0.0, 0.0)          # vx, vy, yaw_speed as the robot reports it
        self.telemetry = True
        self.sent = []
        self.next_tick = self.t

    def run(self, secs, twist=None, twist_hz=10.0):
        """Advance secs. twist = (vx, wz) streamed at twist_hz, or None."""
        end = self.t + secs
        next_twist = self.t
        while self.t < end - 1e-9:
            if twist is not None and self.t >= next_twist - 1e-9:
                self.c.on_twist(self.t, twist[0], 0, 0, 0, 0, twist[1])
                next_twist += 1.0 / twist_hz
            if self.telemetry:
                self.c.on_sport(self.t, self.error_code, self.pos[0], self.pos[1], *self.v)
                self.c.on_low(self.t, *self.hips)
            if self.t >= self.next_tick - 1e-9:
                for r in self.c.tick(self.t):
                    self.sent.append((round(self.t, 3), r[0], r[2]))
                self.next_tick += 0.1
            self.t += 0.02

    def names(self):
        return [n for _, n, _ in self.sent]


print("3    state machine")
s = bc.BridgeCore()
check("fresh core: WAITING, sends nothing", s.tick(1.0) == [] and s.state == "WAITING")

sim = Sim(error_code=1002)
sim.run(2.0, twist=(0.3, 0.0))
check("stand-lock (1002), fresh command -> WAITING, NOTHING sent",
      sim.sent == [] and sim.c.state == "WAITING" and "1002" in sim.c.reason, sim.c.status())

sim = Sim()
sim.run(2.0)
check("balance stand, no command -> IDLE, nothing sent", sim.sent == [] and sim.c.state == "IDLE")

sim = Sim()
sim.run(1.0)
sim.run(2.5, twist=(0.3, 0.0))
moves = [x for x in sim.sent if x[1] == "MOVE"]
check("walk_03 profile: 0.3 m/s for 2.5 s -> ~25 Moves at 10 Hz",
      24 <= len(moves) <= 26 and sim.c.state == "DRIVING", len(moves))
check("every Move carries the commanded velocity",
      all(json.loads(p) == {"x": 0.3, "y": 0.0, "z": 0.0} for _, _, p in moves))
t_last_twist = sim.t - 0.1
sim.run(2.0)
stops = [x for x in sim.sent if x[1] == "STOPMOVE"]
check("input stops -> EXACTLY ONE StopMove", len(stops) == 1, sim.sent[-3:])
dt = stops[0][0] - t_last_twist if stops else None
check("...sent 0.4-0.5 s after the last Twist (cmd_timeout 0.4, 10 Hz tick)",
      dt is not None and 0.4 <= dt <= 0.5 + 1e-6, dt)
check("...and no Move after it", not any(x[1] == "MOVE" and x[0] > stops[0][0] for x in sim.sent))
check("back to IDLE", sim.c.state == "IDLE")

sim = Sim()
sim.run(0.5, twist=(0.3, 0.0))
sim.run(1.0, twist=(0.0, 0.0))
check("zero Twist while DRIVING -> one StopMove, then silence (D3)",
      sim.names().count("STOPMOVE") == 1 and sim.names()[-1] == "STOPMOVE", sim.names()[-4:])
sim = Sim()
sim.run(2.0, twist=(0.0, 0.0))
check("zero Twist while IDLE -> nothing at all", sim.sent == [])

sim = Sim()
sim.run(0.5, twist=(0.9, -3.0))
p = json.loads(sim.sent[-1][2])
check("over-limit Twist is CLIPPED to the params, and counted",
      p == {"x": 0.3, "y": 0.0, "z": -1.0} and sim.c.counts["clipped"] > 0, (p, sim.c.counts))
sim = Sim(params={"vx_max": 0.1})
sim.run(0.5, twist=(0.3, 0.0))
check("a lowered YAML clamp is honoured", json.loads(sim.sent[-1][2])["x"] == 0.1)

c = bc.BridgeCore()
c.on_twist(1.0, 0.2, 0.5, 0.1, 0.3, 0.4, 0.0)
check("vy / lz / ax / ay are zeroed and counted", c.cmd == (0.2, 0.0) and c.counts["zeroed_axes"] == 1)
c.on_twist(1.1, float("nan"), 0, 0, 0, 0, 0)
check("a non-finite Twist is DROPPED whole; the previous command stands",
      c.cmd == (0.2, 0.0) and c.cmd_t == 1.0 and c.counts["dropped_nonfinite"] == 1)
c.on_twist(1.2, 0.1, 0, 0, 0, 0, float("inf"))
check("inf in angular.z drops too", c.counts["dropped_nonfinite"] == 2)

# handset
sim = Sim()
sim.run(0.5, twist=(0.3, 0.0))
n0 = len(sim.sent)
sim.c.on_handset(sim.t, 0, 0, 0, 0, 288)                 # L2+A, as measured in s12
sim.run(2.0, twist=(0.3, 0.0))
check("handset key while DRIVING -> YIELDED, ZERO requests after it (not even StopMove)",
      sim.c.state == "YIELDED" and len(sim.sent) == n0, sim.sent[n0:])
sim = Sim()
sim.c.on_handset(sim.t, 0, 0.15, 0, 0, 0)
sim.run(0.3)
check("stick inside 0.2 is not activity", sim.c.state == "IDLE")
sim.c.on_handset(sim.t, 0, 0.25, 0, 0, 0)
sim.run(0.3)
check("stick past 0.2 is activity -> YIELDED", sim.c.state == "YIELDED")

# thermal
sim = Sim()
sim.run(0.5, twist=(0.3, 0.0))
sim.hips = (50, 41)
sim.run(3.0, twist=(0.3, 0.0))
after = [x for x in sim.sent if x[1] != "MOVE"]
check("rear hip 50 C while engaged -> StopMove, then StandDown ~1 s later, then silence",
      [n for _, n, _ in after] == ["STOPMOVE", "STANDDOWN"]
      and 0.95 <= after[1][0] - after[0][0] <= 1.15 and sim.c.state == "ABORTED",
      (after, sim.c.status()))
check("...and no Move after the abort",
      not any(x[1] == "MOVE" and x[0] >= after[0][0] for x in sim.sent))
sim = Sim()
sim.run(0.5, twist=(0.3, 0.0))
sim.hips = (50, 41)
sim.run(0.15, twist=(0.3, 0.0))
sim.c.on_handset(sim.t, 0, 0, 0, 0, 1)
sim.run(2.0)
check("handset during the abort's StandDown delay -> StandDown is NOT sent",
      "STANDDOWN" not in sim.names() and sim.c.state == "YIELDED", sim.names()[-3:])

sim = Sim(hips=(46, 30))
sim.run(1.0, twist=(0.3, 0.0))
check("46 C before engaging -> WAITING on the arm limit, nothing sent",
      sim.sent == [] and "arm limit" in sim.c.reason, sim.c.reason)
sim = Sim()
sim.run(0.5, twist=(0.3, 0.0))
sim.hips = (46, 30)
sim.run(0.5, twist=(0.3, 0.0))
check("46 C AFTER engaging -> keeps driving (only 50 aborts)", sim.c.state == "DRIVING")
sim = Sim(hips=(51, 30))
sim.run(1.0, twist=(0.3, 0.0))
check("51 C before ever engaging -> ABORTED, but SILENT (posture is the operator's)",
      sim.sent == [] and sim.c.state == "ABORTED", sim.c.status())

# speed guards (session 12's transient)
sim = Sim()
sim.run(0.5, twist=(0.3, 0.0))
sim.v = (0.637, 0.0, 0.0)
sim.run(0.02, twist=(0.3, 0.0))
sim.v = (0.2, 0.0, 0.0)
sim.run(1.0, twist=(0.3, 0.0))
check("ONE 0.637 m/s sample (the s12 transient) does NOT abort", sim.c.state == "DRIVING")
sim.v = (0.7, 0.0, 0.0)
sim.run(0.14, twist=(0.3, 0.0))
sim.v = (0.2, 0.0, 0.0)                     # back to normal: the trip must stay LATCHED
sim.run(0.2, twist=(0.3, 0.0))              # ...and act on the next 10 Hz tick
check("0.7 m/s held 0.14 s DOES abort (latched, acted on at the next tick)", sim.c.state == "ABORTED" and "speed" in sim.c.reason,
      sim.c.status())
sim = Sim()
sim.run(0.5, twist=(0.0, 1.0))
sim.v = (0.0, 0.0, -2.2)
sim.run(0.2, twist=(0.0, 1.0))
sim.v = (0.0, 0.0, 0.8)
sim.run(0.2, twist=(0.0, 1.0))
check("|yaw_speed| 2.2 sustained aborts (abs, 2x the 1.0 clamp)", "yaw" in sim.c.reason, sim.c.reason)

# geofence
sim = Sim()
sim.pos = [5.0, -3.0]                       # odometry persists across stands (s12): origin is not 0
sim.run(0.5, twist=(0.3, 0.0))
sim.pos = [5.0 + 0.7, -3.0 + 0.7]           # 0.99 m
sim.run(0.3, twist=(0.3, 0.0))
check("0.99 m from the engage point -> still driving", sim.c.state == "DRIVING")
sim.pos = [5.0 + 0.72, -3.0 + 0.72]         # 1.018 m
sim.run(0.3, twist=(0.3, 0.0))
check("1.02 m -> ABORTED on the geofence, StopMove sent",
      sim.c.state == "ABORTED" and "geofence" in sim.c.reason and "STOPMOVE" in sim.names())

# staleness
sim = Sim()
sim.run(0.5, twist=(0.3, 0.0))
sim.telemetry = False
sim.run(1.0, twist=(0.3, 0.0))
check("/sportmodestate silent after engaging -> ABORTED, StopMove sent",
      sim.c.state == "ABORTED" and "silent" in sim.c.reason and "STOPMOVE" in sim.names(),
      sim.c.status())
sim = Sim()
sim.telemetry = False
sim.run(1.0, twist=(0.3, 0.0))
check("no telemetry before engaging -> WAITING, not ABORTED", sim.c.state == "WAITING")

# leaving balance stand mid-walk
sim = Sim()
sim.run(0.5, twist=(0.3, 0.0))
n0 = len(sim.sent)
sim.error_code = 2006
sim.run(1.0, twist=(0.3, 0.0))
check("error_code leaves 100 mid-walk -> WAITING, NOTHING sent (not even StopMove)",
      sim.c.state == "WAITING" and len(sim.sent) == n0, sim.sent[n0:])

# latches survive
sim = Sim()
sim.run(0.5, twist=(0.3, 0.0))
sim.c.on_handset(sim.t, 0, 0, 0, 0, 1)
sim.run(0.2)
sim.c.on_handset(sim.t, 0, 0, 0, 0, 0)
n0 = len(sim.sent)
sim.run(3.0, twist=(0.3, 0.0))
check("YIELDED survives the handset going idle and fresh Twists", sim.c.state == "YIELDED" and len(sim.sent) == n0)


# --------------------------------------------------------- 4 package audit ---
ROBOT_TOPIC = "/api/sport/request"
READ_TOPICS = {"/api/sport/response", "/sportmodestate", "/lowstate", "/wirelesscontroller"}
FORBIDDEN = ["/lowcmd", "/arm_Command", "/utlidar/switch", "bashrunner", "programming_actuator",
             "motion_switcher", "sport_lease"]


def audit(sources):
    """sources: {name: text}. Returns (problems, publishers_found)."""
    probs, pubs = [], []
    for name, text in sources.items():
        for bad in FORBIDDEN:
            if bad in text:
                probs.append("%s: forbidden substring %r" % (name, bad))
        # every /api/ occurrence must be one of the two sport topics
        i = text.find("/api/")
        while i >= 0:
            if not (text.startswith(ROBOT_TOPIC, i) or text.startswith("/api/sport/response", i)):
                probs.append("%s: /api/ outside the sport request/response topics: %r"
                             % (name, text[i:i + 40]))
            i = text.find("/api/", i + 1)
        tree = ast.parse(text)
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "create_publisher":
                if len(node.args) < 2:
                    probs.append("%s: create_publisher without a positional topic" % name)
                    continue
                topic = node.args[1]
                if not (isinstance(topic, ast.Constant) and isinstance(topic.value, str)):
                    probs.append("%s: create_publisher topic is not a string literal" % name)
                    continue
                pubs.append(topic.value)
                if topic.value != ROBOT_TOPIC and not topic.value.startswith("/go2/"):
                    probs.append("%s: publisher %r outside /go2/" % (name, topic.value))
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                mod = node.module if isinstance(node, ast.ImportFrom) else node.names[0].name
                if mod and mod.startswith("unitree_api") and "node" not in name:
                    probs.append("%s: unitree_api imported outside the node" % name)
    if pubs.count(ROBOT_TOPIC) != 1:
        probs.append("expected exactly ONE publisher on %s, found %d" % (ROBOT_TOPIC, pubs.count(ROBOT_TOPIC)))
    return probs, pubs


print("4    package audit")
srcs = {}
for root, _, files in os.walk(PKG):
    for f in files:
        if f.endswith(".py") and "__pycache__" not in root:
            with open(os.path.join(root, f)) as fh:
                srcs[os.path.relpath(os.path.join(root, f), PKG)] = fh.read()
print("     scanned: " + ", ".join(sorted(srcs)))
check("scanned at least the core and the node (anti-vacuity: an empty package is not a safe one)",
      any("bridge_core" in k for k in srcs) and any("bridge_node" in k for k in srcs), sorted(srcs))
probs, pubs = audit(srcs)
print("     publishers: %s" % pubs)
check("package audit clean", probs == [], probs)

# ------------------------------------------------------ 5 anti-vacuity ---
print("5    anti-vacuity: the auditor must reject bad sources")
GOOD_NODE = "self.pub = self.create_publisher(Request, '/api/sport/request', 10)\n"
check("auditor passes a minimal good node", audit({"bridge_node.py": GOOD_NODE})[0] == [])
bad_cases = {
    "two robot publishers": GOOD_NODE * 2,
    "no robot publisher": "x = 1\n",
    "topic from a variable": "T = '/api/sport/request'\nself.create_publisher(Request, T, 10)\n",
    "a publisher outside /go2/": GOOD_NODE + "self.create_publisher(Twist, '/cmd_vel', 10)\n",
    "another /api/ topic": GOOD_NODE + "# see /api/bashrunner/request\n",
    "/lowcmd mentioned": GOOD_NODE + "s = '/lowcmd'\n",
}
for label, src in bad_cases.items():
    check("auditor REJECTS: " + label, audit({"bridge_node.py": src})[0] != [])
check("auditor REJECTS unitree_api imported in the core",
      audit({"bridge_node.py": GOOD_NODE, "bridge_core.py": "from unitree_api.msg import Request\n"})[0] != [])

# ------------------------------------------------- 6 clean interpreter ---
print("6    clean interpreter")
probe = ("import sys; sys.path.insert(0, %r)\n"
         "from go2_cmd_bridge import bridge_core\n"
         "assert 'rclpy' not in sys.modules\n"
         "print('CLEAN')\n" % PKG)
p = subprocess.run([sys.executable, "-E", "-s", "-c", probe], capture_output=True, text=True)
check("bridge_core imports without rclpy", p.returncode == 0 and "CLEAN" in p.stdout, p.stderr[-300:])

print()
print("%d checks, %d failure(s)" % (n_checks, len(fails)))
if n_checks < 60:
    print("FAIL: fewer checks ran than written -- refusing to report success")
    sys.exit(1)
print("ALL CMD BRIDGE OFFLINE TESTS PASS" if not fails else "FAILURES:\n  - " + "\n  - ".join(fails))
sys.exit(1 if fails else 0)
