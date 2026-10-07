#!/usr/bin/env python3
"""Offline tests for go2_sport_probe -- NO ROS, NO container, NO robot.

Run BEFORE the Go2 is powered. go2_sport_probe.py is the first file in this
project that publishes to the robot, so the safety properties are checked
mechanically, not by reading:

  1-3  AST: exactly one create_publisher, its topic the literal /api/sport/request,
       and no other command topic named anywhere in the file
  4    anti-vacuity: the same AST checker FAILS a deliberately bad source
  5-7  allowlist and clamps: Damp and every acrobatic api are unbuildable; Move
       outside the clamp raises rather than clips
  8-9  every baked plan validates, and every motion plan ends lying, with each
       Move followed by StopMove
  10   pure helpers: posture, handset, environment
  11   analyse() on synthetic runs whose right answer is known by construction
  12   dry run and --replay in a CLEAN interpreter, asserting rclpy was never
       imported (session 7's lesson: a test that stubs rclpy cannot see this)
"""
import ast
import csv
import json
import math
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
SRC_PATH = os.path.join(HERE, "go2_sport_probe.py")
SRC = open(SRC_PATH, encoding="utf-8").read()

import go2_sport_probe as g                                   # noqa: E402

fails = []
n_checks = 0


def check(name, cond, detail=""):
    global n_checks
    n_checks += 1
    print(f"  {'ok  ' if cond else 'FAIL'} {name}" + (f"  -- {detail}" if detail and not cond else ""))
    if not cond:
        fails.append(name)


def raises(fn, *a, **k):
    try:
        fn(*a, **k)
    except g.RequestRejected:
        return True
    return False


# ---------------------------------------------------------------- 1-4 AST ---
ALLOWED_API_STRINGS = {"/api/sport/request", "/api/sport/response"}
FORBIDDEN = ["/lowcmd", "lowcmd", "programming_actuator", "bashrunner", "sport_lease",
             "motion_switcher", "/arm_Command", "/utlidar/switch", "/webrtcreq", "/gpt_cmd"]


def audit(src):
    """Return (failures, publishers). Pure; applied to the real file AND to a bad one."""
    bad, pubs = [], []
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fn = node.func
            name = fn.attr if isinstance(fn, ast.Attribute) else getattr(fn, "id", None)
            if name == "create_publisher":
                topic = node.args[1] if len(node.args) >= 2 else None
                for kw in node.keywords:
                    if kw.arg == "topic":
                        topic = kw.value
                if isinstance(topic, ast.Constant) and isinstance(topic.value, str):
                    pubs.append(topic.value)
                    if topic.value != "/api/sport/request":
                        bad.append(f"publisher to {topic.value!r}")
                else:
                    bad.append("publisher topic is not a string literal")
            if name in ("create_client", "ActionClient"):
                bad.append(f"{name} call")
    if len(pubs) != 1:
        bad.append(f"{len(pubs)} create_publisher calls, expected exactly 1")
    for f in FORBIDDEN:
        if f in src:
            bad.append(f"forbidden substring {f!r}")
    for m in re.findall(r"/api/[A-Za-z0-9_/]+", src):
        if m not in ALLOWED_API_STRINGS:
            bad.append(f"other /api/ topic named: {m}")
    return bad, pubs


print("1-3  AST audit of go2_sport_probe.py")
bad, pubs = audit(SRC)
check("exactly one publisher, to /api/sport/request, nothing forbidden", not bad, "; ".join(bad))
check("publisher list is non-empty (scan saw something)", pubs == ["/api/sport/request"], str(pubs))
check("Request import is inside the publishing branch only",
      SRC.count("from unitree_api.msg import Request") == 1)

print("4    anti-vacuity: the auditor must reject bad sources")
for label, bad_src in [
    ("lowcmd publisher", 'self.create_publisher(LowCmd, "/lowcmd", 10)\n'),
    ("runtime topic", 't = "/api/sport/request"\nself.create_publisher(Request, t, 10)\n'),
    ("second publisher", 'self.create_publisher(R, "/api/sport/request", 10)\n'
                         'self.create_publisher(R, "/api/sport/request", 10)\n'),
    ("other api", 'self.create_publisher(R, "/api/sport/request", 10)\nx = "/api/vui/request"\n'),
]:
    b, _ = audit(bad_src)
    check(f"auditor rejects: {label}", bool(b))

# ------------------------------------------------------ 5-7 allowlist/clamp ---
print("5-7  allowlist and clamps")
check("allowlist is exactly the six planned ids",
      g.ALLOWED_API_IDS == {2055, 1003, 1004, 1005, 1002, 1008}, str(sorted(g.ALLOWED_API_IDS)))
check("Damp (1001) is not buildable", 1001 not in g.ALLOWED_API_IDS and raises(g.build_request, "DAMP"))
for name in ("FRONTFLIP", "BACKFLIP", "HANDSTAND", "DANCE1", "RECOVERYSTAND", "SWITCHJOYSTICK"):
    check(f"{name} rejected", raises(g.build_request, name))
check("MOVE vx 0.2 accepted (boundary)",
      g.build_request("MOVE", vx=0.2, vy=0.0, vyaw=0.0)[0] == 1008)
check("MOVE parameter uses vendor keys x/y/z",
      json.loads(g.build_request("MOVE", vx=0.1, vy=0.0, vyaw=-0.3)[1]) == {"x": 0.1, "y": 0.0, "z": -0.3})
for label, kw in [("vx 0.21", dict(vx=0.21, vy=0, vyaw=0)), ("vx -0.21", dict(vx=-0.21, vy=0, vyaw=0)),
                  ("vy 0.01", dict(vx=0, vy=0.01, vyaw=0)), ("vyaw 0.31", dict(vx=0, vy=0, vyaw=0.31)),
                  ("vx nan", dict(vx=float("nan"), vy=0, vyaw=0)),
                  ("vx inf", dict(vx=float("inf"), vy=0, vyaw=0)),
                  ("missing vyaw", dict(vx=0.1, vy=0)), ("extra key", dict(vx=0, vy=0, vyaw=0, z=1))]:
    check(f"MOVE {label} raises (never clipped)", raises(g.build_request, "MOVE", **kw))
check("STOPMOVE with a parameter raises", raises(g.build_request, "STOPMOVE", vx=0.1))

# ------------------------------------------------------------ 8-9 plans ---
print("8-9  plans")
for stage, plan in g.PLANS.items():
    try:
        reqs = g.plan_requests(plan)
        check(f"{stage}: every baked request validates ({len(reqs)})", True)
    except g.RequestRejected as e:
        check(f"{stage}: every baked request validates", False, str(e))
check("listen publishes nothing", not g.stage_publishes("listen"))
check("query sends exactly one AUTORECOVERY_GET",
      g.plan_requests(g.PLANS["query"]) == [(2055, "")])
check("move_once sends exactly ONE Move (PM3 needs it)",
      sum(1 for a, _ in g.plan_requests(g.PLANS["move_once"]) if a == 1008) == 1)

for stage, plan in g.PLANS.items():
    names = [s[1] if s[0] == "send" else ("MOVE" if s[0] == "stream" else None) for s in plan]
    names = [n for n in names if n]
    if "STANDUP" in names:
        check(f"{stage}: last command is STANDDOWN", names[-1] == "STANDDOWN", str(names))
        check(f"{stage}: plan ends by waiting until lying",
              ("until", "lying") in plan[-3:], str(plan[-3:]))
    # every Move run is followed by STOPMOVE before anything else motion-related
    for i, n in enumerate(names):
        if n == "MOVE" and (i + 1 == len(names) or names[i + 1] not in ("MOVE", "STOPMOVE")):
            check(f"{stage}: Move at {i} followed by STOPMOVE", False, str(names))
    if "MOVE" in names:
        last_move = max(i for i, n in enumerate(names) if n == "MOVE")
        check(f"{stage}: STOPMOVE follows the last Move", names[last_move + 1] == "STOPMOVE")
    check(f"{stage}: has a posture precondition entry", stage in g.POSTURE_PRE)
check("motion stages all require starting lying",
      all(g.POSTURE_PRE[s] == "lying" for s in g.STANDING_STAGES))

# ------------------------------------------------------------ 10 helpers ---
print("10   helpers")
check("posture lying at 0.0715 (s5 measured)", g.posture_of(0.0715) == "lying")
check("posture standing at 0.30", g.posture_of(0.30) == "standing")
check("posture between at 0.16", g.posture_of(0.16) == "between")
check("posture None when no data", g.posture_of(None) is None)
check("handset idle", not g.handset_active(0.01, -0.02, 0.0, 0.0, 0))
check("handset key = active", g.handset_active(0, 0, 0, 0, 1))
check("handset stick = active", g.handset_active(0, 0.5, 0, 0, 0))
check("env ok", g.check_env({"ROS_DOMAIN_ID": "0", "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}) == [])
check("env: unset domain refused (must be EXPLICIT)",
      g.check_env({"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}) != [])
check("env: domain 42 refused",
      g.check_env({"ROS_DOMAIN_ID": "42", "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}) != [])
check("env: localhost-only refused",
      g.check_env({"ROS_DOMAIN_ID": "0", "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                   "ROS_LOCALHOST_ONLY": "1"}) != [])


# ------------------------------------------------------- 11 analyse() ---
def synth(run_dir, move_profile=None, query=False, q_jitter=0.0):
    """Write a synthetic run. move_profile(t_since_move) -> speed (m/s)."""
    os.makedirs(run_dir, exist_ok=True)
    t0 = 1000.0
    ev = [["t", "utc", "kind", "api_id", "code", "detail"]]
    sp = [["t", "mode", "gait_type", "error_code", "body_height", "px", "py", "pz",
           "vx", "vy", "vz", "yaw_speed", "yaw"]]
    lo = [["t"] + [f"q_{n}" for n in g.JOINT] + [f"temp_{n}" for n in g.JOINT]]
    if query:
        ev.append([t0 + 3, "", "sent", 2055, "", "AUTORECOVERY_GET"])
        ev.append([t0 + 3.02, "", "response", 2055, 0, '{"enable":true}'])
    if move_profile:
        ev.append([t0 + 2, "", "sent", 1004, "", "STANDUP"])
        ev.append([t0 + 6, "", "sent", 1008, "", '{"x": 0.1, "y": 0.0, "z": 0.0}'])
        ev.append([t0 + 8, "", "sent", 1003, "", "STOPMOVE"])
    for k in range(0, 600):                       # 12 s at 50 Hz
        t = t0 + k / 50.0
        v = move_profile(t - (t0 + 6)) if move_profile and t >= t0 + 6 else 0.0
        bh = 0.30 if move_profile and t > t0 + 3 else 0.0715
        sp.append([t, 1, 0, 1001, bh, 0, 0, 0, v, 0, 0, 0, 0])
        lo.append([t] + [0.5 + (q_jitter if k % 2 else 0.0)] * 12 + [26] * 12)
    for name, rows in (("events.csv", ev), ("sport.csv", sp), ("low.csv", lo)):
        with open(os.path.join(run_dir, name), "w", newline="") as fh:
            csv.writer(fh).writerows(rows)


print("11   analyse() on synthetic runs")
with tempfile.TemporaryDirectory() as td:
    d = os.path.join(td, "persist")
    synth(d, move_profile=lambda s: 0.1 if s > 0.3 else 0.0)
    r = g.analyse(d)
    check("Move that never decays -> PERSISTS", "PERSISTS" in r, r)

    d = os.path.join(td, "decays")
    synth(d, move_profile=lambda s: 0.1 if 0.3 < s < 0.9 else 0.0)
    r = g.analyse(d)
    check("Move that decays in 0.6 s -> does NOT persist, PM3 PASS",
          "does NOT persist" in r and "PM3 PASS" in r, r)

    d = os.path.join(td, "ignored")
    synth(d, move_profile=lambda s: 0.0)
    r = g.analyse(d)
    check("Move with no motion -> NO CONCLUSION, not a pass", "PM3 -- NO CONCLUSION" in r, r)

    d = os.path.join(td, "query_still")
    synth(d, query=True)
    r = g.analyse(d)
    check("query, joints still -> PM1 PASS", "PM1" in r and "PASS" in r.split("PM1")[1].split("\n")[0], r)
    check("query response matched with latency", "code=    0" in r and "latency=0.020" in r, r)

    d = os.path.join(td, "query_moved")
    synth(d, query=True, q_jitter=0.01)
    r = g.analyse(d)
    check("query, joints moved 0.01 rad -> PM1 FAIL", "FAIL" in r.split("PM1")[1].split("\n")[0], r)

    d = os.path.join(td, "empty")
    os.makedirs(d)
    r = g.analyse(d)
    check("empty run dir reports zero rows, does not crash", "rows: events 0" in r, r)

    # --------------------------------------- 12 clean interpreter ---
    print("12   clean interpreter: dry run and --replay must never import rclpy")
    probe = (
        "import sys; sys.path.insert(0, %r)\n"
        "import go2_sport_probe as g\n"
        "rc1 = g.main(['move_set'])\n"
        "rc2 = g.main(['--replay', %r])\n"
        "assert 'rclpy' not in sys.modules, 'rclpy was imported'\n"
        "assert 'unitree_api' not in sys.modules\n"
        "print('CLEAN', rc1, rc2)\n" % (HERE, os.path.join(td, "decays")))
    p = subprocess.run([sys.executable, "-E", "-s", "-c", probe],
                       capture_output=True, text=True, cwd=td)
    check("dry run + replay in a clean interpreter, no rclpy",
          p.returncode == 0 and "CLEAN 0 0" in p.stdout and "DRY RUN" in p.stdout,
          (p.stdout[-400:] + p.stderr[-400:]))

print()
print(f"{n_checks} checks, {len(fails)} failure(s)")
if n_checks < 40:
    print("FAIL: fewer checks ran than written -- refusing to report success")
    sys.exit(1)
print("ALL SPORT PROBE OFFLINE TESTS PASS" if not fails else "FAILURES:\n  - " + "\n  - ".join(fails))
sys.exit(1 if fails else 0)
