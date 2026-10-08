#!/usr/bin/env python3
"""Offline tests for cmd_vel_profile.py -- NO ROS, NO container, NO robot.

  1  every baked profile validates; the validator REJECTS bad ones
  2  AST: exactly one publisher, the literal /go2/cmd_vel; no robot command path
  3  analyse_bridge() on synthetic runs whose answer is known by construction
     (PV3 watchdog stop, PV4 dead-bridge stop -- both PASS and FAIL cases)
  4  CLI: refuses without --armed/--dry; --plan and --replay never import rclpy
"""
import ast
import csv
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cmd_vel_profile as cp          # noqa: E402

n_checks = 0
fails = []


def check(label, ok, detail=""):
    global n_checks
    n_checks += 1
    print(("  ok   " if ok else "  FAIL ") + label + ("" if ok else "  -- " + str(detail)[:400]))
    if not ok:
        fails.append(label)


def rejects(steps, name="bad"):
    cp.PROFILES[name] = steps
    try:
        cp.validate_profile(name)
        return False
    except cp.ProfileError:
        return True
    finally:
        del cp.PROFILES[name]


# ------------------------------------------------------------- 1 profiles ---
print("1    profiles")
for name in sorted(cp.PROFILES):
    try:
        cp.validate_profile(name)
        ok = True
    except cp.ProfileError as e:
        ok, err = False, e
    check("baked profile %r validates" % name, ok, "" if ok else err)
check("walk_03 is the s12 command: 0.3 m/s x 2.5 s", ("stream", 0.3, 0.0, 2.5) in cp.PROFILES["walk_03"])
check("listen streams nothing", not any(s[0] == "stream" for s in cp.PROFILES["listen"]))
check("only walk_03_kill kills", [n for n, st in cp.PROFILES.items() if ("kill_bridge",) in st] == ["walk_03_kill"])
check("walk_03_kill stops streaming before the kill (no stream after it)",
      all(s[0] != "stream" for s in cp.PROFILES["walk_03_kill"][cp.PROFILES["walk_03_kill"].index(("kill_bridge",)):]))
W = ("wait", 3.0)
check("REJECTS vx over the cap", rejects([("stream", 0.31, 0.0, 1.0), W]))
check("REJECTS vyaw over the cap", rejects([("stream", 0.0, -1.2, 1.0), W]))
check("REJECTS NaN", rejects([("stream", float("nan"), 0.0, 1.0), W]))
check("REJECTS a 7 s stream", rejects([("stream", 0.1, 0.0, 7.0), W]))
check("REJECTS kill_bridge outside a *_kill profile", rejects([("kill_bridge",), W]))
check("REJECTS a profile that ends on a stream (no settle recorded)", rejects([("stream", 0.1, 0.0, 1.0)]))
check("REJECTS a profile that ends on a 1 s wait", rejects([("stream", 0.1, 0.0, 1.0), ("wait", 1.0)]))
check("REJECTS 0.3 x 3.0 s: 0.9 m + 0.12 m watchdog tail > 0.95 m",
      rejects([("stream", 0.3, 0.0, 3.0), W]))
check("accepts 0.3 x 2.5 s: 0.75 + 0.12 = 0.87 m", not rejects([("stream", 0.3, 0.0, 2.5), W]))
check("REJECTS an unknown step", rejects([("fly", 1.0), W]))

# ------------------------------------------------------------------ 2 AST ---
print("2    AST audit")
src = open(os.path.join(HERE, "cmd_vel_profile.py")).read()
tree = ast.parse(src)
pubs = []
for node in ast.walk(tree):
    if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "create_publisher":
        a = node.args[1] if len(node.args) > 1 else None
        pubs.append(a.value if isinstance(a, ast.Constant) else "<non-literal>")
check("exactly one create_publisher, the literal /go2/cmd_vel", pubs == ["/go2/cmd_vel"], pubs)
imports = set()
for node in ast.walk(tree):
    if isinstance(node, ast.ImportFrom) and node.module == "unitree_api.msg":
        imports.update(a.name for a in node.names)
check("from unitree_api it imports only Response (no Request type to build one with)",
      imports == {"Response"}, imports)
api = [src[i:i + 30] for i in range(len(src)) if src.startswith("/api/", i)]
check("the only /api/ topic named is /api/sport/response (read)",
      api and all(a.startswith("/api/sport/response") for a in api), api)
for bad in ("/lowcmd", "/api/sport/request", "/arm_Command"):
    check("does not name %s" % bad, bad not in src)


# ------------------------------------------------------------- 3 analysis ---
def synth(run_dir, after_cm, settle_s, kill=False):
    """A walk, timed like the REAL bridge in integration test D: Twists at 0.0-2.4 s,
    Moves at 10 Hz offset 0.05 s, the last 4 of them the watchdog tail (2.45-2.75),
    StopMove at 2.85 s = 0.45 s after the last Twist. Kill case: SIGKILL at 1.5 s, no StopMove. After the last request the
    body travels `after_cm` more, linearly, over `settle_s`."""
    os.makedirs(run_dir)
    t0 = 1000.0
    ev = [["t", "utc", "kind", "api_id", "code", "detail"]]
    sp = [["t", "mode", "gait_type", "error_code", "body_height", "px", "py", "pz",
           "vx", "vy", "vz", "yaw_speed", "yaw"]]
    stream_end = 1.5 if kill else 2.5
    for k in range(int(stream_end * 10)):
        ev.append([t0 + k / 10.0, "", "twist", "", "", "0.300 0.000"])
    t_last = None
    for k in range(int(stream_end * 10) + (0 if kill else 3)):
        t = t0 + 0.05 + k / 10.0
        ev.append([t, "", "sent", 1008, "", '{"x": 0.3, "y": 0.0, "z": 0.0}'])
        t_last = t
    if kill:
        ev.append([t0 + 1.5, "", "kill_bridge", "", "", "go2_cmd_bridge_node"])
    else:
        t_last = t0 + 2.85
        ev.append([t_last, "", "sent", 1003, "", "STOPMOVE"])
    ev.sort(key=lambda r: r[0] if isinstance(r[0], float) else 0)
    px = 0.0
    for k in range(int(10 * 50)):                      # 10 s at 50 Hz, yaw 0.5 rad: body frame matters
        t = t0 + k / 50.0
        if t <= t_last:
            px = 0.2 * (t - t0)
        elif t <= t_last + settle_s:
            px = 0.2 * (t_last - t0) + (after_cm / 100.0) * (t - t_last) / settle_s
        import math
        sp.append([t, 0, 0, 100, 0.3, px * math.cos(0.5), px * math.sin(0.5), 0, 0.2, 0, 0, 0, 0.5])
    for name, rows in (("events.csv", ev), ("sport.csv", sp)):
        with open(os.path.join(run_dir, name), "w", newline="") as fh:
            csv.writer(fh).writerows(rows)


print("3    analyse_bridge on synthetic runs")
with tempfile.TemporaryDirectory() as td:
    d = os.path.join(td, "wd_ok"); synth(d, after_cm=1.5, settle_s=0.4)
    r = cp.analyse_bridge(d)
    pv3 = [l for l in r.splitlines() if "travel after the StopMove" in l]
    check("watchdog walk, 1.5 cm after StopMove -> PV3 PASS", pv3 and "PASS" in pv3[0], r[-700:])
    check("...the StopMove timing is reported", "PV3 -- watchdog StopMove 0.4" in r, r[-700:])
    check("...the watchdog tail is counted (4)", "watchdog tail: 4 Move(s)" in r, r[-700:])
    import re
    m = re.search(r"fwd \+([0-9.]+), left ([+-][0-9.]+)", pv3[0]) if pv3 else None
    check("...travel is in the BODY frame (yaw 0.5 rad: all forward, ~0 left)",
          m is not None and 1.3 <= float(m.group(1)) <= 1.5 and abs(float(m.group(2))) <= 0.05, pv3)
    d = os.path.join(td, "wd_bad"); synth(d, after_cm=6.0, settle_s=0.8)
    r = cp.analyse_bridge(d)
    pv3 = [l for l in r.splitlines() if "travel after the StopMove" in l]
    check("watchdog walk, 6 cm after StopMove -> PV3 FAIL", pv3 and "FAIL" in pv3[0], pv3)

    d = os.path.join(td, "kill_ok"); synth(d, after_cm=10.0, settle_s=1.0, kill=True)
    r = cp.analyse_bridge(d)
    pv4 = [l for l in r.splitlines() if l.startswith("PV4")]
    check("kill, 10 cm over 1.0 s -> PV4 PASS", pv4 and "PASS" in pv4[0], r[-700:])
    check("...requests after the kill reported as 0", "requests after the kill: 0" in r, r[-700:])
    check("...no PV3 line in a kill run (no StopMove to time)", "PV3" not in r)
    d = os.path.join(td, "kill_far"); synth(d, after_cm=40.0, settle_s=1.0, kill=True)
    pv4 = [l for l in cp.analyse_bridge(d).splitlines() if l.startswith("PV4")]
    check("kill, 40 cm -> PV4 FAIL", pv4 and "FAIL" in pv4[0], pv4)
    d = os.path.join(td, "kill_slow"); synth(d, after_cm=10.0, settle_s=2.5, kill=True)
    pv4 = [l for l in cp.analyse_bridge(d).splitlines() if l.startswith("PV4")]
    check("kill, 10 cm but still creeping at 2.5 s -> PV4 FAIL (time bound)", pv4 and "FAIL" in pv4[0], pv4)

    # two segments with a pause: the rate must be per segment, not across the pause
    d = os.path.join(td, "two_seg"); os.makedirs(d)
    rows = [["t", "utc", "kind", "api_id", "code", "detail"]]
    for k in range(20):
        rows.append([2000.0 + k / 10.0, "", "sent", 1008, "", '{"x": 0.0, "y": 0.0, "z": 1.0}'])
    for k in range(20):
        rows.append([2004.5 + k / 10.0, "", "sent", 1008, "", '{"x": 0.0, "y": 0.0, "z": -1.0}'])
    with open(os.path.join(d, "events.csv"), "w", newline="") as fh:
        csv.writer(fh).writerows(rows)
    rates = [l for l in cp.analyse_bridge(d).splitlines() if l.startswith("Move rate")]
    check("Move rate reported per segment (two at 10.00 Hz), not across the pause (s13 stage 4)",
          len(rates) == 2 and all("10.00 Hz" in l for l in rates), rates)

    d = os.path.join(td, "empty"); os.makedirs(d)
    check("empty run dir does not crash", "Twists published: 0" in cp.analyse_bridge(d))

    # ------------------------------------------------------------ 4 CLI ---
    print("4    CLI and clean interpreter")
    p = subprocess.run([sys.executable, os.path.join(HERE, "cmd_vel_profile.py"), "walk_03"],
                       capture_output=True, text=True)
    check("refuses without --armed/--dry", p.returncode != 0 and "--armed or --dry" in p.stderr, p.stderr[-200:])
    probe_src = ("import sys; sys.path.insert(0, %r)\n"
                 "import cmd_vel_profile as cp\n"
                 "rc1 = cp.main(['walk_03_kill', '--plan'])\n"
                 "rc2 = cp.main(['--replay', %r])\n"
                 "assert 'rclpy' not in sys.modules, 'rclpy imported'\n"
                 "print('CLEAN', rc1, rc2)\n" % (HERE, os.path.join(td, "kill_ok")))
    p = subprocess.run([sys.executable, "-E", "-s", "-c", probe_src], capture_output=True, text=True, cwd=td)
    check("--plan and --replay in a clean interpreter, no rclpy",
          p.returncode == 0 and "CLEAN 0 0" in p.stdout and "SIGKILL" in p.stdout, p.stdout[-300:] + p.stderr[-300:])

print()
print("%d checks, %d failure(s)" % (n_checks, len(fails)))
if n_checks < 35:
    print("FAIL: fewer checks ran than written")
    sys.exit(1)
print("ALL CMD_VEL PROFILE OFFLINE TESTS PASS" if not fails else "FAILURES:\n  - " + "\n  - ".join(fails))
sys.exit(1 if fails else 0)
