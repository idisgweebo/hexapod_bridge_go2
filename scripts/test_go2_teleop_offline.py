#!/usr/bin/env python3
"""Offline tests for go2_teleop.py -- no ROS, no Tk, no display, no robot.

Key events are synthesised in the shapes MEASURED on 8 Oct (gate7 §10c): a hold is a
press, then after ~500 ms a release+press PAIR every ~30.3 ms with identical X time,
then a real release. Time is passed in; the tick runs every 10 ms as the window's does.
"""
import ast
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import go2_teleop as gt  # noqa: E402

fails = []
n_checks = 0


def check(label, ok, detail=""):
    global n_checks
    n_checks += 1
    print("  %s %s%s" % ("pass" if ok else "FAIL", label, (" -- " + str(detail)[:300]) if not ok else ""))
    if not ok:
        fails.append(label)


def raises(fn, exc=Exception):
    try:
        fn()
    except exc:
        return True
    return False


class Sim:
    """Run a core on a merged list of (t, kind, key, xtime) events, ticking every 10 ms."""

    def __init__(self, core=None):
        self.core = core or gt.TeleopCore()
        self.out = []               # (t, (vx, vyaw))

    def run(self, events, t_end):
        events = sorted(events)
        i, t = 0, 0.0
        while t <= t_end + 1e-9:
            while i < len(events) and events[i][0] <= t + 1e-9:
                et, kind, key, xt = events[i]
                if kind == "press":
                    self.core.press(key, xt, et)
                elif kind == "release":
                    self.core.release(key, xt, et)
                elif kind == "stop":
                    self.core.stop()
                elif kind == "latch":
                    self.core.stop("bridge YIELDED", latch=True)
                i += 1
            o = self.core.tick(t)
            if o is not None:
                self.out.append((round(t, 4), o))
            t = round(t + 0.01, 4)
        return self.out


def hold(key, t0, dur, delay=0.5, interval=0.0303, xt0=None, autorepeat=True):
    """Measured X11 shape. Returns events; the real release is at t0 + dur."""
    xt = int((xt0 if xt0 is not None else t0) * 1000)
    ev = [(t0, "press", key, xt)]
    t = t0 + delay
    while autorepeat and t + 0.00015 < t0 + dur - 1e-9:     # X never repeats after the release
        x = int(t * 1000)
        ev.append((t, "release", key, x))
        ev.append((t + 0.00015, "press", key, x))
        t += interval
    ev.append((t0 + dur, "release", key, int((t0 + dur) * 1000)))
    return ev


def between(out, a, b):
    return [o for o in out if a <= o[0] < b]


print("1  construction refuses unsafe values")
check("defaults construct", gt.TeleopCore().vx == 0.3)
check("vx above 0.3 refuses", raises(lambda: gt.TeleopCore(vx=0.31), gt.TeleopError))
check("vyaw above 1.0 refuses", raises(lambda: gt.TeleopCore(vyaw=1.5), gt.TeleopError))
check("zero / negative / NaN speed refuses",
      all(raises(lambda v=v: gt.TeleopCore(vx=v), gt.TeleopError) for v in (0, -0.1, float("nan"))))
check("debounce above 100 ms refuses (it adds to stop latency)",
      raises(lambda: gt.TeleopCore(debounce=0.2), gt.TeleopError))
check("module caps equal the bridge's hard caps (0.3, 1.0)", (gt.VX_CAP, gt.VYAW_CAP) == (0.3, 1.0))

print("2  startup: nothing published until a key goes down")
out = Sim().run([], 2.0)
check("no key, 2 s -> zero Twists published", out == [], out[:3])

print("3  a 3 s hold of W with measured auto-repeat")
s = Sim()
out = s.run(hold("w", 1.0, 3.0), 5.0)
moving = [o for o in out if o[1] != (0.0, 0.0)]
check("first Twist at the press (t = 1.00)", moving and moving[0][0] == 1.0, moving[:1])
check("every moving Twist is (0.3, 0)", all(o[1] == (0.3, 0.0) for o in moving))
check("~30 moving Twists over 3 s (10 Hz)", 29 <= len(moving) <= 31, len(moving))
gaps = [b[0] - a[0] for a, b in zip(moving, moving[1:])]
check("no gap over 110 ms during the hold (incl. the 500 ms repeat delay)", max(gaps) <= 0.11 + 1e-9, max(gaps))
check("NO zero during the hold -- 83 synthetic releases never stop it",
      not [o for o in between(out, 1.0, 4.0) if o[1] == (0.0, 0.0)])
check("autorepeat pairs recognised", s.core.counts["autorepeat"] >= 80, s.core.counts)
zeros = [o for o in out if o[1] == (0.0, 0.0)]
check("first zero within debounce + one tick of the real release (<= 4.030)",
      zeros and 4.0 < zeros[0][0] <= 4.0 + gt.DEBOUNCE_S + 0.01 + 1e-9, zeros[:1])
check("exactly ZERO_BURST zeros, then silence", len(zeros) == gt.ZERO_BURST and out[-1][1] == (0.0, 0.0),
      len(zeros))
check("zeros spaced at 10 Hz after the first", all(abs((b[0] - a[0]) - 0.1) < 0.011
                                                      for a, b in zip(zeros, zeros[1:])), zeros)
check("one real release counted", s.core.counts["real_release"] == 1, s.core.counts)

print("4  a release-press pair with DIFFERENT X time inside the debounce: still down (never a stop)")
ev = [(0.0, "press", "w", 0), (1.0, "release", "w", 1000), (1.005, "press", "w", 1005), (2.0, "release", "w", 2000)]
out = Sim().run(ev, 3.0)
check("no zero between 0 and 2 s", not [o for o in between(out, 0.0, 2.0) if o[1] == (0.0, 0.0)])

print("5  taps (measured: held ~110 ms, no auto-repeat)")
ev = []
for i in range(5):
    ev += hold("w", 1.0 + i, 0.11, autorepeat=False)
out = Sim().run(ev, 7.0)
moving = [o for o in out if o[1] != (0.0, 0.0)]
check("each tap moves: 5 bursts of 2 Twists", len(moving) == 10, len(moving))
check("each tap ends in its own zero burst", len([o for o in out if o[1] == (0.0, 0.0)]) == 5 * gt.ZERO_BURST)

print("6  two keys: hold W, add A (W's repeats stop, NO release), let A go, keep W")
ev = hold("w", 1.0, 4.0, autorepeat=False)              # W: press at 1.0, release at 5.0
ev += [(1.0 + 0.5 + 0.0303 * k, "release", "w", 1500 + k) for k in range(10)]
ev += [(1.0 + 0.5 + 0.0303 * k + 0.00015, "press", "w", 1500 + k) for k in range(10)]   # W repeats ~0.3 s
ev += hold("a", 2.0, 1.5)                                # A: 2.0-3.5, now A repeats, W silent
out = Sim().run(ev, 6.0)
check("W alone: (0.3, 0)", all(o[1] == (0.3, 0.0) for o in between(out, 1.0, 2.0)))
check("W + A: (0.3, 1.0) from the A press", between(out, 2.0, 2.01) and between(out, 2.0, 2.01)[0][1] == (0.3, 1.0),
      between(out, 2.0, 2.05))
check("after A's release W still drives with no W event at all: (0.3, 0)",
      between(out, 3.6, 4.9) and all(o[1] == (0.3, 0.0) for o in between(out, 3.6, 4.9)), between(out, 3.5, 3.7))
check("no zero until W's release at 5.0", not [o for o in between(out, 1.0, 5.0) if o[1] == (0.0, 0.0)])

print("7  opposite keys cancel")
out = Sim().run(hold("w", 1.0, 2.0, autorepeat=False) + hold("s", 1.5, 0.5, autorepeat=False), 4.0)
check("W + S -> zero while both down", any(o[1] == (0.0, 0.0) for o in between(out, 1.5, 1.6)), between(out, 1.4, 1.7))
check("...and W resumes after S lets go", any(o[1] == (0.3, 0.0) for o in between(out, 2.0, 2.2)))

print("8  stop: Space / focus loss mid-hold")
out = Sim().run(hold("w", 1.0, 3.0) + [(2.0, "stop", "", 0)], 5.0)
check("zero at the stop, same tick (t = 2.00)", (2.0, (0.0, 0.0)) in out, between(out, 1.9, 2.1))
check("no motion after the stop even though W is physically held and repeating",
      not [o for o in out if o[0] > 2.0 and o[1] != (0.0, 0.0)])
# W held through the stop, then a NEW press is needed: auto-repeat pairs alone must not restart it
check("auto-repeat after a stop does not restart motion (pair press arrives with no pending release)",
      not [o for o in out if o[0] > 2.0 and o[1] != (0.0, 0.0)])

# ...but a genuine new press after the stop does move it (a release, then a fresh press)
out = Sim().run(hold("w", 1.0, 2.0) + [(1.5, "stop", "", 0)] + hold("w", 3.5, 1.0, autorepeat=False), 5.0)
check("a real new press after a stop drives again", any(o[1] == (0.3, 0.0) for o in between(out, 3.5, 4.5)))
check("held-through-stop has no motion between the stop and the new press",
      not [o for o in between(out, 1.51, 3.5) if o[1] != (0.0, 0.0)])

print("9  bridge latch (YIELDED / ABORTED)")
s = Sim()
out = s.run(hold("w", 1.0, 3.0) + [(2.0, "latch", "", 0)] + hold("w", 4.5, 1.0, autorepeat=False), 6.0)
check("zero at the latch", (2.0, (0.0, 0.0)) in out)
check("a fresh press after the latch moves NOTHING", not [o for o in out if o[0] > 2.0 and o[1] != (0.0, 0.0)])
check("latched reason kept", s.core.latched == "bridge YIELDED")

print("10 unmapped keys do nothing")
out = Sim().run([(1.0, "press", "x", 1), (1.5, "release", "x", 2), (2.0, "press", "Up", 3)], 3.0)
check("no Twist for x / Up", out == [], out)

print("11 static audit of go2_teleop.py")
src = open(os.path.join(HERE, "go2_teleop.py")).read()
tree = ast.parse(src)
pubs = []
for node in ast.walk(tree):
    if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "create_publisher":
        a = node.args[1] if len(node.args) > 1 else None
        pubs.append(a.value if isinstance(a, ast.Constant) else "<non-literal>")
check("exactly one publisher, on the literal /go2/cmd_vel", pubs == ["/go2/cmd_vel"], pubs)
for bad in ("/api/", "/lowcmd", "unitree_api", "bashrunner", "programming_actuator", "/arm_Command"):
    check("no %r anywhere in the file" % bad, bad not in src)

print("12 clean interpreter: the core imports without rclpy or tkinter")
probe = ("import sys; sys.path.insert(0, %r)\n"
         "import go2_teleop\n"
         "assert 'rclpy' not in sys.modules and 'tkinter' not in sys.modules\n"
         "print('CLEAN')\n" % HERE)
p = subprocess.run([sys.executable, "-E", "-s", "-c", probe], capture_output=True, text=True)
check("imports clean", p.returncode == 0 and "CLEAN" in p.stdout, p.stderr[-300:])
print("13 --hold-max: timed holds (s14) -- it may only SHORTEN a hold")
check("hold_max 0 / negative / NaN / > cap refuses",
      all(raises(lambda v=v: gt.TeleopCore(hold_max=v), gt.TeleopError) for v in (0, -1, float("nan"), 10.5)))
s = Sim(gt.TeleopCore(hold_max=1.5))
out = s.run(hold("w", 1.0, 3.0), 5.0)                    # W held 3.0 s, auto-repeating throughout
moving = [o for o in out if o[1] != (0.0, 0.0)]
check("motion from the press", moving and moving[0][0] == 1.0, moving[:1])
check("last moving Twist before 2.5 (press + 1.5)", moving and moving[-1][0] < 2.5, moving[-1:])
check("zero AT 2.50, the expiry tick", (2.5, (0.0, 0.0)) in out, between(out, 2.4, 2.6))
check("no motion after expiry, W still physically held and repeating to 4.0",
      not [o for o in out if o[0] >= 2.5 and o[1] != (0.0, 0.0)])
check("expiry counted once, and NOT as a real release",
      s.core.counts.get("hold_max") == 1 and s.core.counts["real_release"] == 0, s.core.counts)
check("held-through repeats after expiry classed as such", s.core.counts.get("held_through_stop", 0) > 0, s.core.counts)
s = Sim(gt.TeleopCore(hold_max=1.5))
out = s.run(hold("w", 1.0, 0.8), 3.0)                    # released early: hold_max must not lengthen it
check("early release stops as normal (first zero <= 1.83)",
      [o for o in out if o[1] == (0.0, 0.0)] and [o for o in out if o[1] == (0.0, 0.0)][0][0] <= 1.83,
      out[-4:])
check("early release: hold_max never fired", "hold_max" not in s.core.counts, s.core.counts)
s = Sim(gt.TeleopCore(hold_max=1.5))
out = s.run(hold("w", 1.0, 3.0) + hold("w", 4.5, 1.0, autorepeat=False), 6.0)
check("a real new press after expiry drives again", any(o[1] == (0.3, 0.0) for o in between(out, 4.5, 5.4)))
s = Sim(gt.TeleopCore(hold_max=1.5))
out = s.run(hold("w", 1.0, 3.0, autorepeat=False) + hold("a", 2.0, 2.0), 5.0)   # W+A: W expires first
check("combo: W's expiry at 2.50 stops BOTH keys", (2.5, (0.0, 0.0)) in out and
      not [o for o in out if o[0] >= 2.5 and o[1] != (0.0, 0.0)], between(out, 2.4, 2.7))

p = subprocess.run([sys.executable, os.path.join(HERE, "go2_teleop.py"), "--dry", "--vx", "0.5"],
                   capture_output=True, text=True, stdin=subprocess.DEVNULL)
check("--vx 0.5 refuses before any ROS import (rc 2)", p.returncode == 2 and "REFUSING" in p.stdout,
      (p.returncode, p.stdout[-200:], p.stderr[-200:]))
p = subprocess.run([sys.executable, os.path.join(HERE, "go2_teleop.py")], capture_output=True, text=True)
check("neither --armed nor --dry refuses (argparse rc 2)", p.returncode == 2)

print()
print("%d checks, %d failure(s)" % (n_checks, len(fails)))
if n_checks < 40:
    print("FAIL: fewer checks ran than written -- refusing to report success")
    sys.exit(1)
print("ALL TELEOP OFFLINE TESTS PASS" if not fails else "FAILURES:\n  - " + "\n  - ".join(fails))
sys.exit(1 if fails else 0)
