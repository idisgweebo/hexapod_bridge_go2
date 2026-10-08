#!/usr/bin/env python3
"""Offline tests for key_timing_probe.py -- no ROS, no display, no keyboard."""
import csv
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import key_timing_probe as k  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(f"{'PASS' if cond else 'FAIL'}  {name}  {detail}")
    if not cond:
        FAILS.append(name)


def x11_hold(t0, n_repeats, delay=0.5, interval=0.0303, pair=0.0004, key="w"):
    """X11 non-detectable auto-repeat: press, then (release, press) pairs, then a real release."""
    ev = [{"t": t0, "kind": "press", "key": key}]
    t = t0 + delay
    for _ in range(n_repeats):
        ev.append({"t": t, "kind": "release", "key": key})
        ev.append({"t": t + pair, "kind": "press", "key": key})
        t += interval
    ev.append({"t": t, "kind": "release", "key": key})
    return ev


def term_hold(t0, n_repeats, delay=0.5, interval=0.0303, key="0x77"):
    ev = [{"t": t0, "kind": "press", "key": key}]
    t = t0 + delay
    for _ in range(n_repeats):
        ev.append({"t": t, "kind": "press", "key": key})
        t += interval
    return ev


# 1. x11: three holds + a tap + focus noise
ev = x11_hold(0.0, 80) + x11_hold(5.0, 80) + x11_hold(10.0, 80)
ev += [{"t": 15.0, "kind": "press", "key": "w"}, {"t": 15.09, "kind": "release", "key": "w"}]
ev += [{"t": 16.0, "kind": "focus_out", "key": ""}, {"t": 16.5, "kind": "focus_in", "key": ""}]
c = k.classify(ev)
roles = [e["role"] for e in c]
check("x11 synthetic releases counted", roles.count("synthetic_release") == 240, roles.count("synthetic_release"))
check("x11 real releases counted", roles.count("release") == 4, roles.count("release"))
check("x11 downs counted", roles.count("down") == 4, roles.count("down"))
hs = k.holds(c, "x11")
check("x11 four holds", len(hs) == 4, len(hs))
s = k.hold_stats(hs[0])
check("x11 first gap = delay + pair gap", abs(s["first_gap"] - 0.5004) < 1e-9, s["first_gap"])
check("x11 median interval 30.3 ms", abs(s["median_interval"] - 0.0303) < 1e-9, s["median_interval"])
check("x11 every hold ends in a real release", all(h["real_release"] for h in hs))
check("x11 tap has one press", len(hs[3]["presses"]) == 1)

# 2. a release-press gap just over PAIR_S is NOT a pair (it is a real release and a new press)
ev2 = [{"t": 0.0, "kind": "press", "key": "w"}, {"t": 1.0, "kind": "release", "key": "w"},
       {"t": 1.0 + k.PAIR_S * 1.5, "kind": "press", "key": "w"}, {"t": 2.0, "kind": "release", "key": "w"}]
r2 = [e["role"] for e in k.classify(ev2)]
check("gap > PAIR_S is a real release + new down", r2 == ["down", "release", "down", "release"], r2)

# 3. a release of key A followed by a press of key B is not a pair
ev3 = [{"t": 0.0, "kind": "press", "key": "w"}, {"t": 1.0, "kind": "release", "key": "w"},
       {"t": 1.0001, "kind": "press", "key": "a"}, {"t": 2.0, "kind": "release", "key": "a"}]
r3 = [e["role"] for e in k.classify(ev3)]
check("different key is not a pair", r3 == ["down", "release", "down", "release"], r3)

# 4. term: holds split by BURST_S, taps are single-event holds
ev4 = term_hold(0.0, 80) + term_hold(5.0, 80) + [{"t": 9.0, "kind": "press", "key": "0x77"},
                                                  {"t": 10.2, "kind": "press", "key": "0x77"}]
hs4 = k.holds(k.classify(ev4), "term")
check("term holds split", len(hs4) == 4, len(hs4))
s4 = k.hold_stats(hs4[0])
check("term first gap 500 ms", abs(s4["first_gap"] - 0.5) < 1e-9, s4["first_gap"])
check("term never claims a real release", not any(h["real_release"] for h in hs4))

# 5. report verdict
rep = k.report(ev, "x11", {"t": "synthetic"})
check("report flags first gap > cmd_timeout", "EXCEEDS" in rep)
rep_fast = k.report(x11_hold(0.0, 40, delay=0.25), "x11", {})
check("report 'within' for a 250 ms delay", "within" in rep_fast and "EXCEEDS" not in rep_fast)
check("report with no events does not crash", "nothing to say" in k.report([], "term", {}))

# 6. --replay runs in a clean subprocess with no display and no tty
with tempfile.TemporaryDirectory() as d:
    p = os.path.join(d, "e.csv")
    with open(p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["t", "kind", "key", "xtime"])
        w.writeheader()
        for e in ev:
            w.writerow({**e, "xtime": ""})
    env = {k_: v for k_, v in os.environ.items() if k_ != "DISPLAY"}
    r = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                     "key_timing_probe.py"), "x11", "--replay", p],
                       capture_output=True, text=True, env=env, stdin=subprocess.DEVNULL)
    check("replay subprocess exit 0", r.returncode == 0, r.stderr.strip()[-200:])
    check("replay reproduces the pair count", "n=240" in r.stdout)

print(f"\n{'ALL PASS' if not FAILS else 'FAILED: ' + ', '.join(FAILS)}")
sys.exit(1 if FAILS else 0)
