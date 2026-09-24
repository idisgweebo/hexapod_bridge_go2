#!/usr/bin/env python3
"""Offline test of go2_unload_probe, with the ROS imports stubbed.

Runs with no ROS, no container and no robot, BEFORE anyone puts a hand under a
15 kg machine. The synthetic captures below have a known answer by construction:
a leg that unloads, a leg that unloads without the others taking up the weight,
a lift that does not come back to where it started, and a communications gap
dressed up as a contact event.

The last one is the point of PU0. A dropout produces a simultaneous step down in
every leg -- which is exactly what lifting the whole robot looks like. If the
probe cannot tell those apart it is not an instrument, and the cheapest place to
discover that is here.
"""
import csv, math, os, random, sys, types

# ---- stub ROS so the module imports on a host with no ROS ----
rclpy = types.ModuleType("rclpy")
rclpy.init = lambda *a, **k: None
rclpy.shutdown = lambda *a, **k: None
rclpy.ok = lambda: True
rclpy.spin_once = lambda *a, **k: None
node_mod = types.ModuleType("rclpy.node")
class _Node:
    def __init__(self, *a, **k): pass
    def create_subscription(self, *a, **k): pass
    def destroy_node(self): pass
node_mod.Node = _Node
rclpy.node = node_mod
ug = types.ModuleType("unitree_go")
ugmsg = types.ModuleType("unitree_go.msg")
class LowState: pass
class SportModeState: pass
ugmsg.LowState, ugmsg.SportModeState = LowState, SportModeState
ug.msg = ugmsg
for n, m in [("rclpy", rclpy), ("rclpy.node", node_mod),
             ("unitree_go", ug), ("unitree_go.msg", ugmsg)]:
    sys.modules[n] = m

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import go2_unload_probe as U
from go2_torque_probe import step_for, LEGS

RNG = random.Random(20260924)
HZ = 500.0
# How a leg's proxy splits across hip/thigh/calf, roughly as session 6 measured
# the rear legs (hip 322, thigh 55, calf 132 counts).
SPLIT = (0.62, 0.11, 0.27)


class Motor:
    def __init__(self, tau, temp=30):
        self.mode = 1
        self.q, self.dq, self.tau_est = 0.5, 0.01, tau
        self.ddq = self.q_raw = self.dq_raw = self.ddq_raw = 0.0
        self.temperature = temp
        self.lost = 0


class Low:
    def __init__(self, cnts, tick):
        self.tick = tick
        self.power_v, self.power_a = 29.5, 1.5
        self.foot_force = [109, 109, 89, 105]
        self.foot_force_est = [0, 0, 0, 0]
        self.motor_state = [Motor(cnts[i] * step_for(i)) for i in range(12)] + \
                           [Motor(0.0) for _ in range(8)]


def leg_counts(proxy, loaded, rng):
    """Split a leg's proxy across its three joints, with dither.

    Loaded joints hold one sign (gravity is DC); an unloaded joint dithers
    around zero and flips sign. That is the session-6 discriminator, so the
    synthetic data must reproduce it or PU6 would be testing nothing."""
    out = []
    for j in range(3):
        base = proxy * SPLIT[j]
        if loaded:
            out.append(round(base + rng.uniform(-2, 2)))
        else:
            out.append(round(rng.uniform(-3, 3)))
    return out


def build(profile, n=6000, tick_gap_at=None, ramp=25):
    """profile(k) -> (proxy per leg, loaded flag per leg)."""
    rows, tick = [], 1000
    for k in range(n):
        if tick_gap_at is not None and k == tick_gap_at:
            tick += 500                  # half a second of robot time, no samples
        else:
            tick += 2                    # 500 Hz against a 1 kHz tick
        proxies, loaded = profile(k)
        cnts = []
        for leg in range(4):
            cnts += leg_counts(proxies[leg], loaded[leg], RNG)
        rows.append(Low(cnts, tick))
    return rows


def ramped(k, k0, a, b, ramp):
    """Smooth step from a to b centred on k0 over `ramp` samples."""
    if k <= k0:
        return a
    if k >= k0 + ramp:
        return b
    f = (k - k0) / ramp
    return a + (b - a) * f


def run(rows, t0=1_700_000_000.0, edge_counts=U.EDGE_COUNTS, path=None, step="singleleg"):
    path = path or os.path.join(HERE, "_t_unload.csv")
    cap = U.Capture(path)
    for k, m in enumerate(rows):
        cap.add(t0 + k / HZ, m)
    cap.close()
    rep = U.Report(critical=("PU0",))
    args = types.SimpleNamespace(edge_counts=edge_counts, step=step)
    edges, _ = U.analyse(cap, rep, [], args)
    return rep, edges, path


def verdict(rep, pid):
    return next(r for r in rep.rows if r[0] == pid)[4]


def measured(rep, pid):
    return next(r for r in rep.rows if r[0] == pid)[3]


fails = []
LIFT, DROP = 2000, 4000          # sample indices of lift and replace

# ---- 1. a clean single-leg lift: RR unloads, the other three take it up ----
def p_clean(k, back=500.0):
    rr = ramped(k, LIFT, 500.0, 8.0, 25) if k < DROP else ramped(k, DROP, 8.0, back, 25)
    share = (500.0 - rr) / 3.0           # the weight has to go somewhere
    pr = [300.0 + share, 300.0 + share, rr, 300.0 + share]
    return pr, [True, True, rr > 50, True]

rep, edges, _ = run(build(p_clean))
print("1 clean lift     : PU0..PU6 =", [verdict(rep, f"PU{i}") for i in range(7)])
print("                   edges:", [(LEGS[e["leg"]], round(e["delta"])) for e in edges])
for pid in ("PU0", "PU1", "PU2", "PU3", "PU4", "PU5", "PU6"):
    if verdict(rep, pid) is not True:
        fails.append(f"1: {pid} should PASS, got {verdict(rep, pid)} ({measured(rep, pid)})")
if rep.failed_critical(): fails.append("1: no control should fail")

# ---- 2. hysteresis: the leg comes back to 60 % of where it started ----
rep, _, _ = run(build(lambda k: p_clean(k, back=300.0)))
print("2 hysteresis     : PU5 =", verdict(rep, "PU5"), "|", measured(rep, "PU5"))
if verdict(rep, "PU5") is not False:
    fails.append("2: a leg that does not return must FAIL PU5")

# ---- 3. no redistribution: RR unloads and nobody picks the weight up ----
def p_norediv(k):
    rr = ramped(k, LIFT, 500.0, 8.0, 25) if k < DROP else ramped(k, DROP, 8.0, 500.0, 25)
    return [300.0, 300.0, rr, 300.0], [True, True, rr > 50, True]

rep, _, _ = run(build(p_norediv))
print("3 no redistrib   : PU2 =", verdict(rep, "PU2"), "|", measured(rep, "PU2"))
if verdict(rep, "PU2") is not False:
    fails.append("3: weight vanishing into nowhere must FAIL PU2")

# ---- 3b. the whole-body lift: all four drop at once, and that is CORRECT ----
# Gating this on "somebody must rise" would fail a good capture. PU2 is
# step-aware for exactly this reason; PU0 is what separates it from a dropout.
def p_whole(k):
    v = ramped(k, LIFT, 400.0, 6.0, 25) if k < DROP else ramped(k, DROP, 6.0, 400.0, 25)
    return [v] * 4, [v > 50] * 4

rep_w, _, _ = run(build(p_whole), step="wholebody")
rep_s, _, _ = run(build(p_whole), step="singleleg")
print("3b whole-body    : as wholebody PU2 =", verdict(rep_w, "PU2"),
      "| mislabelled singleleg PU2 =", verdict(rep_s, "PU2"))
if verdict(rep_w, "PU2") is not True:
    fails.append("3b: all four legs dropping together IS the whole-body signature")
if verdict(rep_s, "PU2") is not False:
    fails.append("3b: the same data labelled singleleg must fail -- the step matters")
if verdict(rep_w, "PU0") is not True:
    fails.append("3b: no dropout here, PU0 must hold")

# ---- 4. PU0: a dropout looks exactly like a whole-body lift ----
rep, _, _ = run(build(p_clean, tick_gap_at=3000))
print("4 tick gap       : PU0 =", verdict(rep, "PU0"), "|", measured(rep, "PU0"),
      "| critical:", rep.failed_critical())
if verdict(rep, "PU0") is not False: fails.append("4: a tick gap must FAIL PU0")
if "PU0" not in rep.failed_critical():
    fails.append("4: PU0 must be load-bearing -- edges after a gap are uninterpretable")

# ---- 5. baseline: nothing happens. No edge is a FINDING, not a pass ----
def p_flat(k):
    return [300.0, 300.0, 300.0, 300.0], [True] * 4

rep, edges, _ = run(build(p_flat))
print("5 flat baseline  : PU1 =", verdict(rep, "PU1"), " edges =", len(edges),
      " PU2/PU4/PU5/PU6 =", [verdict(rep, f"PU{i}") for i in (2, 4, 5, 6)])
if verdict(rep, "PU1") is not False: fails.append("5: no edges must be reported as such")
for pid in ("PU2", "PU4", "PU5", "PU6"):
    if verdict(rep, pid) is not None:
        fails.append(f"5: {pid} must be INCONCLUSIVE with no edge, not a negative")
    if "INCONCLUSIVE" not in measured(rep, pid):
        fails.append(f"5: {pid} must say inconclusive")

# ---- 6. a slow transition must fail the timing prediction, not be hidden ----
def p_slow(k):
    rr = ramped(k, LIFT, 500.0, 8.0, 400) if k < DROP else ramped(k, DROP, 8.0, 500.0, 400)
    share = (500.0 - rr) / 3.0
    return [300.0 + share] * 2 + [rr] + [300.0 + share], [True, True, rr > 50, True]
# (order: FR FL RR RL -- RR is index 2)
rep, _, _ = run(build(p_slow))
print("6 slow edge      : PU3 =", verdict(rep, "PU3"), "|", measured(rep, "PU3"))
if verdict(rep, "PU3") is not False:
    fails.append("6: an 800 ms transition must FAIL the < 200 ms prediction")
# The VERDICT being right is not enough: the reported duration must be right
# too. With a +-lag search window an 800 ms ramp measured 296 ms -- clipped to
# the window, and invisible because the gate still failed.
_slow_ms = float(measured(rep, "PU3").split()[1].rstrip("ms,"))
if not 600 <= _slow_ms <= 1000:
    fails.append(f"6: an 800 ms ramp reported as {_slow_ms:.0f} ms -- window clipping")

# ---- 7. replay round-trip: the offline re-analysis must match the live one ----
rep_live, edges_live, path = run(build(p_clean))
cap = U.Replay(path)
rep_replay = U.Report(critical=("PU0",))
edges_replay, _ = U.analyse(cap, rep_replay, [],
                            types.SimpleNamespace(edge_counts=U.EDGE_COUNTS, step="singleleg"))
same_rows = [(a[0], a[4]) for a in rep_live.rows] == [(b[0], b[4]) for b in rep_replay.rows]
same_edges = len(edges_live) == len(edges_replay) and all(
    abs(a["delta"] - b["delta"]) < 1e-6 for a, b in zip(edges_live, edges_replay))
print(f"7 replay         : rows match={same_rows}  edges match={same_edges} "
      f"({len(edges_replay)} edges)")
if not same_rows: fails.append("7: replay verdicts differ from the live analysis")
if not same_edges: fails.append("7: replay edges differ from the live analysis")

# ---- 8. table/summary agreement, real bools (the session-4 defect class) ----
tbl = rep_live.render()
for pid, _, _, _, ok in rep_live.rows:
    mark = {True: "PASS", False: "FAIL", None: "n/a"}[ok]
    line = next(l for l in tbl.splitlines() if l.startswith(pid + " "))
    if mark not in line:
        fails.append(f"8: {pid} table says {line.split()[-1]}, summary says {mark}")
    if ok is not None and not isinstance(ok, bool):
        fails.append(f"8: {pid} verdict is {type(ok)}, not a real bool")
print("8 table/summary agree, all verdicts real bools")

# ---- 9. the proxy is not silently a force, and the source stays read-only ----
src = open(os.path.join(HERE, "go2_unload_probe.py")).read()
for banned in ("create_publisher", "create_client", "ActionClient"):
    if f"self.{banned}" in src or f".{banned}(" in src:
        fails.append(f"9: probe contains a {banned} call site -- it must be read-only")
if "NOT a force" not in src and "not a force" not in src:
    fails.append("9: the proxy's non-force caveat has been edited out of the source")
print("9 read-only, and the 'proxy is not a force' caveat is still in the source")

for p in (os.path.join(HERE, "_t_unload.csv"),):
    if os.path.exists(p):
        os.remove(p)

print()
print("FAILURES:" if fails else "ALL UNLOAD PROBE TESTS PASS")
for f in fails:
    print("  -", f)
sys.exit(1 if fails else 0)
