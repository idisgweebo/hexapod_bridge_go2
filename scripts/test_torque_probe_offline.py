#!/usr/bin/env python3
"""Offline test of go2_torque_probe, with the ROS imports stubbed.

Runs on the laptop with no ROS, no container and no robot, BEFORE the probe is
pointed at the Go2. It exists because the probe has to decide between three
outcomes that look similar in code and mean completely different things:

    rear responds      -> genuine zero when unloaded, contact inference viable
    rear stays zero    -> firmware does not report rear torque       (a finding)
    nothing responds   -> the control failed, so NOTHING is concluded

Session 5's lesson was that a plausible-looking value passed a predicate that
only asked whether it was non-zero. The tests below feed synthetic captures in
which the right answer is known by construction, and check that the probe says
it -- including that it refuses to conclude when the control fails.
"""
import math, os, sys, types

try:
    import numpy as np
except ImportError:                      # numpy absence must not skip test 8
    np = None

# ---- stub the ROS packages so the module imports on a host with no ROS ----
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
import go2_torque_probe as T

U, C = T.STEP_HIP_THIGH, T.STEP_CALF


def step(i):
    return C if i % 3 == 2 else U


class Motor:
    def __init__(self, q=0.0, dq=0.0, tau=0.0, temp=30, mode=1):
        self.mode = mode
        self.q, self.dq, self.tau_est = q, dq, tau
        self.ddq = self.q_raw = self.dq_raw = self.ddq_raw = 0.0
        self.temperature = temp
        self.lost = 0
        self.reserve = [0, 518]


class BMS:
    def __init__(self): self.soc = 45


class Low:
    """One /lowstate sample. `cts` is 12 tau_est values IN LSB COUNTS, so the
    tests state loads the way the probe reports them."""
    def __init__(self, cts, tick=1000, qs=None, temps=None, power_v=28.4):
        self.tick = tick
        self.power_v, self.power_a = power_v, 1.2
        ff = [112, 109, 89, 104]
        self.foot_force = np.array(ff, dtype="int16") if np is not None else ff
        self.foot_force_est = [0, 0, 0, 0]
        qs = qs if qs is not None else [-0.04, 1.25, -2.78] * 4
        temps = temps if temps is not None else [33, 31, 30] * 4
        self.motor_state = [
            Motor(q=qs[i], dq=-0.0155, tau=cts[i] * step(i), temp=temps[i])
            for i in range(12)
        ] + [Motor(mode=0) for _ in range(8)]
        self.bms_state = BMS()


class IMU:
    def __init__(self):
        self.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.rpy = [0.0, 0.0, 0.0]
        self.accelerometer = [0.0, 0.0, 9.4]
        self.gyroscope = [0.0, 0.0, 0.0]


class Sport:
    def __init__(self, body_height):
        self.imu_state = IMU()
        self.mode, self.gait_type = 1, 0
        self.body_height = body_height
        self.foot_force = [0, 0, 0, 0]
        self.yaw_speed = 0.0


def capture(cts_fn, n=400, qs=None, temps=None, body_height=0.07):
    """Build (low, sport). cts_fn(sample_index, motor_index) -> LSB counts."""
    low, sport = [], []
    t0 = 1_700_000_000.0
    for k in range(n):
        t = t0 + k * 0.002
        low.append((t, Low([cts_fn(k, i) for i in range(12)],
                           tick=1000 + k, qs=qs, temps=temps)))
        sport.append((t, Sport(body_height)))
    return low, sport


def run(low, sport, posture):
    stats = T.per_motor_stats(low)
    rep = T.Report()
    T.evaluate(low, sport, rep, posture, stats)
    return rep, stats


def row(rep, pid):
    return next(r for r in rep.rows if r[0] == pid)


def verdict(rep, pid):
    return row(rep, pid)[4]


fails = []

# dither: +-1..3 counts, sign flipping -- what BOTH leg pairs show lying
def dither(k, i):
    return (k % 7) - 3 if (k + i) % 2 else 0

# a real standing load: steady sign, tens of counts, small ripple
def loaded(k, i):
    if i % 3 == 0:
        return 12 + (k % 3)          # hip, lightly loaded
    return (60 if i % 3 == 1 else -95) + (k % 5)   # thigh up, calf down


# ---- 1. standing, rear ALIVE: the prediction holds ----
rep, _ = run(*capture(lambda k, i: loaded(k, i)), "standing")
ok = (verdict(rep, "PT1"), verdict(rep, "PT2"), verdict(rep, "PT4"), verdict(rep, "PT5"))
print("1 standing, rear alive   : PT1/PT2/PT4/PT5 =", ok)
if ok != (True, True, True, True): fails.append("1: all four should PASS")
if rep.failed_critical(): fails.append("1: no control should fail")

# ---- 2. standing, rear ZERO while the front responds: PT2 FALSIFIED, valid ----
rep, _ = run(*capture(lambda k, i: 0 if i >= 6 else loaded(k, i)), "standing")
print("2 standing, rear zero    : PT2 =", verdict(rep, "PT2"),
      " PT4 =", verdict(rep, "PT4"), " critical_failed =", rep.failed_critical(),
      " falsified =", rep.falsified())
if verdict(rep, "PT2") is not False: fails.append("2: PT2 must be recorded as FALSIFIED")
if verdict(rep, "PT4") is not True: fails.append("2: front control must hold")
if rep.failed_critical(): fails.append("2: a falsified PT2 is NOT a control failure")
if "PT2" not in rep.falsified(): fails.append("2: PT2 must appear in falsified()")

# ---- 3. standing, NOTHING responds: the control fails, nothing is concluded ----
rep, _ = run(*capture(dither), "standing")
print("3 standing, all dither   : PT4 =", verdict(rep, "PT4"),
      " critical_failed =", rep.failed_critical())
if verdict(rep, "PT4") is not False: fails.append("3: front control must FAIL")
if "PT4" not in rep.failed_critical():
    fails.append("3: a failed front control must make the run INCONCLUSIVE")

# ---- 4. lying baseline: PT6 holds, PT2/PT4 not judged ----
rep, _ = run(*capture(dither), "lying")
print("4 lying baseline         : PT6 =", verdict(rep, "PT6"),
      " PT2 =", verdict(rep, "PT2"), " PT4 =", verdict(rep, "PT4"),
      " measured:", row(rep, "PT6")[3])
if verdict(rep, "PT6") is not True: fails.append("4: dither must satisfy PT6")
if verdict(rep, "PT2") is not None: fails.append("4: PT2 is unanswerable lying -- must be n/a")
if verdict(rep, "PT4") is not None: fails.append("4: PT4 is untestable lying -- must be n/a")

# ---- 4b. lying but actually loaded: PT6 must FAIL, not be waved through ----
rep, _ = run(*capture(loaded), "lying")
print("4b lying yet loaded      : PT6 =", verdict(rep, "PT6"), "(must be False)")
if verdict(rep, "PT6") is not False:
    fails.append("4b: a loaded 'lying' capture must fail PT6 -- probably a wrong --posture")

# ---- 5. quantisation: exact multiples pass; off-grid values FAIL ----
rep, _ = run(*capture(lambda k, i: loaded(k, i)), "standing")
print("5 quantisation, on-grid  : PT3 =", verdict(rep, "PT3"), row(rep, "PT3")[3])
if verdict(rep, "PT3") is not True: fails.append("5: exact multiples must pass PT3")

low, sport = capture(lambda k, i: loaded(k, i))
for _, m in low:                      # nudge one motor off the grid by half a count
    m.motor_state[7].tau_est += 0.5 * U
rep, _ = run(low, sport, "standing")
print("5b quantisation, off-grid: PT3 =", verdict(rep, "PT3"), row(rep, "PT3")[3])
if verdict(rep, "PT3") is not False: fails.append("5b: off-grid values must fail PT3")
if "RR_thigh" not in row(rep, "PT3")[3]:
    fails.append("5b: PT3 must name the offending motor")

# ---- 6. PT1 control: rear entries dead -> inconclusive, whatever tau says ----
low, sport = capture(lambda k, i: loaded(k, i),
                     qs=[-0.04, 1.25, -2.78] * 2 + [0.0] * 6,
                     temps=[33, 31, 30] * 2 + [0] * 6)
rep, _ = run(low, sport, "standing")
print("6 rear entries dead      : PT1 =", verdict(rep, "PT1"),
      " critical_failed =", rep.failed_critical())
if verdict(rep, "PT1") is not False: fails.append("6: dead rear entries must fail PT1")
if "PT1" not in rep.failed_critical(): fails.append("6: PT1 must be load-bearing")

# ---- 7. sign consistency ----
# A motor above the load threshold whose sign flips every sample is dither that
# happens to be large, not a DC load. PT5 must catch it.
rep, _ = run(*capture(lambda k, i: (40 if k % 2 else -40)), "standing")
print("7 large but sign-flipping: PT5 =", verdict(rep, "PT5"), row(rep, "PT5")[3])
if verdict(rep, "PT5") is not False: fails.append("7: sign-flipping load must fail PT5")

# nothing loaded at all -> PT5 is inconclusive, NOT a negative
rep, _ = run(*capture(dither), "lying")
print("7b nothing loaded        : PT5 =", verdict(rep, "PT5"), "|", row(rep, "PT5")[3])
if verdict(rep, "PT5") is not None: fails.append("7b: PT5 must be n/a when nothing is loaded")
if "INCONCLUSIVE" not in row(rep, "PT5")[3]: fails.append("7b: PT5 must say inconclusive")

# ---- 8. the table must agree with its own summary, all verdicts real bools ----
# This is the session-4 defect class: numpy.bool_(True) is not True, so a row
# rendered PASS could be counted as a failure. foot_force above is a real numpy
# array when numpy is installed, which is where such a value would come from.
rep, _ = run(*capture(lambda k, i: loaded(k, i)), "standing")
tbl = rep.render()
for pid, _, _, _, ok in rep.rows:
    mark = {True: "PASS", False: "FAIL", None: "n/a"}[ok]
    line = next(l for l in tbl.splitlines() if l.startswith(pid + " "))
    if mark not in line:
        fails.append(f"8: {pid} table says {line.split()[-1]}, summary says {mark}")
    if ok is not None and not isinstance(ok, bool):
        fails.append(f"8: {pid} verdict is {type(ok)}, not a real bool")
print("8 table/summary agree, all verdicts real bools "
      f"(numpy {'in use' if np is not None else 'ABSENT -- weaker test'})")

# ---- 9. spares: a layout shift bleeding data into 12..19 must be caught ----
low, sport = capture(lambda k, i: loaded(k, i))
for _, m in low:
    m.motor_state[13].q = 0.5
rep, _ = run(low, sport, "standing")
print("9 spares live            : PT9 =", verdict(rep, "PT9"), "(must be False)")
if verdict(rep, "PT9") is not False: fails.append("9: live spares must fail PT9")

# ---- 10. CSV: the standing numbers must actually reach disk ----
# The whole reason this question is still open is that s5stand2 held these in
# memory and never wrote them.
low, sport = capture(lambda k, i: loaded(k, i), n=5)
csv_path = os.path.join(HERE, "_t_torque.csv")
T.write_csv(low, csv_path)
with open(csv_path) as fh:
    lines = [l.rstrip("\n") for l in fh]
head = lines[0].split(",")
body = [l.split(",") for l in lines[1:]]
print(f"10 csv: {len(head)} columns, {len(body)} rows, header[:4]={head[:4]}")
if len(head) != 8 + 12 * 5: fails.append(f"10: expected {8 + 12 * 5} columns, got {len(head)}")
if len(body) != 5: fails.append("10: one row per sample")
if any(len(r) != len(head) for r in body): fails.append("10: ragged rows")
if "tau_RR_thigh" not in head or "cnt_RR_thigh" not in head:
    fails.append("10: per-motor tau and count columns must be named by motor")
ci = head.index("cnt_FL_calf")
if abs(float(body[0][ci]) - loaded(0, 5)) > 1e-6:
    fails.append("10: count column does not match the synthetic load")
os.remove(csv_path)

# ---- 11. no data -> INCONCLUSIVE (exit 2), never a finding about tau_est ----
argv, ret = sys.argv, None
sys.argv = ["go2_torque_probe.py", "--posture", "lying", "--timeout", "0.2",
            "--csv", os.path.join(HERE, "_t_nodata.csv")]
try:
    ret = T.main()
finally:
    sys.argv = argv
print("11 no samples            : main() ->", ret, "(must be 2)")
if ret != 2: fails.append("11: an empty capture must return 2, not 0 or 1")

# ---- 12. the safety claim in the docstring, checked against the source ----
src = open(os.path.join(HERE, "go2_torque_probe.py")).read()
for banned in ("create_publisher", "create_client", "ActionClient"):
    # the docstring names them as forbidden; count only real call sites
    if f"self.{banned}" in src or f".{banned}(" in src:
        fails.append(f"12: probe contains a {banned} call site -- it must be read-only")
print("12 read-only: no publisher, service client or action client in the source")

print()
print("FAILURES:" if fails else "ALL TORQUE PROBE TESTS PASS")
for f in fails:
    print("  -", f)
sys.exit(1 if fails else 0)
