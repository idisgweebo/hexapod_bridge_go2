#!/usr/bin/env python3
"""Offline test of the new P8s signed-yaw block, with the ROS imports stubbed.

Feeds synthetic /sportmodestate and /lowstate samples whose yaw differs by a
KNOWN constant, then checks that P8s recovers that constant and that the
match verdict is right. Guards the session-4 lesson: a tooling bug produced a
summary that disagreed with its own table.
"""
import math, sys, types, os

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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import go2_layout_acceptance as A


class Stamp:
    def __init__(self, t):
        self.sec = int(t); self.nanosec = int((t - int(t)) * 1e9)

class IMU:
    def __init__(self, rpy):
        self.rpy = rpy
        r, p, y = rpy
        cy, sy = math.cos(y / 2), math.sin(y / 2)
        cp, sp = math.cos(p / 2), math.sin(p / 2)
        cr, sr = math.cos(r / 2), math.sin(r / 2)
        self.quaternion = [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                           cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy]
        self.accelerometer = [0.0, 0.0, 9.80665]
        self.gyroscope = [0.0, 0.0, 0.0]

class Motor:
    def __init__(self, q=0.0): self.q = q; self.dq = 0.0; self.tau_est = 0.0

class BMS:
    def __init__(self): self.soc = 80

class Sport:
    def __init__(self, t_robot, rpy):
        self.stamp = Stamp(t_robot); self.imu_state = IMU(rpy)
        self.mode = 1; self.gait_type = 0
        self.foot_position_body = [0.0] * 12
        self.foot_speed_body = [0.0] * 12
        self.foot_force = [0, 0, 0, 0]      # unpopulated on this firmware
        self.yaw_speed = self.imu_state.gyroscope[2]   # firmware sets these equal

class Low:
    def __init__(self, rpy, tick):
        self.imu_state = IMU(rpy); self.tick = tick
        self.motor_state = [Motor() for _ in range(20)]
        self.bms_state = BMS(); self.power_v = 28.5
        self.foot_force = [105, 105, 88, 100]          # LowState IS populated
        self.head = [0xFE, 0xEF]


def build(true_off, n=600, offset_s=1360.0, pair_dt=0.001, low_every=1):
    """sport yaw = low yaw + true_off.

    sport is emitted every 10 ms, shifted by pair_dt. low is emitted every
    low_every-th slot -- raising low_every thins the low stream so that no
    low sample lands within 5 ms of a sport sample, which is what actually
    starves the pairing. Shifting both dense streams does NOT, because
    nearest-neighbour pairing just picks a different, equally close sample.
    """
    sport, low = [], []
    t0 = 1_700_000_000.0
    for i in range(n):
        t = t0 + i * 0.01
        base = -0.4 + 0.0001 * math.sin(i / 30.0)   # slow common motion
        if i % low_every == 0:
            low.append((t, Low([0.01, 0.02, base], 1000 + i)))
        sport.append((t + pair_dt, Sport(t - offset_s, [0.01, 0.02, base + true_off])))
    return sport, low


def p8s_row(rep):
    return next(r for r in rep.rows if r[0] == "P8s")


fails = []
csv = os.path.join(os.path.dirname(os.path.abspath(__file__)), "t.csv")

# 1. recovers a known offset, and matches the reference it was given
sport, low = build(0.150760)
rep = A.Report(); A.evaluate(sport, low, rep, csv, yaw_ref=0.150760, offset_ref=1353.94)
row = p8s_row(rep)
print("1 recover+match :", row[3], "->", row[4])
if row[4] is not True: fails.append("1: should MATCH a reproduced offset")
if f"{0.150760:+.6f}" not in row[3]: fails.append("1: mean not recovered")

# 2. a re-randomised yaw origin must NOT match
sport, low = build(-0.732)
rep = A.Report(); A.evaluate(sport, low, rep, csv, yaw_ref=0.150760, offset_ref=1353.94)
row = p8s_row(rep)
print("2 re-randomised :", row[3], "->", row[4])
if row[4] is not False: fails.append("2: should FAIL to match")

# 3. no reference supplied -> record, do not judge
sport, low = build(0.150760)
rep = A.Report(); A.evaluate(sport, low, rep, csv, yaw_ref=None, offset_ref=None)
row = p8s_row(rep)
print("3 no reference  :", row[2], "->", row[4])
if row[4] is not None: fails.append("3: should be n/a without a reference")

# 4. wrap-around: offset near +pi must not be mis-signed
sport, low = build(3.10)
rep = A.Report(); A.evaluate(sport, low, rep, csv, yaw_ref=3.10)
row = p8s_row(rep)
print("4 near +pi      :", row[3], "->", row[4])
if row[4] is not True: fails.append("4: wrap handling wrong")

# 5. NO tight pairs -> inconclusive, NOT a false negative.
# BOTH streams must be sparse, and sport must sit in the middle of each low
# gap. Thinning only one stream does not work: nearest-neighbour pairing just
# picks a different, equally close sample, which is why the first two attempts
# at this test kept finding 0 ms pairs.
sport, low = [], []
_t0 = 1_700_000_000.0
for i in range(300):
    _b = -0.4 + 0.0001 * math.sin(i / 30.0)
    low.append((_t0 + i * 0.20, Low([0.01, 0.02, _b], 1000 + i)))
    sport.append((_t0 + i * 0.20 + 0.10, Sport(_t0 + i * 0.20 - 1360.0,
                                               [0.01, 0.02, _b + 0.150760])))
rep = A.Report(); A.evaluate(sport, low, rep, csv, yaw_ref=0.150760)
row = p8s_row(rep)
print("5 no tight pairs:", row[3], "->", row[4])
if row[4] is not None: fails.append("5: must be inconclusive, not False")
if "INCONCLUSIVE" not in row[3]: fails.append("5: must say inconclusive")

# 6. the summary must agree with its own table (the session-4 bug class)
sport, low = build(0.150760)
rep = A.Report(); A.evaluate(sport, low, rep, csv, yaw_ref=0.150760)
tbl = rep.render()
for pid, _, _, _, ok in rep.rows:
    mark = {True: "PASS", False: "FAIL", None: "n/a"}[ok]
    line = next(l for l in tbl.splitlines() if l.startswith(pid + " ") or l.startswith(pid + "  "))
    if mark not in line: fails.append(f"6: {pid} table says {line.split()[-1]}, summary says {mark}")
    if ok is not None and not isinstance(ok, bool):
        fails.append(f"6: {pid} verdict is {type(ok)}, not a real bool")
print("6 table/summary agree, all verdicts real bools")

print()
print("FAILURES:" if fails else "ALL P8s TESTS PASS")
for f in fails: print("  -", f)
sys.exit(1 if fails else 0)
