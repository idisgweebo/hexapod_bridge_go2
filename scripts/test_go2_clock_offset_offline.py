#!/usr/bin/env python3
"""Offline tests for go2_adapter's clock-offset logic and re-stamping.

Runs with NO ROS, NO container and NO robot. clock_offset.py and restamp.py
import nothing but the standard library, which is the point of splitting them
out of the nodes -- see scripts/test_torque_probe_offline.py's sibling problem,
where a suite that stubbed rclpy into sys.modules could never have noticed an
unguarded import. Here there is nothing to stub.

    python3 scripts/test_go2_clock_offset_offline.py
"""
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ros2_ws", "src", "go2_adapter"))

from go2_adapter.clock_offset import (  # noqa: E402
    ClockOffsetEstimator, CONVERGING, CONVERGED, FAULTED)
from go2_adapter import restamp  # noqa: E402

fails = []


def chk(name, got, want):
    if got != want:
        fails.append(f"{name}: got {got!r}, want {want!r}")
        print(f"  FAIL {name}: got {got!r}, want {want!r}")
    else:
        print(f"  pass {name}")


def chk_close(name, got, want, tol):
    if got is None or abs(got - want) > tol:
        fails.append(f"{name}: got {got!r}, want {want!r} +/-{tol}")
        print(f"  FAIL {name}: got {got!r}, want {want!r} +/-{tol}")
    else:
        print(f"  pass {name} ({got:.6f})")


def feed(est, true_offset, n, rng, base=1_700_000_000.0, jitter=0.02):
    """Simulate n samples. Transport delay is POSITIVE-ONLY, which is the
    property that makes the minimum the right estimator and the mean the wrong
    one."""
    for i in range(n):
        stamp = base + i * 0.0533          # /utlidar/robot_pose ~18.75 Hz
        arrival = stamp + true_offset + rng.uniform(0.0, jitter)
        est.add(arrival, stamp)
    return est


print("1. recovery of a known offset")
rng = random.Random(1)
e = feed(ClockOffsetEstimator(window=100), 1396.079, 100, rng)
chk("state is CONVERGED", e.state, CONVERGED)
chk_close("offset recovered", e.offset, 1396.079, 0.01)

print("\n2. INCONCLUSIVE below threshold -- None, never a partial guess")
e = feed(ClockOffsetEstimator(window=100), 1396.079, 99, rng)
chk("state still CONVERGING", e.state, CONVERGING)
chk("offset is None, not a partial estimate", e.offset, None)

print("\n3. the anti-hardcoding test: two different truths, two different answers")
# Guards the failure this project has already been bitten by -- the yaw offset
# was constant within a boot and re-randomised across boots, and anything that
# hardcoded it was wrong. A constant must not be able to pass this suite.
a = feed(ClockOffsetEstimator(window=100), 1375.974, 100, random.Random(2))
b = feed(ClockOffsetEstimator(window=100), 1396.079, 100, random.Random(3))
chk_close("estimator A recovers its own truth", a.offset, 1375.974, 0.01)
chk_close("estimator B recovers its own truth", b.offset, 1396.079, 0.01)
chk("the two answers actually differ", abs(a.offset - b.offset) > 20.0, True)

print("\n4. the minimum beats the mean under one-sided delay")
rng = random.Random(4)
stamps, arrivals = [], []
for i in range(200):
    s = 1_700_000_000.0 + i * 0.0533
    stamps.append(s)
    arrivals.append(s + 1396.079 + rng.uniform(0.0, 0.5))   # heavy one-sided
e = ClockOffsetEstimator(window=200)
for s, ar in zip(stamps, arrivals):
    e.add(ar, s)
mean_est = sum(a - s for a, s in zip(arrivals, stamps)) / len(stamps)
chk("min-estimator within 10 ms of truth", abs(e.offset - 1396.079) < 0.01, True)
chk("mean-estimator would be biased high by >100 ms",
    (mean_est - 1396.079) > 0.1, True)

print("\n5. a mid-session step FAULTS -- it is never silently absorbed")
e = feed(ClockOffsetEstimator(window=100, step_threshold=0.5, step_consecutive=10),
         1396.079, 100, random.Random(5))
chk("converged first", e.state, CONVERGED)
frozen = e.offset
# Go2 reboots: offset jumps by a per-boot step plus drift.
base = 1_700_000_100.0
for i in range(50):
    s = base + i * 0.0533
    e.add(s + 1396.079 + 3.0, s)
chk("state is FAULTED after the step", e.state, FAULTED)
chk("offset is withheld once faulted", e.offset, None)
# A REAL assertion, not a tautology: the internal estimate must still hold the
# pre-step value. If the estimator had quietly re-converged onto the post-step
# delta, _offset would now be ~1399.08 and this would fail.
chk("the internal estimate never moved onto the new delta", e._offset, frozen)
chk("...and the pre-step value is the one we measured", abs(frozen - 1396.079) < 0.01, True)

print("\n6. a fault needs an EXPLICIT re-arm")
chk("still faulted without re-arm", e.state, FAULTED)
e.rearm()
chk("re-arm returns to CONVERGING", e.state, CONVERGING)
chk("and offset is still None until it re-converges", e.offset, None)

print("\n7. transient jitter does NOT fault")
e = feed(ClockOffsetEstimator(window=100, step_threshold=0.5, step_consecutive=10),
         1396.079, 100, random.Random(7))
base = 1_700_000_200.0
for i in range(200):
    s = base + i * 0.0533
    # 5 consecutive bad samples, then good -- under the 10-sample run length
    bad = (i % 40) < 5
    e.add(s + 1396.079 + (3.0 if bad else 0.01), s)
chk("still CONVERGED through intermittent outliers", e.state, CONVERGED)

print("\n8. re-stamping arithmetic")
chk("to_sec_nanosec splits cleanly", restamp.to_sec_nanosec(12.25), (12, 250000000))
chk("round-trips", restamp.to_seconds(*restamp.to_sec_nanosec(12.25)), 12.25)
sec, nsec = restamp.corrected(1_517_156_932, 198_595_549, 1396.079)
chk_close("offset applied to a real captured stamp",
          restamp.to_seconds(sec, nsec),
          1_517_156_932.198595549 + 1396.079, 1e-6)
chk("nanosec stays in range", 0 <= nsec < 1_000_000_000, True)
try:
    restamp.corrected(1, 0, None)
    chk("re-stamp with None offset raises", "no raise", "ValueError")
except ValueError:
    chk("re-stamp with None offset raises", "ValueError", "ValueError")

print()
print("FAILURES:" if fails else "ALL CLOCK-OFFSET TESTS PASS")
for f in fails:
    print("  -", f)
sys.exit(1 if fails else 0)
