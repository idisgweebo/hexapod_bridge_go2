#!/usr/bin/env python3
"""Offline test: /go2/scan must be drop-in compatible with the hexapod's /scan.

Runs with NO ROS, NO container and NO robot.

THE ORACLE IS A REAL CAPTURE, NOT A CONSTANT
--------------------------------------------
logs/ros2_scan_echo.txt is an actual `ros2 topic echo` of the hexapod's rplidar,
taken in Phase 3 of this project. This test parses THAT FILE and requires the
configured go2_adapter geometry to reproduce it field for field.

That makes "anything already written against /scan works unmodified against
/go2/scan" a machine-checked claim rather than an intention. It also means that
if the hexapod's LiDAR is ever reconfigured and re-captured, this test starts
failing instead of the claim silently going stale.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, ".."))
ORACLE = os.path.join(REPO, "logs", "ros2_scan_echo.txt")
CONFIG = os.path.join(REPO, "ros2_ws", "src", "go2_adapter", "config", "go2_adapter.yaml")

fails = []


def chk(name, got, want):
    if got != want:
        fails.append(f"{name}: got {got!r}, want {want!r}")
        print(f"  FAIL {name}: got {got!r}, want {want!r}")
    else:
        print(f"  pass {name}")


def chk_exact_float(name, got, want):
    # Exact equality on purpose. These are copied values, not computed ones; a
    # near-miss means somebody retyped instead of copying, and a retyped float
    # is how a 360-beam scan quietly becomes a 359-beam one.
    if got != want:
        fails.append(f"{name}: got {got!r}, want {want!r}")
        print(f"  FAIL {name}: got {got!r}, want {want!r}")
    else:
        print(f"  pass {name} ({got!r})")


def parse_oracle(path):
    """Pull the scalar header fields out of a `ros2 topic echo` LaserScan dump."""
    want = {"angle_min", "angle_max", "angle_increment", "range_min", "range_max"}
    out = {}
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            if line.startswith("ranges:") or line.startswith("intensities:"):
                break
            k, _, v = line.partition(":")
            k = k.strip()
            if k in want and v.strip():
                out[k] = float(v.strip())
    return out


def parse_config(path):
    """Minimal YAML scalar reader -- avoids a PyYAML dependency in a suite whose
    whole point is running on a bare host with nothing installed."""
    out = {}
    in_p2l = False
    for line in open(path, encoding="utf-8"):
        stripped = line.strip()
        if stripped.startswith("#") or not stripped:
            continue
        if stripped.startswith("pointcloud_to_laserscan_node:"):
            in_p2l = True
            continue
        if in_p2l and line[:1] not in (" ", "\t"):
            in_p2l = False
        if not in_p2l:
            continue
        k, _, v = stripped.partition(":")
        v = v.split("#")[0].strip()
        if not v:
            continue
        try:
            out[k.strip()] = float(v)
        except ValueError:
            out[k.strip()] = v.strip().strip("'\"")
    return out


print(f"oracle: {os.path.relpath(ORACLE, REPO)}")
print(f"config: {os.path.relpath(CONFIG, REPO)}")

if not os.path.exists(ORACLE):
    fails.append(f"oracle capture missing: {ORACLE}")
elif not os.path.exists(CONFIG):
    fails.append(f"config missing: {CONFIG}")
else:
    o = parse_oracle(ORACLE)
    c = parse_config(CONFIG)

    # ANTI-VACUITY: if parsing silently yielded nothing, every comparison below
    # would trivially agree on absence. Require the fields to actually be there.
    print(f"  parsed {len(o)} oracle field(s), {len(c)} config field(s)")
    for k in ("angle_min", "angle_max", "angle_increment", "range_min", "range_max"):
        if k not in o:
            fails.append(f"oracle did not yield {k} -- parse failure, not a match")
        if k not in c:
            fails.append(f"config did not yield {k} -- parse failure, not a match")

    if not fails:
        print("\n1. geometry reproduces the captured rplidar exactly")
        for k in ("angle_min", "angle_max", "angle_increment", "range_min", "range_max"):
            chk_exact_float(f"   {k}", c[k], o[k])

        print("\n2. the beam count works out to a whole 360")
        span = c["angle_max"] - c["angle_min"]
        beams = round(span / c["angle_increment"]) + 1
        chk("   360 beams", beams, 360)
        chk("   increment divides the span evenly (<0.01 beam error)",
            abs(span / c["angle_increment"] - round(span / c["angle_increment"])) < 0.01,
            True)

        print("\n3. no TF lookup is configured")
        # An empty target_frame is what stops pointcloud_to_laserscan waiting
        # forever on a transform the Go2 will never publish.
        chk("   target_frame is empty", c.get("target_frame", "MISSING"), "")

        print("\n4. the unmeasurable slab bounds are ABSENT, not guessed")
        chk("   min_height not set", "min_height" in c, False)
        chk("   max_height not set", "max_height" in c, False)

        print("\n5. scan_time is consistent with the configured throttle")
        chk("   scan_time 0.2 s == 5 Hz", c.get("scan_time"), 0.2)

print()
print("FAILURES:" if fails else "ALL SCAN-GEOMETRY TESTS PASS")
for f in fails:
    print("  -", f)
sys.exit(1 if fails else 0)
