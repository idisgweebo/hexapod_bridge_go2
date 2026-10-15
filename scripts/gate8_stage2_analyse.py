#!/usr/bin/env python3
"""Gate 8 stage 2 -- what Wi-Fi does to the hexapod's command stream. Pure: no ROS; re-runnable on a saved OUT dir.

    gate8_stage2_analyse.py OUT_DIR

Three recordings of the same stream:
  nano_echo.csv          rostopic echo ON THE NANO (localhost)  -- the published count; NANO clock (unsynced: counts only)
  laptop_ros1_echo.csv   rostopic echo on the laptop over Wi-Fi  -- laptop clock
  go2/recorder.jsonl     ROS 2 /go2/cmd_vel after ros1_bridge     -- laptop clock; also go2_cmd_bridge's requests (DRY)
A gap at BOTH laptop recordings is Wi-Fi (or the Nano); a gap only at ROS 2 is the bridge.
Plan: Claude_Research gate8_hexapod_to_go2_plan.md §6 (PG8-S2..).
"""
import csv
import json
import os
import statistics
import sys

OUT = sys.argv[1]
WD = 0.4    # go2_cmd_bridge cmd_timeout


def echo(path):
    rows = []
    try:
        for r in csv.reader(open(path)):
            if r and r[0].isdigit():
                rows.append((int(r[0]) / 1e9, float(r[1])))
    except OSError:
        pass
    return rows


def jsonl(path):
    out = []
    try:
        for line in open(path):
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    except OSError:
        pass
    return out


def gaps(ts):
    return [b - a for a, b in zip(ts, ts[1:])]


def describe(name, ts):
    g = gaps(ts)
    if not g:
        print("  %-22s n %d -- no gaps to describe" % (name, len(ts)))
        return g
    gs = sorted(g)
    p = lambda q: gs[min(len(gs) - 1, int(q * len(gs)))]
    print("  %-22s n %5d  span %7.1f s  dt median %5.1f ms  p99 %6.1f  p99.9 %6.1f  max %6.1f ms  | >150 ms %3d  >200 %3d  >300 %3d  >400 %3d"
          % (name, len(ts), ts[-1] - ts[0], statistics.median(g) * 1e3, p(0.99) * 1e3, p(0.999) * 1e3, max(g) * 1e3,
             sum(x > 0.15 for x in g), sum(x > 0.2 for x in g), sum(x > 0.3 for x in g), sum(x > WD for x in g)))
    return g


nano = echo(os.path.join(OUT, "nano_echo.csv"))
lap = echo(os.path.join(OUT, "laptop_ros1_echo.csv"))
rec = jsonl(os.path.join(OUT, "go2", "recorder.jsonl"))
r2 = [(r["t"], r["x"]) for r in rec if r.get("topic") == "/go2/cmd_vel"]
req = [r["data"] for r in rec if r.get("topic") == "/go2/cmd_bridge/requests" and isinstance(r.get("data"), dict)]
status = [r for r in rec if r.get("topic") == "/go2/cmd_bridge/status"]
moves = [q for q in req if q["name"] == "MOVE"]
stops = [q for q in req if q["name"] == "STOPMOVE"]

print("Gate 8 stage 2 -- %s" % OUT)
inst = bool(status) and any(r.get("topic") == "_recorder" for r in rec)
print("  instrument: recorder %s, go2_cmd_bridge status messages %d -> %s"
      % ("started" if any(r.get("topic") == "_recorder" for r in rec) else "NOT STARTED", len(status), "ALIVE" if inst else "⛔ DEAD"))
describe("nano (local, Nano clk)", [t for t, _ in nano])
gl = describe("laptop ROS 1 (Wi-Fi)", [t for t, _ in lap])
g2 = describe("ROS 2 after bridge", [t for t, _ in r2])
print("  counts: published (nano echo) %d -> laptop ROS 1 %d (lost %d) -> ROS 2 %d (lost after ROS 1: %d)"
      % (len(nano), len(lap), len(nano) - len(lap), len(r2), len(lap) - len(r2)))

# Gaps at ROS 2 > 0.2 s: is the same hole visible at the laptop's ROS 1 echo (= Wi-Fi) or not (= bridge)?
lt = [t for t, _ in lap]
for i, g in enumerate(g2):
    if g > 0.2:
        t0, t1 = r2[i][0], r2[i + 1][0]
        inside = [t for t in lt if t0 + 0.002 < t < t1 - 0.002]
        print("    ROS 2 gap %6.1f ms at +%7.2f s: laptop ROS 1 got %d message(s) inside it -> %s"
              % (g * 1e3, t0 - r2[0][0], len(inside), "Wi-Fi/Nano" if not inside else "BRIDGE"))

mid_stops = stops[:-1] if stops else []
print("  go2_cmd_bridge (DRY): MOVE %d, STOPMOVE %d -> mid-stream watchdog trips %d (the last STOPMOVE is the end of the stream)"
      % (len(moves), len(stops), len(mid_stops)))
for q in mid_stops:
    print("    watchdog STOPMOVE at +%.2f s" % (q["t"] - (r2[0][0] if r2 else 0)))
if r2:
    t_last = [t for t, x in r2 if x][-1] if any(x for _, x in r2) else None
    if t_last and stops:
        print("  end of stream: last Twist -> final STOPMOVE %.3f s" % (stops[-1]["t"] - t_last))
print("  PG8-S2 (>= 1 gap > 0.4 s at the ROS 2 output): %s -- %d gap(s) > 0.4 s, %d watchdog trip(s)%s"
      % ("CONFIRMED" if any(x > WD for x in g2) else "FALSIFIED", sum(x > WD for x in g2), len(mid_stops),
         "" if inst else "  ⛔ INSTRUMENT DEAD -- not scorable"))
