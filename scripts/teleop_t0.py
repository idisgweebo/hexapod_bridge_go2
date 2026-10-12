#!/usr/bin/env python3
"""Gate 7 §10f stage T0 -- Doug drives a FAKE Go2 from his real keyboard and display. NO ROBOT.

Starts, all on loopback-only Cyclone (domain 0, `lo`, no multicast):
    the fake Go2   test_cmd_bridge_integration_rosonly.py --fake robot_only (integrates vx into x)
    the bridge     go2_cmd_bridge_node, ARMED (it can only reach the fake -- loopback, no carrier)
    the teleop     go2_teleop.py --armed, on $DISPLAY (Doug's real X server)
then waits for the teleop window to close (Q / Esc), stops the rest, and scores PT0:

    PT0  real display: >= 1 auto-repeat pair per hold (holds >= 0.6 s; taps cannot repeat),
         0 StopMoves inside holds, key-up -> StopMove <= 0.2 s

REFUSES if the Go2 cable has carrier. The fake's geofence is the bridge's 1 m (D4): drive
more than 1 m from where the bridge engaged and it ABORTS and latches -- restart this script.

    python3 teleop_t0.py --out /logs/t0          # live
    python3 teleop_t0.py --replay /logs/t0/t0_<stamp>   # re-score a saved run, no ROS
"""
import argparse
import csv
import glob
import json
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_cmd_bridge_integration_rosonly as bt     # LOOPBACK_XML, cable check, the fake

MOTION_KEYS = ("w", "s", "a", "d")
MIN_REPEAT_HOLD_S = 0.6     # first auto-repeat at ~500 ms (§10c); shorter holds are taps
STOP_BOUND_S = 0.20         # PT0
FAKE_S = 1800


def load(run_dir):
    rows = [json.loads(l) for l in open(os.path.join(run_dir, "fake.jsonl")) if l.strip()]
    evf = glob.glob(os.path.join(run_dir, "teleop", "teleop_*", "events.csv"))
    events = list(csv.DictReader(open(evf[0]))) if evf else []
    return rows, events


def holds_from(events):
    """Motion intervals: from the first motion-key 'down' to the moment no motion key is down."""
    holds, down, cur = [], set(), None
    for e in events:
        t, kind, key, role = float(e["t"]), e["kind"], e["key"], e["role"]
        if kind == "key" and key in MOTION_KEYS and role == "down":
            if not down:
                cur = {"t0": t, "keys": [], "autorepeat": 0, "end": None, "t1": None}
            down.add(key)
            cur["keys"].append(key)
        elif kind == "key" and role == "autorepeat" and cur is not None and down:
            cur["autorepeat"] += 1
        elif kind == "key" and role == "real_release" and key in down:
            down.discard(key)
            if not down:
                cur.update(t1=t, end="key-up")
                holds.append(cur)
                cur = None
        elif (kind == "key" and role == "stop") or kind in ("focus_out", "latch", "quit"):
            if down:
                cur.update(t1=t, end={"key": "space"}.get(kind, kind))
                holds.append(cur)
                cur = None
            down.clear()
    return holds


def score(run_dir):
    rows, events = load(run_dir)
    req = [r for r in rows if r["kind"] == "robot_req"]
    moves = [r["t"] for r in req if r["api_id"] == 1008]
    stops = [r["t"] for r in req if r["api_id"] == 1003]
    other = sorted({r["api_id"] for r in req} - {1008, 1003})
    holds = holds_from(events)
    out = ["T0 report  %s" % run_dir,
           "robot requests (fake): %d Move, %d StopMove, other api_ids %s" % (len(moves), len(stops), other or "none"),
           "teleop events: %d; holds: %d" % (len(events), len(holds)), ""]
    hdr = "%-3s %-7s %6s %4s %5s %8s %6s %9s  %s" % ("#", "keys", "dur_s", "rep", "moves", "maxgap_s",
                                                      "stp_in", "stop_lat", "ended by")
    out.append(hdr)
    fails, lats, n_long = [], [], 0
    for i, h in enumerate(holds, 1):
        dur = h["t1"] - h["t0"]
        mv = [m for m in moves if h["t0"] <= m <= h["t1"]]
        gaps = [b - a for a, b in zip(mv, mv[1:])]
        sin = [s for s in stops if h["t0"] < s <= h["t1"]]
        after = [s for s in stops if s > h["t1"]]
        lat = after[0] - h["t1"] if after else None
        if dur >= MIN_REPEAT_HOLD_S:
            n_long += 1
            if h["autorepeat"] < 1:
                fails.append("hold %d (%.2f s) saw no auto-repeat pair" % (i, dur))
        if sin:
            fails.append("hold %d: %d StopMove(s) inside the hold" % (i, len(sin)))
        if h["end"] == "key-up":
            lats.append(lat)
            if lat is None or lat > STOP_BOUND_S:
                fails.append("hold %d: key-up -> StopMove %s" % (i, "none" if lat is None else "%.3f s" % lat))
        out.append("%-3d %-7s %6.2f %4d %5d %8s %6d %9s  %s" % (
            i, "+".join(h["keys"]), dur, h["autorepeat"], len(mv), "%.3f" % max(gaps) if gaps else "-",
            len(sin), "%.3f" % lat if lat is not None else "-", h["end"]))
    q = [e for e in events if e["kind"] == "quit"]
    out += ["", "teleop quit: %s" % (q[0]["detail"] if q else "(no quit event)")]
    good = [l for l in lats if l is not None]
    if good:
        out.append("key-up -> StopMove: n=%d  min %.3f  median %.3f  max %.3f s"
                   % (len(good), min(good), sorted(good)[len(good) // 2], max(good)))
    if not holds:
        fails.append("no holds recorded -- nothing to score")
    if not n_long:
        fails.append("no hold >= %.1f s -- the auto-repeat leg of PT0 is untested" % MIN_REPEAT_HOLD_S)
    if not good:
        fails.append("no key-up-ended hold -- the stop-latency leg of PT0 is untested")
    out += ["", "PT0: %s" % ("PASS" if not fails else "FAIL")] + ["  - " + f for f in fails]
    text = "\n".join(out) + "\n"
    open(os.path.join(run_dir, "t0_report.txt"), "w").write(text)
    return text, not fails


def live(out_base):
    if bt.cable_has_carrier():
        print("REFUSING: %s has carrier. Unplug the Go2 cable -- T0 runs an ARMED bridge." % bt.CABLE_IFACE)
        return 2
    if not os.environ.get("DISPLAY"):
        print("REFUSING: no DISPLAY -- T0 is about Doug's real display.")
        return 2
    run_dir = os.path.join(out_base, time.strftime("t0_%Y%m%dT%H%M%SZ", time.gmtime()))
    os.makedirs(os.path.join(run_dir, "teleop"))
    xml = os.path.join(run_dir, "cyclonedds_loopback.xml")
    open(xml, "w").write(bt.LOOPBACK_XML)
    env = dict(os.environ, CYCLONEDDS_URI="file://" + xml, ROS_DOMAIN_ID="0",
               RMW_IMPLEMENTATION="rmw_cyclonedds_cpp")
    env.pop("ROS_LOCALHOST_ONLY", None)
    print("run dir %s\nloopback-only Cyclone, domain 0; %s no carrier; DISPLAY %s"
          % (run_dir, bt.CABLE_IFACE, env["DISPLAY"]))
    log = os.path.join(run_dir, "fake.jsonl")
    open(log, "w").close()
    fk = subprocess.Popen([sys.executable, os.path.join(HERE, "test_cmd_bridge_integration_rosonly.py"),
                           "--fake", "robot_only", log, str(FAKE_S)], env=env,
                          stdout=open(os.path.join(run_dir, "fake.txt"), "w"), stderr=subprocess.STDOUT,
                          start_new_session=True)
    time.sleep(2.0)
    br = subprocess.Popen(["ros2", "run", "go2_cmd_bridge", "go2_cmd_bridge_node", "--ros-args",
                           "-p", "armed:=true"], env=env,
                          stdout=open(os.path.join(run_dir, "bridge.txt"), "w"), stderr=subprocess.STDOUT,
                          start_new_session=True)
    time.sleep(3.0)
    print("\nfake Go2 + ARMED bridge up. Opening the teleop window -- drive with W/A/S/D, Space stops,"
          "\nQ or Esc quits. Keep within ~1 m of the start (the geofence aborts and latches).\n")
    rc = None
    try:
        rc = subprocess.call([sys.executable, os.path.join(HERE, "go2_teleop.py"), "--armed",
                              "--out", os.path.join(run_dir, "teleop")], env=env)
    except KeyboardInterrupt:
        rc = "interrupted"
    finally:
        for p in (br, fk):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in (br, fk):
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
    print("\nteleop rc %s\n" % rc)
    text, ok = score(run_dir)
    print(text)
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--out", help="base dir for a live run")
    g.add_argument("--replay", help="a saved t0_* run dir: re-score, no ROS")
    a = ap.parse_args()
    if a.replay:
        text, ok = score(a.replay)
        print(text)
        return 0 if ok else 1
    return live(a.out)


if __name__ == "__main__":
    sys.exit(main())
