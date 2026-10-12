#!/usr/bin/env python3
"""Gate 7 §10f stage T0 -- Doug drives a FAKE Go2 from his real keyboard and display. NO ROBOT.

Starts, all on loopback-only Cyclone (domain 0, `lo`, no multicast):
    the fake Go2   test_cmd_bridge_integration_rosonly.py --fake robot_only (integrates vx into x)
    the bridge     go2_cmd_bridge_node, ARMED (it can only reach the fake -- loopback, no carrier)
    the teleop     go2_teleop.py --armed, on $DISPLAY (Doug's real X server)
then waits for the teleop window to close (Q / Esc), stops the rest, and scores PT0:

    PT0  real display: >= 1 auto-repeat pair per hold (holds >= 0.6 s; taps cannot repeat),
         0 StopMoves inside holds, key-up -> StopMove <= 0.2 s
    PT7  two keys (s14): while a key is down, every Move carries its axis -- W/S keep x through
         an A/D press and release (X stops W's repeats with no release), A/D keep z
    PTH  --hold-max H (s14): a hold that outlives H ends at H (+- one 10 ms poll), then
         StopMove <= 0.2 s, and NO Move afterwards while the key is still physically held

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
        elif (kind == "key" and role in ("stop", "hold_max")) or kind in ("focus_out", "latch", "quit"):
            if down:
                cur.update(t1=t, end=role if role == "hold_max" else {"key": "space"}.get(kind, kind))
                holds.append(cur)
                cur = None
            down.clear()
    return holds


AXIS = {"w": ("x", 1), "s": ("x", -1), "a": ("z", 1), "d": ("z", -1)}
OPPOSITE = {"w": "s", "s": "w", "a": "d", "d": "a"}


def key_intervals(events):
    """Per key: (key, t_down, t_end) -- ends at its real release, or at any stop / expiry / focus loss."""
    out, down = [], {}
    for e in events:
        t, kind, key, role = float(e["t"]), e["kind"], e["key"], e["role"]
        if kind == "key" and key in MOTION_KEYS and role == "down":
            down[key] = t
        elif kind == "key" and role == "real_release" and key in down:
            out.append((key, down.pop(key), t))
        elif (kind == "key" and role in ("stop", "hold_max")) or kind in ("focus_out", "latch", "quit"):
            out += [(k, t0, t) for k, t0 in down.items()]
            down = {}
    return sorted(out, key=lambda x: x[1])


def score_combo(events, req):
    """PT7: inside each key's interval (from 0.15 s after its press, so the first Move can land),
    every Move's axis has that key's sign -- unless the opposite key overlaps (they cancel)."""
    ivs = key_intervals(events)
    moves = [(r["t"], json.loads(r["parameter"])) for r in req if r["api_id"] == 1008]
    lines, bad, n_combo = [], [], 0
    for k, t0, t1 in ivs:
        others = [o for o in ivs if o[0] != k and o[1] < t1 and o[2] > t0]
        if any(o[0] == OPPOSITE[k] for o in others):
            continue
        if not others:
            continue
        n_combo += 1
        ax, sign = AXIS[k]
        inside = [p for t, p in moves if t0 + 0.15 <= t <= t1]
        wrong = [p for p in inside if not (p[ax] * sign > 0)]
        lines.append("  %s %.2f s, overlapped by %s: %d Moves, %d without %s%s" % (
            k, t1 - t0, "+".join(sorted({o[0] for o in others})), len(inside), len(wrong), ax,
            "+" if sign > 0 else "-"))
        if wrong or not inside:
            bad.append("%s held %.2f s with %s: %d of %d Moves lost its axis" % (
                k, t1 - t0, "+".join(o[0] for o in others), len(wrong), len(inside)))
    return lines, bad, n_combo


def score(run_dir, report="t0_report.txt"):
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
    # PT7 -- two keys
    clines, cbad, n_combo = score_combo(events, req)
    out += ["", "PT7 two-key intervals scored: %d" % n_combo] + clines
    # PTH -- timed holds
    hm = [h for h in holds if h["end"] == "hold_max"]
    hmax = [e["detail"] for e in events if e["role"] == "hold_max"]
    hfails = []
    if hm:
        H = float(hmax[0])
        out += ["", "PTH --hold-max %.3f s: %d hold(s) ended by it" % (H, len(hm))]
        for h in hm:
            dur = h["t1"] - h["t0"]
            after = [s for s in stops if s > h["t1"]]
            lat = after[0] - h["t1"] if after else None
            nxt = [e for e in events if float(e["t"]) > h["t1"] and e["role"] == "down"]
            t_next = float(nxt[0]["t"]) if nxt else float("inf")
            late = [m for m in moves if (after[0] if after else h["t1"]) < m < t_next]
            out.append("  %s  %.3f s (H %+.0f ms)  StopMove %s  Moves after it before a new press: %d" % (
                "+".join(h["keys"]), dur, 1000 * (dur - H), "%.3f s" % lat if lat is not None else "none", len(late)))
            if abs(dur - H) > 0.015:
                hfails.append("hold ended %.0f ms from H" % (1000 * (dur - H)))
            if lat is None or lat > STOP_BOUND_S:
                hfails.append("hold_max -> StopMove %s" % lat)
            if late:
                hfails.append("%d Move(s) after the expiry StopMove, before a new press" % len(late))
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
    if n_combo:
        out += ["PT7: %s" % ("PASS" if not cbad else "FAIL")] + ["  - " + f for f in cbad]
    else:
        out.append("PT7: not exercised (no non-opposing key overlap)")
    if hm:
        out += ["PTH: %s" % ("PASS" if not hfails else "FAIL")] + ["  - " + f for f in hfails]
    elif hmax or "--hold-max" in " ".join(e.get("detail", "") for e in events if e["kind"] == "start"):
        out.append("PTH: not exercised (no hold outlived --hold-max)")
    allfails = fails + cbad + hfails
    text = "\n".join(out) + "\n"
    open(os.path.join(run_dir, report), "w").write(text)
    return text, not allfails


def live(out_base, teleop_extra=()):
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
                              *teleop_extra, "--out", os.path.join(run_dir, "teleop")], env=env)
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
    ap.add_argument("--hold-max", type=float, help="passed to go2_teleop.py (timed holds)")
    a = ap.parse_args()
    if a.replay:
        text, ok = score(a.replay, "t0_report_replay.txt")    # never overwrite the live report
        print(text)
        return 0 if ok else 1
    return live(a.out, ("--hold-max", str(a.hold_max)) if a.hold_max else ())


if __name__ == "__main__":
    sys.exit(main())
