#!/usr/bin/env python3
"""Gate 8 stage 1 -- score a gate8_loopback_test.sh run against PG8-0..PG8-11. Pure: no ROS, re-runnable on a saved OUT dir.

    gate8_stage1_analyse.py OUT_DIR

Clocks: every timestamp here is this host's wall clock (all containers share it) -- ros1_echo %time is rostopic echo's
RECEIPT time, recorder t is the ROS 2 receipt, request t is go2_cmd_bridge's send time, fake t is the fake's receipt.
Plan: Claude_Research gate8_hexapod_to_go2_plan.md §4.
"""
import csv
import glob
import json
import os
import statistics
import sys

OUT = sys.argv[1]
results = []


def score(pid, ok, detail):
    tag = "PASS" if ok is True else ("FAIL" if ok is False else "n/a ")
    results.append((pid, tag))
    print("  %s %-7s %s" % (tag, pid, detail))


def rd(path, default=None):
    try:
        return open(path).read()
    except OSError:
        return default


def tfloat(path):
    s = rd(path)
    return float(s.strip()) if s else None


def jsonl(path):
    rows = []
    for line in (rd(path, "") or "").splitlines():
        line = line.strip()
        if line:
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass
    return rows


def echo_rows(P):
    rows = []
    for r in csv.reader((rd(os.path.join(P, "ros1_echo.csv"), "") or "").splitlines()):
        if r and r[0].isdigit():
            rows.append((int(r[0]) / 1e9, float(r[1]), float(r[6])))     # t, linear.x, angular.z
    return rows


def load(N):
    P = os.path.join(OUT, N)
    if not os.path.isdir(P):
        return None
    rec = jsonl(os.path.join(P, "recorder.jsonl"))
    d = {
        "P": P,
        "echo": echo_rows(P),
        "tw": [r for r in rec if r.get("topic") == "/go2/cmd_vel"],
        "req": [r["data"] for r in rec if r.get("topic") == "/go2/cmd_bridge/requests" and isinstance(r.get("data"), dict)],
        "fake": [r for r in jsonl(os.path.join(P, "fake.jsonl")) if r.get("kind") == "robot_req"],
        "blog": "".join(rd(f, "") for f in sorted(glob.glob(os.path.join(P, "bridge_logs", "*", "bridge.txt")))),
        "bcfg": "".join(rd(f, "") for f in sorted(glob.glob(os.path.join(P, "bridge_logs", "*", "config.txt")))),
        "node": rd(os.path.join(P, "node.txt"), ""),
        "after": rd(os.path.join(P, "master_after.txt"), ""),
        "t_kill": tfloat(os.path.join(P, "t_kill.txt")),
    }
    # INSTRUMENT CHECK: the recorder started AND heard go2_cmd_bridge's 2 Hz status. Without both, a zero below is a dead
    # instrument, not a negative (first run 15 Oct: the go2 side never started and the control "passed" on nothing).
    d["inst"] = any(r.get("topic") == "_recorder" for r in rec) and sum(r.get("topic") == "/go2/cmd_bridge/status" for r in rec) >= 4
    d["moves"] = [q for q in d["req"] if q.get("name") == "MOVE"]
    d["stops"] = [q for q in d["req"] if q.get("name") == "STOPMOVE"]
    return d


def dts(ts):
    return [b - a for a, b in zip(ts, ts[1:])]


def tail(d, before=None):
    """Last non-zero Twist at the ROS 2 recorder (optionally before t) -> first STOPMOVE after it."""
    tw = [r["t"] for r in d["tw"] if (r["x"] or r["wz"]) and (before is None or r["t"] <= before)]
    if not tw:
        return None, None, None
    t_last = tw[-1]
    st = [q for q in d["stops"] if q["t"] > t_last]
    if not st:
        return t_last, None, None
    after_moves = [q for q in d["moves"] if q["t"] > st[0]["t"]]
    return t_last, st[0]["t"] - t_last, len(after_moves)


D = {N: load(N) for N in "CDXAKR"}
for N in "CDXAK":
    if D[N] and not D[N]["inst"]:
        print("  ⛔ phase %s: INSTRUMENT DEAD (no recorder start or < 4 status messages) -- its predictions are n/a" % N)
        D[N] = None
print("Gate 8 stage 1 -- %s" % OUT)
for N, d in D.items():
    if d:
        print("  phase %s: ros1 echo %d, ros2 twists %d, MOVE %d (%s), STOPMOVE %d, fake robot_req %d"
              % (N, len(d["echo"]), len(d["tw"]), len(d["moves"]), ",".join(sorted(set(q.get("routed", "?") for q in d["moves"]))) or "-",
                 len(d["stops"]), len(d["fake"])))

# PG8-0 -- exactly one bridge, no services, param cleaned up (every phase that ran)
bad, n = [], 0
for N, d in D.items():
    if not d:
        continue
    n += 1
    # Two lines per topic in 611755fd: "Trying to create bidirectional bridge for topic 'T' ..." then the INFO
    # "create bidirectional bridge for topic T". Count the INFO confirmation (first run 15 Oct counted both -> "2").
    creates = [l for l in d["blog"].splitlines() if "]: create bidirectional bridge for topic" in l]
    svc = [l for l in d["blog"].splitlines() if "service" in l.lower() and "doesn't exist" not in l and "isn't an array" not in l]
    if not (len(creates) == 1 and creates[0].rstrip().endswith(" /go2/cmd_vel")) or svc or "gate8" in d["after"] or "deleted /gate8_bridge" not in d["bcfg"]:
        bad.append("%s: creates=%d svc=%d left=%s deleted=%s" % (N, len(creates), len(svc), "gate8" in d["after"],
                                                                  "deleted /gate8_bridge" in d["bcfg"]))
score("PG8-0", (not bad) if n else None, "%d phase(s); %s" % (n, "; ".join(bad) or "one bridge /go2/cmd_vel each, no services, param deleted"))

d = D["C"]
if d:
    score("PG8-1", len(d["echo"]) > 0 and not d["tw"] and not d["moves"],
          "control: ros1 echo %d (source ran), ros2 twists %d, MOVE %d" % (len(d["echo"]), len(d["tw"]), len(d["moves"])))
else:
    score("PG8-1", None, "phase C not run")

d = D["D"]
if d and d["tw"]:
    nz = [r for r in d["tw"] if r["x"] or r["wz"]]
    span = nz[-1]["t"] - nz[0]["t"] if len(nz) > 1 else 0.0
    exp = 10 * (span + 0.1)            # AS REGISTERED -- omits the 0.4 s watchdog tail (PV2's omission again; first run 15 Oct)
    vals = set((json.loads(q["parameter"])["x"], json.loads(q["parameter"])["z"]) for q in d["moves"])
    ok = abs(len(d["moves"]) - exp) <= 3 and vals == {(0.3, 0.0)} and all(q["routed"] == "dry" for q in d["moves"]) and not d["fake"]
    score("PG8-2", ok, "MOVE %d vs ~%.0f expected (twist span %.2f s); values %s; routed %s; fake robot_req %d"
          % (len(d["moves"]), exp, span, sorted(vals), sorted(set(q["routed"] for q in d["moves"])), len(d["fake"])))
    print("        (not a prediction: with the 0.4 s tail the expectation is ~%.0f)" % (10 * (span + 0.4)))
    t_last, tl, after = tail(d)
    score("PG8-3", tl is not None and 0.40 <= tl <= 0.51 and len(d["stops"]) == 1 and after == 0,
          "tail %s s, STOPMOVE count %d, MOVE after it %s" % ("%.3f" % tl if tl is not None else None, len(d["stops"]), after))
    e = d["echo"]
    dup_e = sum(1 for x in dts([r[0] for r in e]) if x < 0.02)
    dup_r = sum(1 for x in dts([r["t"] for r in d["tw"]]) if x < 0.02)
    score("PG8-4", len(d["tw"]) <= len(e) and dup_e == 0 and dup_r == 0,
          "ros2 twists %d <= ros1 echo %d ; dt < 20 ms: echo %d, recorder %d ; ros1 echo dt max %.3f s"
          % (len(d["tw"]), len(e), dup_e, dup_r, max(dts([r[0] for r in e]) or [0])))
    info = rd(os.path.join(d["P"], "rostopic_info.txt"), "")
    pub_s, _, sub_s = info.partition("Subscribers:")
    score("PG8-9", "/ros_bridge" in pub_s and "/ros_bridge" in sub_s and all(D[k] is None or D[k]["after"].lstrip("'").startswith("melodic")
                                                                            for k in D),
          "rostopic info: /ros_bridge pub=%s sub=%s ; /rosdistro after each phase: %s"
          % ("/ros_bridge" in pub_s, "/ros_bridge" in sub_s, {k: v["after"].splitlines()[0] if v and v["after"] else None
                                                              for k, v in D.items() if v}))
    lat = []
    for r in d["tw"]:
        prior = [x[0] for x in e if x[0] <= r["t"] + 0.002]
        if prior:
            lat.append(r["t"] - prior[-1])
    score("PG8-10", bool(lat) and statistics.median(lat) < 0.005,
          "ros1 echo receipt -> ros2 recorder receipt: median %.2f ms, max %.2f ms (n %d) ⚠️ receipt vs receipt, not publish"
          % (statistics.median(lat) * 1e3, max(lat) * 1e3, len(lat)) if lat else "no pairs")
else:
    for p in ("PG8-2", "PG8-3", "PG8-4", "PG8-9", "PG8-10"):
        score(p, False if d else None, "phase D: no Twists at the ROS 2 recorder" if d else "phase D not run")

d = D["X"]
if d:
    vals = set((json.loads(q["parameter"])["x"], json.loads(q["parameter"])["z"]) for q in d["moves"])
    score("PG8-5", bool(d["moves"]) and vals == {(0.3, 1.0)}, "MOVE %d, values %s (input 0.9 / 3.0)" % (len(d["moves"]), sorted(vals)))
else:
    score("PG8-5", None, "phase X not run")

d = D["A"]
if d:
    robot_moves = [q for q in d["moves"] if q["routed"] == "robot"]
    fm = [r for r in d["fake"] if r["api_id"] == 1008]
    fs = [r for r in d["fake"] if r["api_id"] == 1003]
    score("PG8-6", bool(fm) and len(fm) == len(robot_moves) and len(fs) == 1,
          "bridge routed robot %d, fake MOVE %d, fake STOPMOVE %d, other api_ids %s"
          % (len(robot_moves), len(fm), len(fs), sorted(set(r["api_id"] for r in d["fake"]) - {1008, 1003})))
else:
    score("PG8-6", None, "phase A not run")

d = D["K"]
if d and d["t_kill"]:
    t_last, tl, after = tail(d)
    fs = [r for r in d["fake"] if r["api_id"] == 1003]
    fm_after = [r for r in d["fake"] if r["api_id"] == 1008 and fs and r["t"] > fs[0]["t"]]
    tw_after_kill = [r for r in d["tw"] if r["t"] > d["t_kill"] + 0.05]
    score("PG8-7", tl is not None and 0.40 <= tl <= 0.51 and after == 0 and len(fs) == 1 and not fm_after and not tw_after_kill,
          "last twist %.3f s before kill; tail %s s; MOVE after STOPMOVE %s; fake STOPMOVE %d, fake MOVE after %d; twists after kill %d"
          % (d["t_kill"] - t_last if t_last else float("nan"), "%.3f" % tl if tl is not None else None, after, len(fs),
             len(fm_after), len(tw_after_kill)))
else:
    score("PG8-7", None, "phase K not run")

d = D["R"]
if d:
    e5 = [r for r in d["echo"] if abs(r[1] - 0.05) < 1e-9]
    dup = sum(1 for x in dts([r[0] for r in e5]) if x < 0.02)
    score("PG8-8", len(e5) == 5 and dup == 0, "ROS 2 pub x 0.05 x5 -> ROS 1 echo rows with x 0.05: %d (dt<20 ms: %d); other echo rows %d"
          % (len(e5), dup, len(d["echo"]) - len(e5)))
else:
    score("PG8-8", None, "phase R not run")

summ = rd(os.path.join(OUT, "summary.txt"), "") or ""
fw = [l for l in summ.splitlines() if "ip_forward=" in l]
score("PG8-11", bool(fw) and all("ip_forward=0" in l for l in fw) and "carrier=1" not in summ,
      "%d ip_forward readings, all 0: %s" % (len(fw), all("ip_forward=0" in l for l in fw)))

n_pass = sum(1 for _, t in results if t == "PASS")
n_fail = sum(1 for _, t in results if t == "FAIL")
print("STAGE 1: %d pass, %d fail, %d n/a -- %s" % (n_pass, n_fail, len(results) - n_pass - n_fail,
                                                   "fails: " + " ".join(p for p, t in results if t == "FAIL") if n_fail else "no failures"))
