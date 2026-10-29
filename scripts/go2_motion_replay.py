#!/usr/bin/env python3
"""D10 calibration: run go2_adapter's MotionDetector over recorded Go2 runs. NO ROS.

Plan: phase1_go2_feedback_plan.md D10 / PFC-11. Reads the go2_sport_probe-format
run directories (events.csv + sport.csv) and scores the detector against a ground
truth taken from the COMMAND SCHEDULE -- the robot requests actually sent -- never
from the estimator the detector itself reads.

GROUND TRUTH (registered 29 Oct, before the first scoring run)
  * robot Moves = events with kind 'sent' and api_id 1008. Dry-run rows ('dry_dry')
    are not robot requests. Bursts split at gaps > 0.5 s.
  * MOVING   from burst start + ONSET_S to the burst's last Move
  * STOPPED  before the first Move, and from SETTLE_S after a burst's last Move
             (PV4 measured settling within 1.31 s after a kill; 2.0 s covers it)
  * excluded the transitions between those, the first window_s of every run
             (detector warm-up), and everything after a YIELDED (the handset owns
             the body then; s13's yield was L2+A, a posture change)
  * a run with no robot Moves is STOPPED throughout (dry run, lying, listen)
Errors are scored PER SAMPLE, which is stricter than per 0.3 s window.

⚠️ The recordings are ~50 Hz (the recorder's decimation); the node sees
/sportmodestate at its native rate. The detector is time-based, but a faster
stream gives a noisy sample more chances to break a sustain. Inference, unmeasured.
⚠️ All s13 runs are firmware v1.1.11. Thresholds re-checked on v1.1.15 at stage 4/5.

    go2_motion_replay.py RUN_DIR [RUN_DIR ...] [--sweep] [--param k=v ...]
"""
import argparse
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "ros2_ws", "src", "go2_adapter")))
from go2_adapter import state_core as sc                          # noqa: E402

ONSET_S = 1.0
SETTLE_S = 2.0
BURST_GAP_S = 0.5


def load(run):
    with open(os.path.join(run, "sport.csv")) as f:
        sport = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    moves, yielded = [], None
    with open(os.path.join(run, "events.csv")) as f:
        for r in csv.DictReader(f):
            if r["kind"] == "sent" and r["api_id"] == "1008":
                moves.append(float(r["t"]))
            if r["kind"] == "bridge_state" and r["detail"].startswith("YIELDED") and yielded is None:
                yielded = float(r["t"])
    return sport, moves, yielded


def bursts(moves):
    out = []
    for t in moves:
        if out and t - out[-1][1] <= BURST_GAP_S:
            out[-1][1] = t
        else:
            out.append([t, t])
    return out


def truth(t, t_start, bs, yielded, window_s):
    if t < t_start + window_s or (yielded is not None and t >= yielded):
        return None
    for b0, b1 in bs:
        if b0 + ONSET_S <= t <= b1:
            return sc.MOVING
        if b0 <= t < b1 + SETTLE_S:
            return None
    return sc.STOPPED


def score(run, params):
    sport, moves, yielded = load(run)
    bs = bursts(moves)
    det = sc.MotionDetector(sc.validate_params(params))
    t0 = sport[0]["t"]
    res = {"n": 0, "err": 0, "excluded": 0, "errors": [], "rest_max": [0.0] * 4, "move_min": [math.inf] * 4,
           "onsets": [], "stops": []}
    states = []
    for r in sport:
        st = det.update(r["t"], r["vx"], r["vy"], r["yaw_speed"], r["px"], r["py"], r["yaw"])
        states.append((r["t"], st))
        tr = truth(r["t"], t0, bs, yielded, params.get("window_s", sc.DEFAULTS["window_s"]))
        if tr is None:
            res["excluded"] += 1
            continue
        res["n"] += 1
        d = det.detail
        feats = (d["speed"], abs(d["yaw_speed"]), d["trans"], d["turn"])
        if tr == sc.STOPPED:
            res["rest_max"] = [max(a, b) for a, b in zip(res["rest_max"], feats)]
        else:
            res["move_min"] = [min(a, b) for a, b in zip(res["move_min"], feats)]
        if st != tr:
            res["err"] += 1
            if len(res["errors"]) < 5:
                res["errors"].append((round(r["t"] - t0, 2), tr, st))
    for b0, b1 in bs:
        on = next((t for t, s in states if t >= b0 and s == sc.MOVING), None)
        off = next((t for t, s in states if t > b1 and s == sc.STOPPED), None)
        res["onsets"].append(None if on is None else round(on - b0, 2))
        res["stops"].append(None if off is None else round(off - b1, 2))
    res["bursts"], res["yielded"] = len(bs), yielded is not None
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--sweep", action="store_true", help="grid over the four thresholds")
    ap.add_argument("--param", action="append", default=[], help="k=v override of a state_core DEFAULT")
    a = ap.parse_args()
    params = dict(sc.DEFAULTS)
    for kv in a.param:
        k, v = kv.split("=")
        params[k] = float(v)
    sc.validate_params(params)

    print("params: " + " ".join("%s=%s" % (k, params[k]) for k in ("v_mov", "w_mov", "d_mov", "a_mov", "window_s")))
    print("truth: onset %.1f s, settle %.1f s, burst gap %.1f s; scored per sample" % (ONSET_S, SETTLE_S, BURST_GAP_S))
    print()
    print("%-38s %6s %5s %5s %4s  %-26s %-14s %s" % ("run", "scored", "excl", "ERR", "bur", "onset s / stop s", "yielded", "first errors"))
    total_err = 0
    rest_max, move_min = [0.0] * 4, [math.inf] * 4
    for run in a.runs:
        r = score(run, params)
        total_err += r["err"]
        rest_max = [max(x, y) for x, y in zip(rest_max, r["rest_max"])]
        move_min = [min(x, y) for x, y in zip(move_min, r["move_min"])]
        print("%-38s %6d %5d %5d %4d  %-26s %-14s %s" % (
            os.path.basename(run.rstrip("/")), r["n"], r["excluded"], r["err"], r["bursts"],
            "%s / %s" % (r["onsets"], r["stops"]), r["yielded"], r["errors"]))
    print()
    names = ("speed m/s", "|yaw_speed| rad/s", "trans/window m", "turn/window rad")
    thr = (params["v_mov"], params["w_mov"], params["d_mov"], params["a_mov"])
    print("%-20s %12s %12s %12s" % ("feature", "rest MAX", "threshold", "moving MIN"))
    for n, rm, t, mm in zip(names, rest_max, thr, move_min):
        print("%-20s %12.4f %12.4f %12.4f" % (n, rm, t, mm))
    print("  (moving MIN is over all MOVING-truth samples, walks AND turns: a turn has ~0 speed and a walk ~0 turn,"
          " so a low MIN on one axis is expected. The pairing is what has to separate them.)")
    print()
    print("TOTAL misclassified samples outside transitions: %d" % total_err)

    if a.sweep:
        print()
        print("SWEEP: zero-error combinations (v_mov w_mov d_mov a_mov), each value x {0.5, 1, 2, 4} of the default")
        ok = []
        for fv in (0.5, 1, 2, 4):
            for fw in (0.5, 1, 2, 4):
                for fd in (0.5, 1, 2, 4):
                    for fa in (0.5, 1, 2, 4):
                        p = dict(params, v_mov=sc.DEFAULTS["v_mov"] * fv, w_mov=sc.DEFAULTS["w_mov"] * fw,
                                 d_mov=sc.DEFAULTS["d_mov"] * fd, a_mov=sc.DEFAULTS["a_mov"] * fa)
                        if sum(score(run, p)["err"] for run in a.runs) == 0:
                            ok.append((fv, fw, fd, fa))
        print("  %d of 256 combinations have zero errors" % len(ok))
        for axis, i in (("v_mov", 0), ("w_mov", 1), ("d_mov", 2), ("a_mov", 3)):
            print("  %-6s factors present in a zero-error combination: %s" % (axis, sorted({c[i] for c in ok})))
        print("  default (1,1,1,1) zero-error: %s" % ((1, 1, 1, 1) in ok))
    return 1 if total_err else 0


if __name__ == "__main__":
    sys.exit(main())
