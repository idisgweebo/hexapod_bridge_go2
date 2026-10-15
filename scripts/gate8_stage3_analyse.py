#!/usr/bin/env python3
"""Gate 8 stage 3 -- one step's recorder file (gate8_ros2_recorder.py --robot) -> what the Go2 did. Pure: no ROS.

    gate8_stage3_analyse.py STEP.jsonl [STEP2.jsonl ...]

All times are the laptop's clock (one host). Travel is from /sportmodestate.position (odom frame), rotated into the BODY
frame at the yaw of the first Move (session 12's correction: odom displacement is not body displacement).
⚠️ position/velocity are the robot's own estimates -- self-report, not ground truth (no external measurement exists).
Plan: Claude_Research gate8_hexapod_to_go2_plan.md §7.
"""
import json
import math
import sys

WD = 0.4


def load(path):
    rows = []
    for line in open(path):
        try:
            rows.append(json.loads(line))
        except ValueError:
            pass
    return rows


def body(px, py, x0, y0, yaw0):
    dx, dy = px - x0, py - y0
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    return c * dx - s * dy, s * dx + c * dy


def analyse(path):
    R = load(path)
    tw = [r for r in R if r.get("topic") == "/go2/cmd_vel"]
    req = [r["data"] for r in R if r.get("topic") == "/go2/cmd_bridge/requests" and isinstance(r.get("data"), dict)]
    st = [r for r in R if r.get("topic") == "/go2/cmd_bridge/status"]
    sp = [r for r in R if r.get("topic") == "/sportmodestate"]
    lo = [r for r in R if r.get("topic") == "/lowstate"]
    hs = [r for r in R if r.get("topic") == "/wirelesscontroller"]
    moves = [q for q in req if q.get("name") == "MOVE"]
    stops = [q for q in req if q.get("name") == "STOPMOVE"]
    other = [q for q in req if q.get("name") not in ("MOVE", "STOPMOVE")]
    print("== %s" % path)
    alive = any(r.get("topic") == "_recorder" for r in R)
    print("  instrument: recorder %s; status %d; sport %d; low %d; handset %d"
          % ("started" if alive else "NOT STARTED", len(st), len(sp), len(lo), len(hs)))
    if st:
        seq = []
        for r in st:
            k = (r["data"].get("state"), r["data"].get("armed"))
            if not seq or seq[-1][1] != k:
                seq.append((r["t"], k, r["data"].get("reason")))
        print("  bridge status: " + " -> ".join("%s%s (%s)" % (k[0], " ARMED" if k[1] else " dry", why) for _, k, why in seq))
    if not tw:
        print("  Twists: 0")
    else:
        dts = [b["t"] - a["t"] for a, b in zip(tw, tw[1:])]
        print("  Twists: %d over %.2f s; values %s; dt max %.1f ms; gaps > 0.4 s: %d"
              % (len(tw), tw[-1]["t"] - tw[0]["t"], sorted(set((r["x"], r["wz"]) for r in tw)), max(dts or [0]) * 1e3,
                 sum(d > WD for d in dts)))
    routed = sorted(set(q.get("routed") for q in moves))
    rate = (len(moves) - 1) / (moves[-1]["t"] - moves[0]["t"]) if len(moves) > 1 else 0
    print("  requests: MOVE %d (%s, %.2f Hz), STOPMOVE %d, other %s"
          % (len(moves), ",".join(map(str, routed)) or "-", rate, len(stops), [q.get("name") for q in other]))
    if tw and stops:
        last_nz = [r["t"] for r in tw if r["x"] or r["wz"]]
        if last_nz:
            after = [q for q in stops if q["t"] > last_nz[-1]]
            if after:
                print("  last Twist -> STOPMOVE %.3f s; MOVEs after the last Twist %d"
                      % (after[0]["t"] - last_nz[-1], sum(q["t"] > last_nz[-1] for q in moves)))
        mid = [q for q in stops if last_nz and q["t"] < last_nz[-1]]
        print("  mid-stream STOPMOVEs (watchdog trips): %d" % len(mid))
    if sp:
        ecs = []
        for r in sp:
            if not ecs or ecs[-1][1] != r["ec"]:
                ecs.append((r["t"], r["ec"]))
        t0 = sp[0]["t"]
        print("  error_code: " + " -> ".join("%d@+%.2f" % (e, t - t0) for t, e in ecs))
        print("  body_height %.3f -> %.3f" % (sp[0]["bh"], sp[-1]["bh"]))
        ref_t = moves[0]["t"] if moves else (tw[0]["t"] if tw else sp[0]["t"])
        s0 = min(sp, key=lambda r: abs(r["t"] - ref_t))
        end = sp[-1]
        bx, by = body(end["px"], end["py"], s0["px"], s0["py"], s0["yaw"])
        print("  travel (body frame @ first Move/Twist, to end of file): x %+.1f cm, y %+.1f cm, yaw %+.1f deg"
              % (bx * 100, by * 100, math.degrees(end["yaw"] - s0["yaw"])))
        if stops:
            ts = stops[-1]["t"]
            sS = min(sp, key=lambda r: abs(r["t"] - ts))
            ax, ay = body(sS["px"], sS["py"], s0["px"], s0["py"], s0["yaw"])
            print("  at the last STOPMOVE: x %+.1f cm; after it, to end: %+.1f cm" % (ax * 100, (bx - ax) * 100))
            # settled: first time after STOPMOVE where |v| < 0.02 m/s for 0.5 s
            post = [r for r in sp if r["t"] >= ts]
            for i, r in enumerate(post):
                win = [q for q in post[i:] if q["t"] <= r["t"] + 0.5]
                if win and win[-1]["t"] - r["t"] > 0.45 and all(math.hypot(q["vx"], q["vy"]) < 0.02 for q in win):
                    print("  settled +%.2f s after STOPMOVE" % (r["t"] - ts))
                    break
        if moves:
            sm = [r for r in sp if moves[0]["t"] <= r["t"] <= moves[-1]["t"]]
            if sm:
                print("  robot vx during Moves: mean %.3f m/s, max %.3f" % (sum(r["vx"] for r in sm) / len(sm), max(r["vx"] for r in sm)))
    if lo:
        rr = [r["temps"][6] for r in lo]
        rl = [r["temps"][9] for r in lo]
        print("  rear hips RR %d->%d (max %d), RL %d->%d (max %d) C; all-joint max %d; modes all 1: %s; power_v %.2f"
              % (rr[0], rr[-1], max(rr), rl[0], rl[-1], max(rl), max(max(r["temps"]) for r in lo),
                 all(all(m == 1 for m in r["modes"]) for r in lo), lo[-1]["power_v"]))
    for r in hs:
        if r["keys"] or max(abs(r["lx"]), abs(r["ly"]), abs(r["rx"]), abs(r["ry"])) > 0.05:
            print("  handset @+%.2f: keys %d sticks %.2f %.2f %.2f %.2f"
                  % (r["t"] - (sp[0]["t"] if sp else r["t"]), r["keys"], r["lx"], r["ly"], r["rx"], r["ry"]))


for p in sys.argv[1:]:
    analyse(p)
