#!/usr/bin/env python3
"""Gate 6 -- the FIRST script in this project that publishes to the Go2.

WHAT IT DOES
------------
Sends a small, fixed set of high-level sport requests on /api/sport/request and
records what the robot does about them. Plan: gate6_go2_motion_plan.md.

    stage       robot before -> after   publishes
    listen      any                     NOTHING (no publisher is even created)
    query       lying                   one AUTORECOVERY_GET (2055) -- a getter
    stand       lying -> lying          StandUp, then StandDown
    move_once   lying -> lying          StandUp, BalanceStand, ONE Move, StopMove at +2 s, StandDown
    move_set    lying -> lying          StandUp, BalanceStand, four 1.5 s Moves at 10 Hz, StandDown
    stop        any                     StopMove
    lie         any -> lying            StopMove, StandDown

Every motion stage starts lying and ends lying, so standing time per stage is
seconds, not minutes (hard rule 7: rear hips, 50 C ceiling).

WHY A SCRIPT AND NOT `ros2 topic pub`
-------------------------------------
Hard rule 6 and Gate 4's standing rule 2: no hand-typed velocities. On the
hexapod, cmd_guard clamps a /cmd_vel stream. This file plays that role for the
Go2 -- ⚠️ it is NOT cmd_guard, and the deviation is recorded in the plan.

  * ALLOWLIST. Only the six api_ids in API can be built into a request. Damp
    (1001) is deliberately absent: the handset's L2+B is the damping e-stop, and
    a script must never be able to collapse a standing robot.
  * CLAMPS. Move is limited to |vx| <= 0.2 m/s, vy == 0, |vyaw| <= 0.3 rad/s. A
    value outside the clamp RAISES -- it is never silently clipped, because every
    value here is baked into a plan, so an out-of-range one is a code error.
  * DRY RUN BY DEFAULT. Without --armed a publishing stage prints its plan and
    exits, without importing ROS.
  * ONE PUBLISHER, ONE TOPIC, LITERAL. Enforced by test_sport_probe_offline.py.

RUNTIME GUARDS (live)
---------------------
Preflight, before the publisher exists: ROS_DOMAIN_ID exported as 0, Cyclone
RMW, /sportmodestate and /lowstate both arriving, posture as the stage expects,
rear hips under START_MAX_C, handset idle. After the publisher exists: it must
MATCH at least one subscriber (the sport server) or nothing is sent -- an
unmatched publish is indistinguishable from a refused one (U5).

During the stage, any of these aborts it:
  * rear hip >= 50 C, |v| > SPEED_ABORT or |yaw_speed| > YAW_ABORT SUSTAINED for
    SUSTAIN_S (a single-sample spike is an estimator transient -- session 12),
    displacement > RADIUS_ABORT (the cable), a posture wait timing out,
    Ctrl+C, an exception
    -> StopMove, then StandDown if motion was commanded.
  * /sportmodestate silent > STALE_S
    -> StopMove is SENT, but ⚠️ a link that is down cannot carry it. That is why
       U2 (does one Move persist?) is measured before any streaming.
  * HANDSET ACTIVITY -> YIELD. The script publishes NOTHING more and exits. The
    human has taken over; a StopMove landing after an L2+B could fight it.

DATA REACHES DISK AS IT ARRIVES (events.csv, sport.csv, low.csv, handset.csv),
and --replay DIR re-analyses a run with no ROS at all.
"""

import argparse
import csv
import json
import math
import os
import sys
import time

# Pure -- go2_thermal_watch imports rclpy only inside its own main().
from go2_thermal_watch import classify, RR_HIP, RL_HIP, CEILING, JOINT

# ---------------------------------------------------------------------------
# The only topic this file may publish to. test_sport_probe_offline.py checks
# that create_publisher appears exactly once, with exactly this literal.
# ---------------------------------------------------------------------------
REQUEST_TOPIC = "/api/sport/request"
RESPONSE_TOPIC = "/api/sport/response"

# From unitree_ros2 @ 668d1ec5, example/src/include/common/ros2_sport_client.h.
# Vendor source, not measured on this robot.
API = {
    "AUTORECOVERY_GET": 2055,
    "STOPMOVE": 1003,
    "STANDUP": 1004,
    "STANDDOWN": 1005,
    "BALANCESTAND": 1002,
    "MOVE": 1008,
}
ALLOWED_API_IDS = frozenset(API.values())
API_NAME = {v: k for k, v in API.items()}

# Raised 0.2 -> 0.3 on 7 Oct (session 12), with Doug's approval: streamed Moves at 0.1 m/s
# produced a LEAN, not a gait (move_set: +1.3 cm in 1.4 s), and Doug reports the handset
# also leans until the stick is pushed further. 0.3 is the vendor example's own value.
VX_MAX = 0.3         # m/s
VY_MAX = 0.0         # m/s   -- no strafing in this gate
VYAW_MAX = 0.3       # rad/s
MOVE_RATE_HZ = 10.0

# Posture, from SportModeState.body_height. Lying measured 0.0715 m (s5).
# Standing is PREDICTED 0.25-0.35 m (PM2) -- these thresholds sit well clear of both.
LYING_MAX_M = 0.12
STANDING_MIN_M = 0.20
POSTURE_TIMEOUT_S = 8.0

START_MAX_C = 45.0     # refuse to START a standing stage above this (rear hips)
SPEED_ABORT = 2 * VX_MAX   # m/s   -- 2x the clamp
YAW_ABORT = 0.6        # rad/s -- 2x the clamp
RADIUS_ABORT = 1.0     # m     -- the cable lies behind the robot
# Session 12, first `stand`: StopMove landing on a stand-locked robot produced ONE
# SportModeState sample at 0.637 m/s while position moved 7.6 mm and settled back --
# a mode-transition transient in the velocity ESTIMATE, not motion. A single-sample
# guard aborted on it. The excess must now persist for SUSTAIN_S; at 0.4 m/s that is
# 4 cm of travel, and RADIUS_ABORT still backs it up.
SUSTAIN_S = 0.1
STALE_S = 0.5          # /sportmodestate is 300 Hz; 0.5 s silent is 150 lost samples
HANDSET_STICK = 0.2    # |stick| above this, or any key, is a human at the controls
MATCH_TIMEOUT_S = 3.0
PREFLIGHT_S = 2.0

POSTURE_PRE = {"query": "lying", "stand": "lying", "move_once": "lying",
               "move_set": "lying", "walk_02": "lying", "walk_03": "lying",
               "listen": None, "stop": None, "lie": None}
STANDING_STAGES = {"stand", "move_once", "move_set", "walk_02", "walk_03"}


class RequestRejected(ValueError):
    pass


def build_request(name, **params):
    """Return (api_id, parameter_json). Pure: no ROS. The ONLY way a request is made.

    Raises RequestRejected for any api outside the allowlist or any Move outside
    the clamp. Never clips."""
    if name not in API:
        raise RequestRejected(f"{name!r} is not in the allowlist {sorted(API)}")
    api_id = API[name]
    if api_id not in ALLOWED_API_IDS:          # belt and braces against an edited API dict
        raise RequestRejected(f"api_id {api_id} not allowed")
    if name == "MOVE":
        if set(params) != {"vx", "vy", "vyaw"}:
            raise RequestRejected(f"MOVE needs exactly vx, vy, vyaw; got {sorted(params)}")
        vx, vy, vyaw = (float(params[k]) for k in ("vx", "vy", "vyaw"))
        for label, v, lim in (("vx", vx, VX_MAX), ("vy", vy, VY_MAX), ("vyaw", vyaw, VYAW_MAX)):
            if not math.isfinite(v) or abs(v) > lim + 1e-9:
                raise RequestRejected(f"MOVE {label}={v} outside clamp +/-{lim}")
        # Vendor keys: x, y, z  (ros2_sport_client.cpp, SportClient::Move)
        return api_id, json.dumps({"x": vx, "y": vy, "z": vyaw})
    if params:
        raise RequestRejected(f"{name} takes no parameters; got {sorted(params)}")
    return api_id, ""


# ---------------------------------------------------------------------------
# Plans. Steps:
#   ("send", NAME, params)        one request
#   ("wait", seconds)
#   ("until", "standing"|"lying") wait for posture, abort on POSTURE_TIMEOUT_S
#   ("stream", vx, vy, vyaw, s)   MOVE at MOVE_RATE_HZ for s seconds
# ---------------------------------------------------------------------------
def _stand_wrap(inner):
    return ([("wait", 2.0), ("send", "STANDUP", {}), ("until", "standing"), ("wait", 2.0)]
            + inner
            + [("send", "STOPMOVE", {}), ("wait", 1.0),
               ("send", "STANDDOWN", {}), ("until", "lying"), ("wait", 2.0)])


PLANS = {
    "listen": [("wait", 20.0)],
    "query": [("wait", 3.0), ("send", "AUTORECOVERY_GET", {}), ("wait", 5.0)],
    "stand": _stand_wrap([("wait", 3.0)]),
    "move_once": _stand_wrap([
        ("send", "BALANCESTAND", {}), ("wait", 2.0),
        ("send", "MOVE", {"vx": 0.1, "vy": 0.0, "vyaw": 0.0}),   # ONCE -- U2 / PM3
        ("wait", 2.0),
        ("send", "STOPMOVE", {}), ("wait", 3.0),
    ]),
    "move_set": _stand_wrap([
        ("send", "BALANCESTAND", {}), ("wait", 2.0),
        ("stream", 0.1, 0.0, 0.0, 1.5), ("send", "STOPMOVE", {}), ("wait", 2.0),
        ("stream", -0.1, 0.0, 0.0, 1.5), ("send", "STOPMOVE", {}), ("wait", 2.0),
        ("stream", 0.0, 0.0, 0.3, 1.5), ("send", "STOPMOVE", {}), ("wait", 2.0),
        ("stream", 0.0, 0.0, -0.3, 1.5), ("send", "STOPMOVE", {}), ("wait", 2.0),
    ]),
    # Forward only: a walk is up to ~60-75 cm, and the cable lies behind the robot.
    "walk_02": _stand_wrap([
        ("send", "BALANCESTAND", {}), ("wait", 2.0),
        ("stream", 0.2, 0.0, 0.0, 3.0), ("send", "STOPMOVE", {}), ("wait", 2.0),
    ]),
    "walk_03": _stand_wrap([
        ("send", "BALANCESTAND", {}), ("wait", 2.0),
        ("stream", 0.3, 0.0, 0.0, 2.5), ("send", "STOPMOVE", {}), ("wait", 2.0),
    ]),
    "stop": [("wait", 1.0), ("send", "STOPMOVE", {}), ("wait", 2.0)],
    "lie": [("wait", 1.0), ("send", "STOPMOVE", {}), ("wait", 1.0),
            ("send", "STANDDOWN", {}), ("until", "lying"), ("wait", 2.0)],
}


def plan_requests(plan):
    """Expand a plan into the (api_id, parameter) it would send. Validates every
    request through build_request, so a bad baked value fails offline."""
    out = []
    for step in plan:
        if step[0] == "send":
            out.append(build_request(step[1], **step[2]))
        elif step[0] == "stream":
            _, vx, vy, vyaw, secs = step
            n = int(round(secs * MOVE_RATE_HZ))
            req = build_request("MOVE", vx=vx, vy=vy, vyaw=vyaw)
            out.extend([req] * n)
    return out


def stage_publishes(stage):
    return any(s[0] in ("send", "stream") for s in PLANS[stage])


def describe(stage):
    lines = [f"stage {stage!r}: posture before = {POSTURE_PRE[stage] or 'any'}"]
    t = 0.0
    for s in PLANS[stage]:
        if s[0] == "wait":
            lines.append(f"  t~{t:5.1f}  wait {s[1]:.1f} s"); t += s[1]
        elif s[0] == "until":
            lines.append(f"  t~{t:5.1f}  wait until {s[1]} (<= {POSTURE_TIMEOUT_S:.0f} s)")
        elif s[0] == "send":
            api_id, p = build_request(s[1], **s[2])
            lines.append(f"  t~{t:5.1f}  SEND {s[1]} ({api_id}) {p}")
        elif s[0] == "stream":
            api_id, p = build_request("MOVE", vx=s[1], vy=s[2], vyaw=s[3])
            lines.append(f"  t~{t:5.1f}  STREAM MOVE ({api_id}) {p} at {MOVE_RATE_HZ:.0f} Hz "
                         f"for {s[4]:.1f} s ({int(round(s[4]*MOVE_RATE_HZ))} msgs)")
            t += s[4]
    lines.append(f"  {len(plan_requests(PLANS[stage]))} request(s) in total")
    return "\n".join(lines)


def posture_of(body_height):
    if body_height is None:
        return None
    if body_height <= LYING_MAX_M:
        return "lying"
    if body_height >= STANDING_MIN_M:
        return "standing"
    return "between"


def handset_active(lx, ly, rx, ry, keys):
    return keys != 0 or max(abs(lx), abs(ly), abs(rx), abs(ry)) > HANDSET_STICK


class SustainedGuard:
    """Trips only when |value| > limit continuously for sustain_s. Pure."""

    def __init__(self, limit, sustain_s=SUSTAIN_S):
        self.limit, self.sustain_s, self.since = limit, sustain_s, None

    def update(self, t, value):
        if abs(value) > self.limit:
            if self.since is None:
                self.since = t
            return t - self.since >= self.sustain_s
        self.since = None
        return False


def check_env(env):
    """Return a list of problems. CLAUDE.md: a silent domain mismatch looks exactly
    like a refusal, so ROS_DOMAIN_ID must be EXPORTED as 0, never defaulted."""
    probs = []
    if env.get("ROS_DOMAIN_ID") != "0":
        probs.append(f"ROS_DOMAIN_ID must be exported as '0' (got {env.get('ROS_DOMAIN_ID')!r})")
    if env.get("RMW_IMPLEMENTATION") != "rmw_cyclonedds_cpp":
        probs.append(f"RMW_IMPLEMENTATION must be rmw_cyclonedds_cpp "
                     f"(got {env.get('RMW_IMPLEMENTATION')!r})")
    if env.get("ROS_LOCALHOST_ONLY", "0") not in ("", "0"):
        probs.append("ROS_LOCALHOST_ONLY is set -- the robot would be invisible")
    return probs


# ---------------------------------------------------------------------------
# Analysis -- pure, used live and by --replay.
# ---------------------------------------------------------------------------
def _read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def _f(row, k):
    try:
        return float(row[k])
    except (KeyError, TypeError, ValueError):
        return None


def analyse(run_dir):
    """Report on a run directory. Says 'NO CONCLUSION' rather than guess."""
    ev = _read_csv(os.path.join(run_dir, "events.csv"))
    sp = _read_csv(os.path.join(run_dir, "sport.csv"))
    lo = _read_csv(os.path.join(run_dir, "low.csv"))
    hs = _read_csv(os.path.join(run_dir, "handset.csv"))
    out = [f"run: {run_dir}",
           f"rows: events {len(ev)}, sport {len(sp)}, low {len(lo)}, handset {len(hs)}"]

    sent = [e for e in ev if e["kind"] == "sent"]
    resp = [e for e in ev if e["kind"] == "response"]
    aborts = [e for e in ev if e["kind"] in ("abort", "yield", "refuse")]
    out.append(f"requests sent: {len(sent)}   responses seen: {len(resp)}")
    for e in aborts:
        out.append(f"  ⚠️ {e['kind'].upper()} at t={float(e['t']):.3f}: {e['detail']}")

    # Responses to OUR requests: first response with the same api_id after each send.
    out.append("")
    out.append("PM0 -- responses to our requests (first same-api_id reply within 1 s):")
    seen_ids = set()
    for e in sent:
        aid = e["api_id"]
        if aid in seen_ids and API_NAME.get(int(aid)) == "MOVE":
            continue                      # don't list every streamed Move
        seen_ids.add(aid)
        t0 = float(e["t"])
        match = [r for r in resp if r["api_id"] == aid and 0 <= float(r["t"]) - t0 <= 1.0]
        if match:
            r = match[0]
            out.append(f"  {API_NAME.get(int(aid), aid):17s} code={r['code']:>5s}  "
                       f"latency={float(r['t']) - t0:.3f} s  data={r['detail'][:60]!r}")
        else:
            out.append(f"  {API_NAME.get(int(aid), aid):17s} NO RESPONSE within 1 s")
    if resp and not sent:
        out.append("  (responses seen with nothing sent: other clients' traffic -- see events.csv)")

    # PM1 -- no joint motion around the query
    q_cols = [f"q_{n}" for n in JOINT]
    q_sent = [e for e in sent if API_NAME.get(int(e["api_id"])) == "AUTORECOVERY_GET"]
    if q_sent and lo:
        t0 = float(q_sent[0]["t"])
        win = [r for r in lo if abs(float(r["t"]) - t0) <= 2.5]
        if len(win) >= 2:
            dq = max(max(_f(r, c) for r in win) - min(_f(r, c) for r in win) for c in q_cols)
            out.append(f"\nPM1 -- max |dq| over +/-2.5 s of the query: {dq:.6f} rad "
                       f"({len(win)} samples) -> {'PASS' if dq < 0.001 else 'FAIL'} (< 0.001)")
        else:
            out.append("\nPM1 -- NO CONCLUSION: no /lowstate samples around the query")

    # PM2 -- body height
    bh = [(float(r["t"]), _f(r, "body_height")) for r in sp if _f(r, "body_height") is not None]
    if bh:
        hs_ = [b for _, b in bh]
        out.append(f"\nbody_height: first {hs_[0]:.4f}  min {min(hs_):.4f}  max {max(hs_):.4f}  "
                   f"last {hs_[-1]:.4f} m")
        if any(API_NAME.get(int(e["api_id"])) == "STANDUP" for e in sent):
            ok = 0.25 <= max(hs_) <= 0.35
            out.append(f"PM2 -- standing height {max(hs_):.4f} m vs predicted 0.25-0.35 -> "
                       f"{'PASS' if ok else 'FAIL'}")

    # PM3 -- does one Move persist? Only meaningful for move_once.
    moves = [e for e in sent if API_NAME.get(int(e["api_id"])) == "MOVE"]
    if len(moves) == 1 and sp:
        tm = float(moves[0]["t"])
        stops = [float(e["t"]) for e in sent
                 if API_NAME.get(int(e["api_id"])) == "STOPMOVE" and float(e["t"]) > tm]
        ts = stops[0] if stops else tm + 2.0
        seg = [(float(r["t"]) - tm, _speed(r)) for r in sp if tm <= float(r["t"]) <= ts]
        if seg:
            peak = max(v for _, v in seg)
            started = [t for t, v in seg if v > 0.03]
            if not started:
                out.append(f"\nPM3 -- NO CONCLUSION: the robot never moved (peak {peak:.3f} m/s). "
                           f"The Move was ignored or refused -- check its response code.")
            else:
                t_on = started[0]
                after = [(t, v) for t, v in seg if t > t_on]
                stopped = [t for t, v in after if v < 0.02]
                if stopped:
                    out.append(f"\nPM3 -- moved at +{t_on:.2f} s, peak {peak:.3f} m/s, back under "
                               f"0.02 m/s at +{stopped[0]:.2f} s with NO new command "
                               f"-> Move does NOT persist (PM3 {'PASS' if stopped[0] <= t_on + 1.0 else 'FAIL: >1 s'})")
                else:
                    out.append(f"\nPM3 -- moved at +{t_on:.2f} s, peak {peak:.3f} m/s, still moving "
                               f"when StopMove was sent at +{ts - tm:.2f} s -> ⛔ Move PERSISTS (PM3 FAIL)")

    # PM4 -- signs during streamed Moves. ⚠️ velocity frame is unknown (body or world);
    # both are reported, the body-frame figure uses SportModeState's own yaw.
    if len(moves) > 1 and sp:   # streamed: move_set, walk_*
        out.append("\nPM4 -- sign of measured motion during each streamed Move "
                   "(⚠️ velocity frame unverified):")
        segs = _segments(moves)
        for (t_a, t_b, param) in segs:
            rows = [r for r in sp if t_a <= float(r["t"]) <= t_b]
            if not rows:
                out.append(f"  {param}: NO DATA"); continue
            vx_w = sum(_f(r, "vx") for r in rows) / len(rows)
            vbx = sum(_body_vx(r) for r in rows) / len(rows)
            wz = sum(_f(r, "yaw_speed") for r in rows) / len(rows)
            ext = [r for r in sp if t_a <= float(r["t"]) <= t_b + 1.5]
            dx = _f(ext[-1], "px") - _f(ext[0], "px")
            dy = _f(ext[-1], "py") - _f(ext[0], "py")
            dyaw = math.degrees(_f(ext[-1], "yaw") - _f(ext[0], "yaw"))
            lw = [r for r in lo if t_a <= float(r["t"]) <= t_b + 1.5]
            calf = (max(max(_f(r, f"q_{j}") for r in lw) - min(_f(r, f"q_{j}") for r in lw)
                        for j in JOINT if j.endswith("calf")) if lw else float("nan"))
            out.append(f"  {param}: mean vx(raw) {vx_w:+.3f}  vx(body) {vbx:+.3f}  "
                       f"yaw_speed {wz:+.3f}  (n={len(rows)})")
            out.append(f"      displacement {math.hypot(dx, dy) * 100:.1f} cm (dx {dx * 100:+.1f}, "
                       f"dy {dy * 100:+.1f}), dyaw {dyaw:+.1f} deg, max calf range {calf:.3f} rad "
                       f"-> {'STEPPING' if calf > 0.3 else 'no stepping (lean)'} (inference: >0.3 rad)")

    # Thermal
    if lo:
        rr = [_f(r, f"temp_{JOINT[RR_HIP]}") for r in lo]
        rl = [_f(r, f"temp_{JOINT[RL_HIP]}") for r in lo]
        rr = [x for x in rr if x is not None]; rl = [x for x in rl if x is not None]
        if rr and rl:
            out.append(f"\nrear hips: RR {rr[0]:.0f} -> {rr[-1]:.0f} (max {max(rr):.0f}) C, "
                       f"RL {rl[0]:.0f} -> {rl[-1]:.0f} (max {max(rl):.0f}) C")

    if hs:
        act = [r for r in hs if handset_active(*(float(r[k]) for k in ("lx", "ly", "rx", "ry")),
                                               int(r["keys"]))]
        out.append(f"handset: {len(hs)} msgs, {len(act)} active")
    return "\n".join(out)


def _speed(r):
    vx, vy = _f(r, "vx") or 0.0, _f(r, "vy") or 0.0
    return math.hypot(vx, vy)


def _body_vx(r):
    yaw = _f(r, "yaw") or 0.0
    return math.cos(yaw) * (_f(r, "vx") or 0.0) + math.sin(yaw) * (_f(r, "vy") or 0.0)


def _segments(moves):
    """Group streamed Moves into contiguous segments: (t_first, t_last, parameter)."""
    segs = []
    for e in moves:
        t = float(e["t"])
        if segs and segs[-1][2] == e["detail"] and t - segs[-1][1] <= 3.0 / MOVE_RATE_HZ:
            segs[-1] = (segs[-1][0], t, segs[-1][2])
        else:
            segs.append((t, t, e["detail"]))
    return segs


# ---------------------------------------------------------------------------
# Live -- the only part that needs ROS. Imports are local so everything above
# runs, and is tested, on a host with no ROS at all.
# ---------------------------------------------------------------------------
class Abort(Exception):
    pass


class Yield(Exception):
    pass


class Refuse(Exception):
    pass


def run_live(args):
    import rclpy
    from rclpy.node import Node
    from unitree_go.msg import SportModeState, LowState, WirelessController
    from unitree_api.msg import Response
    publishing = stage_publishes(args.stage)
    if publishing:
        from unitree_api.msg import Request

    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    run_dir = os.path.join(args.out, f"sport_{args.stage}_{stamp}")
    os.makedirs(run_dir, exist_ok=True)
    print(f"run dir: {run_dir}")
    print(describe(args.stage))

    files = {}

    def writer(name, head):
        fh = open(os.path.join(run_dir, name), "w", newline="")
        w = csv.writer(fh); w.writerow(head); files[name] = fh
        return w

    w_ev = writer("events.csv", ["t", "utc", "kind", "api_id", "code", "detail"])
    w_sp = writer("sport.csv", ["t", "mode", "gait_type", "error_code", "body_height",
                                "px", "py", "pz", "vx", "vy", "vz", "yaw_speed", "yaw"])
    w_lo = writer("low.csv", ["t"] + [f"q_{n}" for n in JOINT] + [f"temp_{n}" for n in JOINT])
    w_hs = writer("handset.csv", ["t", "lx", "ly", "rx", "ry", "keys"])

    def event(kind, api_id="", code="", detail=""):
        t = time.time()
        w_ev.writerow([f"{t:.6f}", time.strftime("%H:%M:%S", time.gmtime(t)), kind,
                       api_id, code, detail])
        files["events.csv"].flush()
        print(f"[{time.strftime('%H:%M:%SZ', time.gmtime(t))}] {kind:9s} {api_id} {code} {detail}")

    st = {"sport_t": None, "low_t": None, "bh": None, "px0": None, "py0": None,
          "px": None, "py": None, "speed": 0.0, "wz": 0.0, "rr": None, "rl": None,
          "handset": False, "n_sport": 0, "n_low": 0, "n_hs": 0,
          "speed_trip": False, "yaw_trip": False}
    g_speed, g_yaw = SustainedGuard(SPEED_ABORT), SustainedGuard(YAW_ABORT)

    class Probe(Node):
        def __init__(self):
            super().__init__("go2_sport_probe")
            self.create_subscription(SportModeState, "/sportmodestate", self.on_sport, 10)
            self.create_subscription(LowState, "/lowstate", self.on_low, 10)
            self.create_subscription(WirelessController, "/wirelesscontroller", self.on_hs, 10)
            self.create_subscription(Response, RESPONSE_TOPIC, self.on_resp, 10)
            self.pub = None

        def make_publisher(self):
            # ⛔ The single publisher in this file. Literal topic; see the offline test.
            self.pub = self.create_publisher(Request, "/api/sport/request", 10)

        def on_sport(self, m):
            t = time.time(); st["sport_t"] = t; st["n_sport"] += 1
            st["bh"] = float(m.body_height)
            st["px"], st["py"] = float(m.position[0]), float(m.position[1])
            if st["px0"] is None:
                st["px0"], st["py0"] = st["px"], st["py"]
            st["speed"] = math.hypot(m.velocity[0], m.velocity[1])
            st["wz"] = float(m.yaw_speed)
            st["speed_trip"] = st["speed_trip"] or g_speed.update(t, st["speed"])
            st["yaw_trip"] = st["yaw_trip"] or g_yaw.update(t, st["wz"])
            if st["n_sport"] % 6 == 0:                       # 300 Hz -> 50 Hz on disk
                w_sp.writerow([f"{t:.6f}", m.mode, m.gait_type, m.error_code,
                               f"{m.body_height:.5f}", *(f"{x:.5f}" for x in m.position),
                               *(f"{x:.5f}" for x in m.velocity), f"{m.yaw_speed:.5f}",
                               f"{m.imu_state.rpy[2]:.6f}"])

        def on_low(self, m):
            t = time.time(); st["low_t"] = t; st["n_low"] += 1
            st["rr"] = m.motor_state[RR_HIP].temperature
            st["rl"] = m.motor_state[RL_HIP].temperature
            if st["n_low"] % 10 == 0:                        # 500 Hz -> 50 Hz on disk
                w_lo.writerow([f"{t:.6f}"] + [f"{m.motor_state[i].q:.6f}" for i in range(12)]
                              + [m.motor_state[i].temperature for i in range(12)])

        def on_hs(self, m):
            t = time.time(); st["n_hs"] += 1
            w_hs.writerow([f"{t:.6f}", f"{m.lx:.3f}", f"{m.ly:.3f}", f"{m.rx:.3f}",
                           f"{m.ry:.3f}", m.keys])
            if handset_active(m.lx, m.ly, m.rx, m.ry, m.keys):
                st["handset"] = True

        def on_resp(self, m):
            event("response", m.header.identity.api_id, m.header.status.code,
                  m.data.replace("\n", " "))

    rclpy.init()
    node = Probe()
    motion_sent = False

    def spin_for(secs, guard=True):
        t_end = time.time() + secs
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.01)
            if guard:
                check_guards()

    def check_guards():
        now = time.time()
        if st["handset"]:
            raise Yield("handset activity -- the operator has taken over")
        if st["sport_t"] is None or now - st["sport_t"] > STALE_S:
            raise Abort(f"/sportmodestate silent > {STALE_S} s")
        peak = max(t for t in (st["rr"], st["rl"]) if t is not None)
        tag, stop = classify(peak)
        if stop:
            raise Abort(f"rear hip {peak} C >= {CEILING} C")
        if st["speed_trip"]:
            raise Abort(f"speed > {SPEED_ABORT} m/s for {SUSTAIN_S} s (now {st['speed']:.3f})")
        if st["yaw_trip"]:
            raise Abort(f"|yaw_speed| > {YAW_ABORT} rad/s for {SUSTAIN_S} s (now {st['wz']:.3f})")
        if st["px0"] is not None and math.hypot(st["px"] - st["px0"], st["py"] - st["py0"]) > RADIUS_ABORT:
            raise Abort(f"displacement > {RADIUS_ABORT} m")

    def send(name, **params):
        nonlocal motion_sent
        api_id, p = build_request(name, **params)
        req = Request()
        req.header.identity.api_id = api_id
        req.parameter = p
        node.pub.publish(req)
        if name in ("STANDUP", "BALANCESTAND", "MOVE"):
            motion_sent = True
        event("sent", api_id, "", p if p else name)

    def wait_posture(want):
        t_end = time.time() + POSTURE_TIMEOUT_S
        while time.time() < t_end:
            spin_for(0.05)
            if posture_of(st["bh"]) == want:
                event("posture", detail=f"{want} at body_height {st['bh']:.4f}")
                return
        raise Abort(f"posture {want!r} not reached in {POSTURE_TIMEOUT_S} s "
                    f"(body_height {st['bh']})")

    rc = 0
    try:
        # ---------------- preflight: nothing can be sent yet ----------------
        probs = check_env(os.environ)
        spin_for(PREFLIGHT_S, guard=False)
        if st["n_sport"] == 0:
            probs.append("no /sportmodestate in preflight -- blind; refusing")
        if st["n_low"] == 0:
            probs.append("no /lowstate in preflight -- no thermal watch; refusing")
        want = POSTURE_PRE[args.stage]
        if want and posture_of(st["bh"]) != want:
            probs.append(f"stage needs the robot {want}; body_height = {st['bh']}")
        if args.stage in STANDING_STAGES and st["rr"] is not None:
            peak = max(st["rr"], st["rl"])
            if peak >= START_MAX_C:
                probs.append(f"rear hip {peak} C >= {START_MAX_C} C start limit")
        if st["handset"]:
            probs.append("handset active during preflight")
        event("preflight", detail=f"sport {st['n_sport']} low {st['n_low']} hs {st['n_hs']} "
                                  f"bh {st['bh']} RR {st['rr']} RL {st['rl']}")
        if probs:
            for p in probs:
                event("refuse", detail=p)
            raise Refuse(f"{len(probs)} preflight problem(s)")

        if publishing:
            node.make_publisher()
            t_end = time.time() + MATCH_TIMEOUT_S
            while node.pub.get_subscription_count() < 1 and time.time() < t_end:
                rclpy.spin_once(node, timeout_sec=0.05)
            n = node.pub.get_subscription_count()
            event("matched", detail=f"{n} subscriber(s) on {REQUEST_TOPIC}")
            if n < 1:
                event("refuse", detail="publisher matched no subscriber -- nothing sent")
                raise Refuse("unmatched publisher")

        # ---------------- the plan ----------------
        for step in PLANS[args.stage]:
            kind = step[0]
            if kind == "wait":
                spin_for(step[1])
            elif kind == "until":
                wait_posture(step[1])
            elif kind == "send":
                send(step[1], **step[2])
            elif kind == "stream":
                _, vx, vy, vyaw, secs = step
                for _ in range(int(round(secs * MOVE_RATE_HZ))):
                    send("MOVE", vx=vx, vy=vy, vyaw=vyaw)
                    spin_for(1.0 / MOVE_RATE_HZ)
        event("done", detail=args.stage)

    except Refuse as r:
        event("refuse", detail=f"stage not run: {r}")      # nothing was sent
        rc = 2
    except Yield as y:
        # ⛔ Publish NOTHING more. The human is driving.
        event("yield", detail=str(y))
        rc = 3
    except (Abort, KeyboardInterrupt, Exception) as exc:
        detail = "Ctrl+C" if isinstance(exc, KeyboardInterrupt) else f"{type(exc).__name__}: {exc}"
        event("abort", detail=detail)
        rc = 1
        if node.pub is not None and not st["handset"]:
            try:
                send("STOPMOVE")
                if motion_sent:
                    spin_for(1.0, guard=False)
                    send("STANDDOWN")
                    spin_for(4.0, guard=False)
            except Exception as exc2:                    # report, never mask the abort
                event("abort", detail=f"recovery failed: {exc2}")
    finally:
        for fh in files.values():
            fh.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    report = analyse(run_dir)
    with open(os.path.join(run_dir, "report.txt"), "w") as fh:
        fh.write(report + "\n")
    print("\n" + report)
    return rc


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("stage", nargs="?", choices=sorted(PLANS))
    ap.add_argument("--armed", action="store_true",
                    help="actually publish. Without it a publishing stage is a dry run.")
    ap.add_argument("--out", default="/logs")
    ap.add_argument("--replay", metavar="RUN_DIR", help="re-analyse a run; no ROS needed")
    args = ap.parse_args(argv)

    if args.replay:
        print(analyse(args.replay))
        return 0
    if not args.stage:
        ap.error("a stage is required (or --replay)")
    if stage_publishes(args.stage) and not args.armed:
        print("DRY RUN -- nothing will be published. Add --armed to run it.\n")
        print(describe(args.stage))
        return 0
    return run_live(args)


if __name__ == "__main__":
    sys.exit(main())
