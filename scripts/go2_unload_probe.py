#!/usr/bin/env python3
"""Gate 5 -- contact TRANSITION probe. Time series, not aggregates.

WHY A SECOND PROBE
------------------
go2_torque_probe.py answered "does tau_est respond to load?" -- yes, on all 12
motors, 32-330 counts standing against 1-9 lying (session 6). It answers that
with AGGREGATES over a 12 s window: min, max, mean, sign fraction.

That shape is useless for the next question. A contact detector does not fire on
a mean; it fires on an EDGE. A step change buried in a 12 s mean is invisible.
This module keeps the time series and looks for the edge.

WHAT IT MEASURES
----------------
  * per-leg load proxy, per sample, at the /lowstate rate (~500 Hz, 2 ms)
  * edges in that proxy: when, which direction, how big, how fast (10-90 %)
  * redistribution: what the OTHER three legs did at the same moment
  * hysteresis: does a leg return to its pre-lift level after the foot is back?

⛔ THE PROXY IS NOT A FORCE. It is the sum of |tau_est|, in quantisation counts,
over a leg's three joints. Converting joint torque to a ground reaction force
needs a leg Jacobian and link lengths -- a MODEL we have not measured. So the
redistribution check below is DIRECTIONAL ("the other three moved the right
way"), never arithmetic ("the sum is conserved"). Do not upgrade it.

THE BUILT-IN CONTROL
--------------------
When one foot leaves the ground the other three must carry that weight. So a
real unload shows TWO things at once: the lifted leg collapses AND the others
rise. A capture where only the lifted leg changes is evidence against the
reading, not for it. Four-legged standing could not provide this control;
lifting one leg can.

PU0 guards the obvious confound: a communications dropout looks exactly like a
step down in every leg at once. The tick field is checked for continuity so an
edge can be told apart from a gap in the data.

SAFETY -- read-only. Subscriptions only. It must never construct a publisher, a
service client, or an action client. Every physical action in this protocol is
performed by Doug by hand; we observe. Reviewers: grep for create_publisher /
create_client. There are none.

DATA REACHES DISK AS IT ARRIVES. Rows are written during the capture, not after,
so a crash in the analysis cannot lose a capture that took physical setup to
produce. The analysis can always be re-run offline from the CSV with --replay.
"""

import argparse
import csv
import os
import statistics
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from unitree_go.msg import LowState, SportModeState
except ImportError:                       # --replay works with no ROS at all
    rclpy = None
    Node = object

# One source of truth for the quantisation constants and motor naming: they are
# measured values recorded in the torque probe, and a second copy would drift.
from go2_torque_probe import (
    LEGS, JOINTS, N_REAL, Report, counts, motor_name, step_for,
    STEP_HIP_THIGH, STEP_CALF,
)

# Session 6 measurements that set the scales below:
#   standing per-leg proxy  ~250-500 counts   (RR: hip 322 + thigh 55 + calf 132)
#   lying    per-leg proxy  ~5-20 counts
# An unload should therefore move a leg by hundreds of counts. 50 is well clear
# of dither and well below a real transition, so neither outcome is a near miss.
EDGE_COUNTS = 50.0
# Dither ceiling from three lying captures, session 6: 9 counts on one motor.
# A leg genuinely off the ground should land near its lying level; 3x gives room
# for a standing leg that is unloaded but still held in a stance posture.
DITHER_PROXY = 30.0
MEDIAN_WIN = 25          # 50 ms at 500 Hz -- kills dither, keeps a real edge
LAG = 100                # 200 ms -- the lag over which a step is looked for
SETTLE = 250             # 500 ms of steady samples either side of an edge


def leg_of(i):
    return i // 3


def median_filter(xs, win):
    """Median over a centred window. Chosen over a mean because tau_est dither
    is quantised and occasionally spikes by several counts; a mean smears a
    spike across the window and can invent a small edge."""
    n = len(xs)
    if n < win or win < 2:
        return list(xs)
    half = win // 2
    out = [0.0] * n
    for k in range(n):
        lo = max(0, k - half)
        hi = min(n, k + half + 1)
        out[k] = statistics.median(xs[lo:hi])
    return out


class Capture:
    """Flat, streaming storage.

    The torque probe keeps (t, msg) tuples, which is fine for 6000 samples and
    is NOT fine here: a 120 s capture is ~60000 LowState messages, each with 20
    MotorState entries and a 15-cell BMS. Fields are extracted on arrival and
    the message is dropped."""

    def __init__(self, csv_path):
        self.t = []
        self.tick = []
        self.power_v = []
        self.foot_force = []
        self.cnt = [[] for _ in range(N_REAL)]      # tau_est in counts
        self.q = [[] for _ in range(N_REAL)]
        self.temp = [[] for _ in range(N_REAL)]
        self.body_height = []
        self.fh = open(csv_path, "w", newline="")
        self.w = csv.writer(self.fh)
        head = ["recv_epoch", "tick", "power_v", "ff_FR", "ff_FL", "ff_RR", "ff_RL"]
        for i in range(N_REAL):
            n = motor_name(i)
            head += [f"q_{n}", f"dq_{n}", f"tau_{n}", f"cnt_{n}", f"temp_{n}"]
        head += [f"proxy_{leg}" for leg in LEGS]
        self.w.writerow(head)

    def add(self, t, m):
        self.t.append(t)
        self.tick.append(m.tick)
        self.power_v.append(m.power_v)
        ff = [int(v) for v in list(m.foot_force)[:4]]
        self.foot_force.append(ff)
        row = [f"{t:.6f}", m.tick, f"{m.power_v:.4f}"] + ff
        proxy = [0.0] * 4
        for i in range(N_REAL):
            ms = m.motor_state[i]
            c = counts(i, ms.tau_est)
            self.cnt[i].append(c)
            self.q[i].append(ms.q)
            self.temp[i].append(ms.temperature)
            proxy[leg_of(i)] += abs(c)
            row += [f"{ms.q:.9g}", f"{ms.dq:.9g}", f"{ms.tau_est:.9g}",
                    f"{c:.6g}", ms.temperature]
        row += [f"{p:.6g}" for p in proxy]
        self.w.writerow(row)

    def close(self):
        self.fh.close()

    def proxy(self, leg):
        """Per-leg load proxy: sum of |counts| over the leg's three joints.
        ⛔ Not a force. See the module docstring."""
        a, b, c = (self.cnt[3 * leg + j] for j in range(3))
        return [abs(x) + abs(y) + abs(z) for x, y, z in zip(a, b, c)]


class Collector(Node):
    def __init__(self, cap, duration):
        super().__init__("go2_unload_probe")
        self.cap = cap
        self.deadline = time.time() + duration
        self.sport_bh = []
        self.create_subscription(LowState, "/lowstate", self._on_low, 50)
        self.create_subscription(SportModeState, "/sportmodestate", self._on_sport, 10)

    def _on_low(self, msg):
        self.cap.add(time.time(), msg)

    def _on_sport(self, msg):
        self.sport_bh.append((time.time(), msg.body_height, msg.mode))

    def done(self):
        return time.time() >= self.deadline


def level(series, lo, hi):
    """Steady level over a slice, as a median."""
    seg = series[max(0, lo):max(1, hi)]
    return statistics.median(seg) if seg else 0.0


def levels_around(series, k, lag=LAG, settle=SETTLE):
    """The steady levels either side of an edge at index k.

    ⚠️ The sampling windows sit a FULL SECOND clear of the edge, not immediately
    beside it. Measured right next to a slow transition they land INSIDE it: an
    800 ms ramp then reports a 10-90 % time of ~300 ms, because `before` is
    already partway down the ramp. The gate still failed, which is how that
    would have survived -- a correct verdict sitting next to a wrong number.

    ⚑ The cost is a protocol constraint: each state must be HELD for at least
    ~2 s, or these windows bleed into the neighbouring event. The session 7
    protocol says hold for 5 s.
    """
    a = level(series, k - lag // 2 - 3 * settle, k - lag // 2 - 2 * settle)
    b = level(series, k + lag // 2 + 2 * settle, k + lag // 2 + 3 * settle)
    return a, b


def find_edges(t, proxy_s, thresh=EDGE_COUNTS, lag=LAG, settle=SETTLE):
    """Edges in one leg's smoothed proxy.

    A step is a point where the level `lag` samples later differs from the level
    now by more than `thresh`. Consecutive detections are merged into one edge
    and reported at the steepest point, with before/after levels measured over
    `settle` samples clear of the transition.
    """
    n = len(proxy_s)
    # levels_around() samples up to 3*settle clear of an edge, so a capture
    # shorter than that cannot produce a trustworthy level either side.
    if n < lag + 6 * settle:
        return []
    raw = []
    for k in range(n - lag):
        d = proxy_s[k + lag] - proxy_s[k]
        if abs(d) >= thresh:
            raw.append((k, d))
    if not raw:
        return []
    edges = []
    group = [raw[0]]
    for item in raw[1:]:
        if item[0] - group[-1][0] <= lag:
            group.append(item)
        else:
            edges.append(group)
            group = [item]
    edges.append(group)

    out = []
    for g in edges:
        k = max(g, key=lambda it: abs(it[1]))[0] + lag // 2
        before, after = levels_around(proxy_s, k, lag, settle)
        d = after - before
        if abs(d) < thresh:
            continue
        # 10-90 % transition time, measured on the smoothed series.
        # The search window must be WIDER than any transition worth reporting,
        # or the number is silently clipped to the window: an 800 ms ramp
        # measured 296 ms when this was +-lag (400 ms total). The verdict was
        # still right, which is exactly why it would have gone unnoticed --
        # a gate can be correct while the measurement beside it is wrong.
        lo_v, hi_v = (before + 0.1 * d), (before + 0.9 * d)
        span = lag + settle                       # 700 ms either side at 500 Hz
        idx = range(max(0, k - span), min(n, k + span))
        crossed = [j for j in idx
                   if (min(lo_v, hi_v) <= proxy_s[j] <= max(lo_v, hi_v))]
        dt = (t[crossed[-1]] - t[crossed[0]]) if len(crossed) >= 2 else float("nan")
        out.append({"i": k, "t": t[k], "before": before, "after": after,
                    "delta": d, "rise_s": dt})
    return out


def analyse(cap, rep, events, args):
    t = cap.t
    n = len(t)
    dur = t[-1] - t[0] if n > 1 else 0.0
    rate = (n - 1) / dur if dur > 0 else 0.0

    # ---- PU0: CONTROL -- no data gap masquerading as an edge ----
    # A dropout looks exactly like a simultaneous step in every leg. tick is a
    # 1 kHz counter on the robot, so a gap in it is a gap in the DATA, not in
    # the robot's state.
    gaps = []
    for k in range(1, n):
        dtick = (cap.tick[k] - cap.tick[k - 1]) % (2 ** 32)
        if dtick > 20:                   # >20 ms of robot time between samples
            gaps.append((t[k], dtick))
    worst = max((g[1] for g in gaps), default=0)
    rep.add("PU0", "CONTROL sample continuity", "no tick gap > 20 ms",
            f"{len(gaps)} gaps, worst {worst} ms, {rate:.0f} Hz over {dur:.1f} s",
            len(gaps) == 0)

    smoothed = {}
    for leg in range(4):
        smoothed[leg] = median_filter(cap.proxy(leg), MEDIAN_WIN)

    # ---- per-leg edges ----
    all_edges = []
    for leg in range(4):
        for e in find_edges(t, smoothed[leg], args.edge_counts):
            e["leg"] = leg
            all_edges.append(e)
    all_edges.sort(key=lambda e: e["t"])

    rep.add("PU1", "edges detected in the per-leg proxy", f">= {args.edge_counts:.0f} counts",
            f"{len(all_edges)} edges across {len({e['leg'] for e in all_edges})} legs",
            len(all_edges) > 0)

    # ---- PU2: redistribution -- THE CONTROL a four-legged stand cannot give ----
    #
    # Evaluated per TIME CLUSTER, not per edge, and the predicate depends on the
    # protocol step. Both of those are corrections to a first version that got
    # this wrong:
    #
    #   * Per edge was wrong because ONE physical event produces FOUR edges. A
    #     lift shows the lifted leg down and three up; the replace shows exactly
    #     the mirror -- three legs down, one up -- which read edge-by-edge looks
    #     like three unloads with no redistribution. It is one event, and it is
    #     the SAME conservation signature.
    #
    #   * Step-aware because lifting the WHOLE robot legitimately drops all four
    #     legs at once with nothing rising. Gating that on "somebody must rise"
    #     would fail a correct capture. ⛔ That is the same defect as applying a
    #     standing-only control to a lying capture (go2_torque_probe, session 6),
    #     so it is worth naming twice: a predicate is only meaningful against the
    #     manoeuvre it was written for.
    #
    # PU0 is what separates a whole-body lift from a dropout. Without it, "all
    # four legs went quiet at once" has two completely different explanations.
    for e in all_edges:
        k = e["i"]
        others = []
        for leg in range(4):
            if leg == e["leg"]:
                continue
            b, a = levels_around(smoothed[leg], k)
            others.append(a - b)
        e["others"] = others

    clusters = []
    for e in sorted(all_edges, key=lambda e: e["i"]):
        if clusters and e["i"] - clusters[-1][-1]["i"] <= SETTLE:
            clusters[-1].append(e)
        else:
            clusters.append([e])

    MOVE = 10.0            # counts; below this a leg did not meaningfully move
    if not clusters:
        rep.add("PU2", "redistribution (per event cluster)", "weight moves, it does not vanish",
                "NO edge detected -- INCONCLUSIVE, not a negative", None)
    elif args.step == "wholebody":
        # Expect the opposite signature: every leg moves the SAME way at once.
        oks, detail = [], []
        for c in clusters:
            dom = max(c, key=lambda e: abs(e["delta"]))
            same = sum(1 for d in dom["others"] if d * dom["delta"] > 0 and abs(d) >= MOVE)
            oks.append(same == 3)
            detail.append(f"{LEGS[dom['leg']]}{dom['delta']:+.0f}/{same}same")
        rep.add("PU2", f"whole-body lift, {len(clusters)} event(s)",
                "all four legs move together", " ".join(detail), all(oks))
    elif args.step == "baseline":
        rep.add("PU2", "redistribution (per event cluster)", "no event expected in a baseline",
                f"{len(clusters)} cluster(s) detected -- investigate", None)
    else:
        oks, detail = [], []
        for c in clusters:
            dom = max(c, key=lambda e: abs(e["delta"]))
            opp = sum(1 for d in dom["others"] if d * dom["delta"] < 0 and abs(d) >= MOVE)
            oks.append(opp >= 2)
            detail.append(f"{LEGS[dom['leg']]}{dom['delta']:+.0f}/{opp}opp")
        rep.add("PU2", f"redistribution over {len(clusters)} event cluster(s)",
                ">= 2 other legs move the OPPOSITE way", " ".join(detail), all(oks))

    # ---- PU3: how fast the transition resolves ----
    rises = [e["rise_s"] for e in all_edges if e["rise_s"] == e["rise_s"]]
    if rises:
        rep.add("PU3", "edge 10-90 % transition time", "< 0.200 s",
                f"max {max(rises) * 1e3:.0f} ms, median {statistics.median(rises) * 1e3:.0f} ms",
                max(rises) < 0.200)
    else:
        rep.add("PU3", "edge 10-90 % transition time", "< 0.200 s",
                "no edge with a measurable transition", None)

    # ---- PU4: does an unloaded leg reach the dither band? ----
    downs = [e for e in all_edges if e["delta"] < 0]
    if downs:
        deepest = min(downs, key=lambda e: e["after"])
        rep.add("PU4", "unloaded leg reaches the dither band",
                f"<= {DITHER_PROXY:.0f} counts",
                f"lowest post-edge level {deepest['after']:.0f} counts "
                f"({LEGS[deepest['leg']]})", deepest["after"] <= DITHER_PROXY)
    else:
        rep.add("PU4", "unloaded leg reaches the dither band", f"<= {DITHER_PROXY:.0f} counts",
                "no unload edge -- INCONCLUSIVE", None)

    # ---- PU5: hysteresis -- THE ONE THAT DECIDES WHETHER A THRESHOLD WORKS ----
    # Pair each down edge with the next up edge on the same leg and compare the
    # level before the lift with the level after the foot is back. If they do
    # not match, a fixed threshold is not enough on its own.
    pairs = []
    for leg in range(4):
        legedges = [e for e in all_edges if e["leg"] == leg]
        for a, b in zip(legedges, legedges[1:]):
            if a["delta"] < 0 and b["delta"] > 0:
                pairs.append((leg, a["before"], b["after"]))
    if pairs:
        worst_leg, pre, post = max(pairs, key=lambda p: abs(p[2] - p[1]))
        # Signed for the reader, absolute for the gate: "-40 %" and "+40 %" are
        # very different findings and the row must not blur them.
        rel = (post - pre) / pre * 100.0 if pre else float("inf")
        rep.add("PU5", f"hysteresis over {len(pairs)} lift/replace pair(s)",
                "returns within 10 % of pre-lift",
                f"worst {LEGS[worst_leg]}: {pre:.0f} -> {post:.0f} counts ({rel:+.1f} %)",
                abs(rel) <= 10.0)
    else:
        rep.add("PU5", "hysteresis over lift/replace pairs", "returns within 10 %",
                "no down-then-up pair on one leg -- INCONCLUSIVE", None)

    # ---- PU6: sign consistency collapses on the unloaded leg ----
    # Session 6: dither flips sign (64-99 % modal), load does not (100 %). If
    # that is the best discriminator, an unloaded leg must show it.
    if downs:
        e = min(downs, key=lambda x: x["after"])
        k = e["i"]
        lo, hi = k + LAG // 2, k + LAG // 2 + SETTLE * 4
        fracs = []
        for j in range(3):
            i = 3 * e["leg"] + j
            seg = [v for v in cap.cnt[i][lo:hi] if v != 0.0]
            if seg:
                pos = sum(1 for v in seg if v > 0)
                fracs.append(max(pos, len(seg) - pos) / len(seg))
        rep.add("PU6", f"sign consistency, unloaded {LEGS[e['leg']]}", "< 95 % (dither-like)",
                " ".join(f"{100 * f:.0f}%" for f in fracs) or "no non-zero samples",
                bool(fracs) and min(fracs) < 0.95)
    else:
        rep.add("PU6", "sign consistency on an unloaded leg", "< 95 %",
                "no unload edge -- INCONCLUSIVE", None)

    # ---- PU7: foot_force, one more time, against a real contact event ----
    # Session 6 showed it ignores a posture change. A foot LEAVING THE GROUND is
    # the most direct test there is. Recorded, not gated -- a surprise here would
    # be a major finding and must not be gated away.
    ffmin = [min(r[i] for r in cap.foot_force) for i in range(4)]
    ffmax = [max(r[i] for r in cap.foot_force) for i in range(4)]
    rep.add("PU7", "LowState.foot_force across the whole capture",
            "unchanged -- no contact response (s6)",
            " ".join(f"{LEGS[i]} {ffmin[i]}-{ffmax[i]}" for i in range(4)), None)

    return all_edges, smoothed


def render_edges(edges, events):
    if not edges:
        return "\nNo edges detected. If a physical action was performed, that is a FINDING."
    out = ["", "Edges, in time order. 'others' = what the other three legs did at the",
           "same moment (directional check only -- the proxy is not a force).", "",
           "   t_rel   leg   before    after    delta   10-90%   others (FR FL RR RL, minus self)",
           "  ------  ----  -------  -------  -------  -------  --------------------------------"]
    t0 = edges[0]["t"]
    for e in edges:
        oth = ("  ".join(f"{d:+6.0f}" for d in e["others"])) if "others" in e else "-- (up edge)"
        rise = "   n/a " if e["rise_s"] != e["rise_s"] else f"{e['rise_s'] * 1e3:5.0f}ms"
        out.append(f"  {e['t'] - t0:6.2f}  {LEGS[e['leg']]:>4}  {e['before']:7.0f}  "
                   f"{e['after']:7.0f}  {e['delta']:+7.0f}  {rise}  {oth}")
    if events:
        out += ["", "Narrated events (label only -- the DATA gives the time):"]
        for ts, label in events:
            out.append(f"  {ts - t0:6.2f}  {label}")
    return "\n".join(out)


def load_events(path, ):
    """Optional 'epoch label' lines. The narration supplies WHAT happened; the
    edge detector supplies WHEN. Chat or keyboard latency of a second or two
    does not matter under that division of labour."""
    out = []
    if not path or not os.path.exists(path):
        return out
    with open(path) as fh:
        for line in fh:
            parts = line.strip().split(None, 1)
            if len(parts) == 2:
                try:
                    out.append((float(parts[0]), parts[1]))
                except ValueError:
                    pass
    return out


class Replay(Capture):
    """Re-analyse a capture offline, with no ROS and no robot. The analysis can
    be fixed and re-run against data that cost physical setup to obtain."""

    def __init__(self, path):
        self.t, self.tick, self.power_v, self.foot_force = [], [], [], []
        self.cnt = [[] for _ in range(N_REAL)]
        self.q = [[] for _ in range(N_REAL)]
        self.temp = [[] for _ in range(N_REAL)]
        self.body_height = []
        with open(path) as fh:
            for row in csv.DictReader(fh):
                self.t.append(float(row["recv_epoch"]))
                self.tick.append(int(row["tick"]))
                self.power_v.append(float(row["power_v"]))
                self.foot_force.append([int(row[f"ff_{l}"]) for l in LEGS])
                for i in range(N_REAL):
                    n = motor_name(i)
                    self.cnt[i].append(float(row[f"cnt_{n}"]))
                    self.q[i].append(float(row[f"q_{n}"]))
                    self.temp[i].append(int(row[f"temp_{n}"]))

    def close(self):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=90.0,
                    help="seconds of continuous capture (default 90)")
    ap.add_argument("--session", default="s7")
    ap.add_argument("--step", required=True,
                    choices=("push", "singleleg", "wholebody", "baseline"),
                    help="which protocol step this capture is. Recorded in the banner; "
                         "a wrong label here mislabels the evidence, it does not change it.")
    ap.add_argument("--events", default=None,
                    help="optional file of 'epoch label' lines, appended live")
    ap.add_argument("--edge-counts", type=float, default=EDGE_COUNTS)
    ap.add_argument("--csv", default=None)
    ap.add_argument("--replay", default=None,
                    help="re-analyse an existing CSV offline -- no ROS, no robot")
    args = ap.parse_args()
    csv_path = args.csv or f"/logs/{args.session}_{args.step}_unload_samples.csv"

    if args.replay:
        cap = Replay(args.replay)
        sport = []
        elapsed = cap.t[-1] - cap.t[0] if len(cap.t) > 1 else 0.0
    else:
        if rclpy is None:
            print("ROS not available and --replay not given.", file=sys.stderr)
            return 2
        cap = Capture(csv_path)
        rclpy.init()
        node = Collector(cap, args.duration)
        t0 = time.time()
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        sport = node.sport_bh
        elapsed = time.time() - t0
        node.destroy_node()
        rclpy.shutdown()
        cap.close()

    print("=" * 78)
    print(f"CONTACT TRANSITION PROBE -- Gate 5 session {args.session}, step: {args.step.upper()}")
    print("Read-only: subscriptions only, no publisher. All motion is by hand.")
    print(f"Collected: /lowstate {len(cap.t)} samples in {elapsed:.1f}s"
          + (f"  [REPLAY of {args.replay}]" if args.replay else ""))
    print(f"UTC now: {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}")
    print("=" * 78)

    if len(cap.t) < LAG + 6 * SETTLE:
        print(f"TOO FEW SAMPLES ({len(cap.t)}) to look for an edge "
              f"(need {LAG + 6 * SETTLE}, ~3.2 s at 500 Hz).")
        print("INCONCLUSIVE. Not a finding about contact: check the cable, the domain,")
        print("and that the robot is on.")
        return 2

    events = load_events(args.events)
    rep = Report(critical=("PU0",))
    edges, _ = analyse(cap, rep, events, args)
    print(rep.render())
    print()
    print("* = load-bearing. PU0 is the CONTROL: a data gap looks exactly like a")
    print("  simultaneous step in every leg, and must be excluded first.")
    print(render_edges(edges, events))
    if sport:
        bh = [b for _, b, _ in sport]
        print(f"\nbody_height {min(bh):.4f}..{max(bh):.4f} m over the capture "
              f"(a stand/sit inside the window would show here)")
    if not args.replay:
        print(f"\nPer-sample CSV: {csv_path}")

    bad = rep.failed_critical()
    print()
    if bad:
        print(f"RESULT: INCONCLUSIVE -- control(s) failed: {', '.join(bad)}")
        print("Edges in this capture may be dropouts. Do not interpret them.")
        return 1
    print("RESULT: VALID -- sample continuity held, so the edges are the robot's, not the link's.")
    print("Scope: this measures tau_est's response to a contact change made BY HAND.")
    print("It does not calibrate tau_est, and the per-leg proxy is NOT a ground force.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
