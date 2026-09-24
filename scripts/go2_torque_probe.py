#!/usr/bin/env python3
"""Gate 5 -- LowState.motor_state[].tau_est probe. Posture-paired, re-runnable.

THE QUESTION
------------
Session 4's single lying echo (logs/go2/s4_echo_lowstate.txt) shows:

    motors 0-5  (FR, FL legs)   tau_est  +-0.0247 .. +-0.0742   "populated"
    motors 6-11 (RR, RL legs)   tau_est   exactly 0.0           "dead?"
    motors 12-19 (spares)       mode 0, everything 0            inert, expected

Session 5 left this open and said, correctly, that ONE LYING CAPTURE CANNOT
DISTINGUISH a genuine zero from an unreported field -- that is the same defect
that made P7/P13 pass vacuously and that made the `foot_force` claim wrong.

Re-reading the artifact narrows it twice before any new measurement:

  1. The rear block is NOT unpopulated. Motors 6-11 report mode 1, live q, live
     dq, and real temperatures (35, 30, 31 C) in the same MotorState entries
     whose tau_est reads 0.0. A struct-layout error cannot do that -- the array
     entries share one layout, so a misalignment would corrupt motors 0-5 too.
     The question is about the tau_est FIELD on the rear motors, not the block.

  2. The front values are 1-3 LSB of quantisation dither, not a load reading.
     Every one of the six is an exact integer multiple of a per-joint-type step:

         hip, thigh (i%3 in 0,1)   0.024738281965255737 N.m
         calf       (i%3 == 2)     0.047415040433406830 N.m   (= 23/12 x above)

     so 0-5 read +2, -1, +1, +3, +1, -1 counts. Lying, the front legs are as
     unloaded as the rear; the visible difference is one to three counts of
     dither against a hard zero. ==Tagged as inference== from 6 values in one
     echo -- PT3 below measures it properly.

WHY BOTH POSTURES, AND WHY THE FRONT IS THE CONTROL
---------------------------------------------------
Standing is what loads the legs. But "rear tau_est stayed 0 while standing" is
only a defensible negative if the SAME capture shows tau_est responding to load
somewhere -- otherwise a dead rear and a firmware that never reports torque at
all are indistinguishable, and we would have measured a broken instrument.

  PT4 (front legs respond when standing) IS THAT CONTROL. It is load-bearing.
  PT2 (the rear question) is gated against its registered prediction so a
  falsification is recorded as a falsification, but it is NOT a test failure:
  a firmware that does not report rear torque is a finding about the robot.

Run it twice in one session, same boot, with only the posture changed:

    ./go2_torque_probe.py --session s6lie   --posture lying    --samples 6000
    ./go2_torque_probe.py --session s6stand --posture standing --samples 6000

SAFETY -- read-only. This module creates subscriptions only. It must never
construct a publisher, a service client, or an action client. Standing is
commanded by Doug on the handset; we observe it. Reviewers: grep this file for
create_publisher / create_client. There are none.
"""

import argparse
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState, SportModeState

# Load-bearing: the controls that make a rear-leg negative defensible. PT2, the
# actual question, is deliberately NOT here -- see the module docstring.
#
# The set is POSTURE-DEPENDENT. PT4 asks whether the front legs respond to load,
# which is untestable lying: it records n/a, and an n/a is not a pass. Treating
# the set as fixed made every lying run report "INCONCLUSIVE -- control failed:
# PT4" and exit 1, by construction, no matter how good the capture was. Caught
# by the first real lying capture, session 6.
CRITICAL_STANDING = ("PT1", "PT4")
CRITICAL_LYING = ("PT1",)


def critical_for(posture):
    return CRITICAL_STANDING if posture == "standing" else CRITICAL_LYING

# Quantisation steps, from the session-4 lying echo. Registered as a PREDICTION
# (PT3), never used to correct or snap a measured value.
STEP_HIP_THIGH = 0.024738281965255737
STEP_CALF = 0.047415040433406830

# 12 real joints; 0-2 FR, 3-5 FL, 6-8 RR, 9-11 RL, each hip/thigh/calf.
LEGS = ("FR", "FL", "RR", "RL")
JOINTS = ("hip", "thigh", "calf")
N_REAL = 12
FRONT = range(0, 6)
REAR = range(6, 12)

# PT2/PT4 thresholds, in LSB counts rather than N.m so they are independent of
# which joint type is being looked at. Standing on ~15 kg loads a thigh or calf
# joint by whole newton-metres: tens to hundreds of counts. 10 counts (~0.25
# N.m on a hip/thigh) is far above the 1-3 counts of dither seen lying and far
# below any plausible standing torque, so neither outcome is a near miss.
LOADED_COUNTS = 10.0
DITHER_COUNTS = 4.0


def motor_name(i):
    return f"{LEGS[i // 3]}_{JOINTS[i % 3]}"


def step_for(i):
    return STEP_CALF if i % 3 == 2 else STEP_HIP_THIGH


def counts(i, tau):
    """tau in LSB counts of its joint type's quantisation step."""
    return tau / step_for(i)


def modal_sign_fraction(vals):
    """Fraction of NON-ZERO samples sharing the majority sign.

    A DC load holds one sign. Quantisation dither around zero flips. Zeros are
    excluded: they are neither sign, and counting them would let a mostly-zero
    motor look sign-consistent.
    """
    nz = [v for v in vals if v != 0.0]
    if not nz:
        return None
    pos = sum(1 for v in nz if v > 0)
    return max(pos, len(nz) - pos) / len(nz)


class Collector(Node):
    """One node, one participant. /lowstate carries the motors; /sportmodestate
    is subscribed only to corroborate posture (body_height, mode) from an
    independent topic rather than trusting the --posture label.
    """

    def __init__(self, want):
        super().__init__("go2_torque_probe")
        self.want = want
        self.low = []
        self.sport = []
        # Default QoS: RELIABLE / VOLATILE / KEEP_LAST. Never TRANSIENT_LOCAL --
        # durability is VOLATILE robot-wide and the request would match nothing.
        self.create_subscription(LowState, "/lowstate", self._on_low, 10)
        self.create_subscription(SportModeState, "/sportmodestate", self._on_sport, 10)

    def _on_low(self, msg):
        if len(self.low) < self.want:
            # /lowstate has NO stamp -- only uint32 tick. Stamp on arrival.
            self.low.append((time.time(), msg))

    def _on_sport(self, msg):
        if len(self.sport) < self.want:
            self.sport.append((time.time(), msg))

    def done(self):
        return len(self.low) >= self.want


class Report:
    def __init__(self, critical=CRITICAL_STANDING):
        self.rows = []
        self.critical = tuple(critical)

    def add(self, pid, what, predicted, measured, ok):
        # rclpy hands back array fields as numpy arrays, so anything derived
        # from them compares to numpy.bool_, and `numpy.bool_(True) is not True`
        # is True. Coerce, or the summary silently disagrees with its own table.
        self.rows.append((pid, what, predicted, measured, None if ok is None else bool(ok)))

    def failed_critical(self):
        return [r[0] for r in self.rows if r[0] in self.critical and r[4] is not True]

    def falsified(self):
        return [r[0] for r in self.rows if r[4] is False and r[0] not in self.critical]

    def render(self):
        w = [max(len(str(r[i])) for r in self.rows) for i in range(4)]
        head = ("#", "Quantity", "Predicted", "Measured")
        w = [max(w[i], len(head[i])) for i in range(4)]
        line = "  ".join("-" * w[i] for i in range(4)) + "  ------"
        out = ["  ".join(head[i].ljust(w[i]) for i in range(4)) + "  RESULT", line]
        for pid, what, pred, meas, ok in self.rows:
            mark = {True: "PASS", False: "FAIL", None: "n/a "}[ok]
            star = " *" if pid in self.critical else ""
            cells = [pid, what, str(pred), str(meas)]
            out.append("  ".join(cells[i].ljust(w[i]) for i in range(4)) + f"  {mark}{star}")
        return "\n".join(out)


def per_motor_stats(low):
    """Per-motor aggregates over the whole capture."""
    stats = []
    for i in range(N_REAL):
        taus = [m.motor_state[i].tau_est for _, m in low]
        qs = [m.motor_state[i].q for _, m in low]
        dqs = [m.motor_state[i].dq for _, m in low]
        temps = [m.motor_state[i].temperature for _, m in low]
        cts = [counts(i, t) for t in taus]
        absmax = max(abs(c) for c in cts)
        stats.append({
            "i": i,
            "name": motor_name(i),
            "tau_min": min(taus), "tau_max": max(taus),
            "tau_mean": sum(taus) / len(taus),
            "c_absmax": absmax,
            "c_mean": sum(cts) / len(cts),
            "n_zero": sum(1 for t in taus if t == 0.0),
            "n_distinct": len(set(taus)),
            "sign_frac": modal_sign_fraction(taus),
            "q_min": min(qs), "q_max": max(qs),
            "dq_absmax": max(abs(v) for v in dqs),
            "t_min": min(temps), "t_max": max(temps),
            "n": len(taus),
        })
    return stats


def render_motor_table(stats):
    out = [
        "",
        "Per-motor tau_est, over the whole capture. 'counts' = LSB of that joint's",
        f"quantisation step ({STEP_HIP_THIGH:.9f} hip/thigh, {STEP_CALF:.9f} calf).",
        "",
        "  #  motor      tau_min    tau_max   tau_mean  |c|max   zeros/n   distinct  sign%  |dq|max  temp",
        "  -  ---------  ---------  ---------  ---------  ------  ---------  --------  -----  -------  -------",
    ]
    for s in stats:
        sf = "   n/a" if s["sign_frac"] is None else f"{100.0 * s['sign_frac']:5.1f}"
        out.append(
            f"  {s['i']:<2} {s['name']:<9}  {s['tau_min']:+9.4f}  {s['tau_max']:+9.4f}  "
            f"{s['tau_mean']:+9.4f}  {s['c_absmax']:6.1f}  {s['n_zero']:5}/{s['n']:<4} "
            f"{s['n_distinct']:8}  {sf}  {s['dq_absmax']:7.4f}  {s['t_min']:3}-{s['t_max']:<3}"
        )
    return "\n".join(out)


def evaluate(low, sport, rep, posture, stats):
    standing = posture == "standing"

    # ---- PT1: CONTROL -- the rear motor entries are alive at all ----
    # If q/dq/temperature on motors 6-11 went dead, the instrument is suspect and
    # nothing below means anything. This is what makes a tau_est zero a real
    # negative instead of a possibly-broken query.
    rear_dq = max(abs(s["dq_absmax"]) for s in stats if s["i"] in REAR)
    rear_qspan = max(abs(s["q_max"]) for s in stats if s["i"] in REAR)
    rear_temp = min(s["t_min"] for s in stats if s["i"] in REAR)
    alive = rear_qspan > 0.0 and rear_temp > 0
    rep.add("PT1", "CONTROL rear 6-11 q/dq/temp live", "q, temp non-zero (both postures)",
            f"max|q| {rear_qspan:.3f}, max|dq| {rear_dq:.4f}, temp>={rear_temp}", alive)

    # ---- PT2: THE QUESTION -- does rear tau_est respond to load? ----
    # Registered prediction: standing, the rear thigh and calf carry real torque,
    # so tau_est exceeds LOADED_COUNTS. Lying, it stays at zero.
    rear_max = max(s["c_absmax"] for s in stats if s["i"] in REAR)
    rear_loaded = [s for s in stats if s["i"] in REAR and s["c_absmax"] >= LOADED_COUNTS]
    if standing:
        rep.add("PT2", "REAR tau_est standing", f">= {LOADED_COUNTS:.0f} counts on thigh/calf",
                f"max {rear_max:.1f} counts, {len(rear_loaded)}/6 motors loaded",
                rear_max >= LOADED_COUNTS)
    else:
        rep.add("PT2", "REAR tau_est lying", "~0 (unloaded, or unreported)",
                f"max {rear_max:.1f} counts, zeros "
                f"{sum(s['n_zero'] for s in stats if s['i'] in REAR)}/"
                f"{sum(s['n'] for s in stats if s['i'] in REAR)}", None)

    # ---- PT3: quantisation. Tagged as inference from 6 values; now measured ----
    # Every tau_est must be an integer multiple of its joint type's step. This is
    # what establishes the RESOLUTION FLOOR of any torque-based contact
    # inference, and it is also a layout check no mis-decode could satisfy:
    # garbage floats are not integer multiples of two specific constants.
    worst_resid, worst_i = 0.0, None
    smallest_nz = {"hip_thigh": None, "calf": None}
    for _, m in low:
        for i in range(N_REAL):
            tau = m.motor_state[i].tau_est
            c = counts(i, tau)
            resid = abs(c - round(c))
            if resid > worst_resid:
                worst_resid, worst_i = resid, i
            if tau != 0.0:
                key = "calf" if i % 3 == 2 else "hip_thigh"
                a = abs(tau)
                if smallest_nz[key] is None or a < smallest_nz[key]:
                    smallest_nz[key] = a
    rep.add("PT3", "tau_est is integer x step", "residual < 1e-4 counts",
            f"worst {worst_resid:.2e} counts"
            + (f" (motor {worst_i} {motor_name(worst_i)})" if worst_i is not None else ""),
            worst_resid < 1e-4)
    rep.add("PT3b", "  ...smallest non-zero |tau|", "= the step, if 1-count dither occurs",
            " ".join(f"{k}={'none' if v is None else f'{v:.9f}'}" for k, v in smallest_nz.items()),
            None)

    # ---- PT4: CONTROL -- the front legs respond to load when standing ----
    # Without this, a zero rear is uninterpretable: it cannot be told apart from
    # a firmware that reports no torque anywhere. Load-bearing when standing.
    front_max = max(s["c_absmax"] for s in stats if s["i"] in FRONT)
    front_loaded = [s for s in stats if s["i"] in FRONT and s["c_absmax"] >= LOADED_COUNTS]
    if standing:
        rep.add("PT4", "CONTROL front 0-5 respond to load", f">= {LOADED_COUNTS:.0f} counts",
                f"max {front_max:.1f} counts, {len(front_loaded)}/6 motors loaded",
                front_max >= LOADED_COUNTS)
    else:
        rep.add("PT4", "CONTROL front 0-5 respond to load", "standing only -- not testable lying",
                f"max {front_max:.1f} counts (unloaded)", None)

    # ---- PT5: sign consistency -- DC load vs dither ----
    # A leg holding up the robot produces a steady-signed torque. Dither around
    # zero does not. This separates "small but real" from "noise".
    loaded = [s for s in stats if s["c_absmax"] >= LOADED_COUNTS and s["sign_frac"] is not None]
    if loaded:
        worst = min(loaded, key=lambda s: s["sign_frac"])
        rep.add("PT5", f"sign consistency on {len(loaded)} loaded motors", ">= 95 % modal sign",
                f"worst {100.0 * worst['sign_frac']:.1f} % ({worst['name']})",
                worst["sign_frac"] >= 0.95)
    else:
        rep.add("PT5", "sign consistency on loaded motors", ">= 95 % modal sign",
                "NO motor above the load threshold -- INCONCLUSIVE, not a negative", None)

    # ---- PT6: the lying baseline, re-measured ----
    # Session 4's "0-5 populated" was 1-3 counts. Predicted: lying, all 12 sit
    # within DITHER_COUNTS of zero -- i.e. that "populated" meant dither.
    allmax = max(s["c_absmax"] for s in stats)
    if standing:
        rep.add("PT6", "lying baseline |tau| <= 4 counts", "lying only", f"max {allmax:.1f} counts", None)
    else:
        rep.add("PT6", "lying baseline |tau| <= 4 counts", f"<= {DITHER_COUNTS:.0f} counts on all 12",
                f"max {allmax:.1f} counts", allmax <= DITHER_COUNTS)

    # ---- PT7: foot_force, third boot. Replication of the session-5 finding ----
    # Session 5: fixed per-foot constants, identical lying and standing. Recorded
    # per posture so the two runs can be compared directly.
    lff = [list(m.foot_force) for _, m in low]
    perfoot = [[row[i] for row in lff] for i in range(4)]
    rep.add("PT7", "LowState.foot_force per foot [FR FL RR RL]", "fixed constants, posture-blind",
            " ".join(f"{min(f)}-{max(f)}" for f in perfoot), None)
    est = [list(m.foot_force_est) for _, m in low]
    rep.add("PT7b", "  ...foot_force_est", "all zero on this firmware",
            f"max |v| {max(max(abs(v) for v in row) for row in est)}", None)

    # ---- PT8: the raw fields are NOT data. Say so, so nobody reads them later ----
    raw = 0.0
    for _, m in low:
        for i in range(N_REAL):
            ms = m.motor_state[i]
            raw = max(raw, abs(ms.q_raw), abs(ms.dq_raw), abs(ms.ddq_raw), abs(ms.ddq))
    rep.add("PT8", "q_raw / dq_raw / ddq_raw / ddq", "unpopulated on this fw",
            f"max |v| {raw:.6g}", None)

    # ---- PT9: spares still inert (the layout guard from P11) ----
    spare_live = False
    smax = 0.0
    for _, m in low:
        for ms in m.motor_state[N_REAL:]:
            smax = max(smax, abs(ms.q), abs(ms.dq), abs(ms.tau_est))
            if ms.q != 0.0 or ms.dq != 0.0 or ms.tau_est != 0.0:
                spare_live = True
    rep.add("PT9", "motor_state[12..19] spares inert", "0 -- a shift bleeds data in",
            f"max |v| {smax:.3g}, live={spare_live}", not spare_live)

    # ---- posture corroboration, from the OTHER topic ----
    # --posture is a label typed by a human. body_height and max|q| are measured,
    # and session 5 used exactly these to show a stand had happened.
    if sport:
        bh = [m.body_height for _, m in sport]
        modes = sorted({m.mode for _, m in sport})
        rep.add("PT10", "posture corroboration (sportmodestate)", f"label says {posture}",
                f"body_height {min(bh):.4f}..{max(bh):.4f} m, mode={modes}", None)
    else:
        rep.add("PT10", "posture corroboration (sportmodestate)", f"label says {posture}",
                "NO sportmodestate samples -- posture unverified by a second topic", None)
    qmax = max(abs(s["q_max"]) for s in stats)
    pv = [m.power_v for _, m in low]
    rep.add("PT11", "  ...max|q| and power_v", "max|q| ~2.8 lying, ~1.4 standing",
            f"max|q| {qmax:.3f}, power_v {sum(pv) / len(pv):.2f} V "
            f"({min(pv):.2f}..{max(pv):.2f})", None)


def write_csv(low, path):
    """Full per-sample dump. Every command's full output goes to a file.

    The standing tau_est numbers were collected in memory by s5stand2 and never
    written down, which is why this question is still open. Write them.
    """
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        head = ["recv_epoch", "tick", "power_v", "power_a",
                "ff_FR", "ff_FL", "ff_RR", "ff_RL"]
        for i in range(N_REAL):
            n = motor_name(i)
            head += [f"q_{n}", f"dq_{n}", f"tau_{n}", f"cnt_{n}", f"temp_{n}"]
        w.writerow(head)
        for t, m in low:
            row = [f"{t:.6f}", m.tick, f"{m.power_v:.4f}", f"{m.power_a:.4f}"]
            row += [int(v) for v in list(m.foot_force)[:4]]
            for i in range(N_REAL):
                ms = m.motor_state[i]
                row += [f"{ms.q:.9g}", f"{ms.dq:.9g}", f"{ms.tau_est:.9g}",
                        f"{counts(i, ms.tau_est):.6g}", ms.temperature]
            w.writerow(row)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=6000)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--session", default="s6", help="tag for the banner and default file names")
    ap.add_argument("--posture", required=True, choices=("lying", "standing"),
                    help="MEASURED posture. PT2/PT4/PT6 are meaningless without it, and a "
                         "wrong label here produces a confidently wrong verdict.")
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()
    csv_path = args.csv or f"/logs/{args.session}_torque_samples.csv"

    rclpy.init()
    node = Collector(args.samples)
    t0 = time.time()
    while rclpy.ok() and not node.done() and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    low, sport = node.low, node.sport
    node.destroy_node()
    rclpy.shutdown()

    print("=" * 78)
    print(f"LowState.motor_state[].tau_est PROBE -- Gate 5 session {args.session}")
    print(f"Posture: {args.posture.upper()}.  Read-only: subscriptions only, no publisher.")
    print(f"Collected: /lowstate {len(low)}, /sportmodestate {len(sport)} "
          f"(requested {args.samples}) in {time.time() - t0:.1f}s")
    print(f"UTC now: {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}")
    print("=" * 78)

    # Distinguish "no data" from "broken instrument": name the silent topic.
    if not low:
        print(f"NO DATA on /lowstate -- 0 samples in {args.timeout}s.")
        print("\nINCONCLUSIVE. Not a finding about tau_est: no samples to decode.")
        print("Check ROS_DOMAIN_ID, CYCLONEDDS_URI, the cable, and that the robot is on.")
        return 2

    stats = per_motor_stats(low)
    rep = Report(critical_for(args.posture))
    evaluate(low, sport, rep, args.posture, stats)
    print(rep.render())
    print()
    print(f"* = load-bearing for this posture: {', '.join(rep.critical)}. These are CONTROLS:")
    print("  without them a zero on the rear motors cannot be told apart from a broken")
    print("  or torque-blind instrument. PT4 is untestable lying and is not gated there.")
    print(render_motor_table(stats))
    write_csv(low, csv_path)
    print(f"\nPer-sample CSV: {csv_path}")

    # ---- the interpretation, stated explicitly, both ways ----
    rear_max = max(s["c_absmax"] for s in stats if s["i"] in REAR)
    front_max = max(s["c_absmax"] for s in stats if s["i"] in FRONT)
    print()
    if args.posture == "standing":
        if front_max < LOADED_COUNTS:
            print("READING: neither front nor rear responded to standing load.")
            print("  Nothing can be concluded about the REAR specifically -- the control")
            print("  failed, so this is a statement about the instrument, not the robot.")
        elif rear_max >= LOADED_COUNTS:
            print("READING: rear tau_est comes ALIVE under load while the front also responds.")
            print("  The lying zeros were a GENUINE zero. Torque-based contact inference is")
            print("  viable on all four legs -- the only contact signal this robot has, since")
            print("  foot_force is a fixed constant and foot_force_est is zero.")
        else:
            print("READING: front responds to load, rear stays at zero, in the same capture,")
            print("  on the same boot, with rear q/dq/temperature live (PT1).")
            print("  tau_est is NOT REPORTED for motors 6-11 on this firmware. Combined with")
            print("  foot_force (constant) and foot_force_est (zero), the robot exposes NO")
            print("  usable contact signal for the rear legs. Hard constraint on go2_adapter.")
    else:
        print("READING: lying is the BASELINE run. It cannot answer PT2 on its own --")
        print("  that is the error session 5 diagnosed. Pair it with a standing run on the")
        print("  SAME boot before drawing any conclusion about the rear motors.")

    bad = rep.failed_critical()
    fal = rep.falsified()
    print()
    if bad:
        print(f"RESULT: INCONCLUSIVE -- control(s) failed: {', '.join(bad)}")
        print("Do not interpret the tau_est numbers above. Rule out the setup first:")
        print("wrong image, unsourced /ws overlay, unbound Cyclone, domain mismatch,")
        print("or a --posture label that does not match what the robot was doing.")
        return 1
    print(f"RESULT: VALID -- controls ({', '.join(rep.critical)}) held.")
    if fal:
        print(f"FALSIFIED predictions: {', '.join(fal)}. Record them as falsified; do not")
        print("overwrite the prediction. A falsified PT2 is a finding about the firmware.")
    print("Scope: this measures what the firmware PUBLISHES in tau_est. It does not")
    print("establish the accuracy or units of the value, only its presence, quantisation,")
    print("and response to a controlled posture change.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
