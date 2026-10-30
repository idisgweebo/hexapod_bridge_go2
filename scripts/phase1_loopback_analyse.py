#!/usr/bin/env python3
"""Score phase 1 STAGE 2 (scripts/phase1_loopback_test.sh) against the expectations
registered in phase1_go2_feedback_plan.md §9 "Stage 2" on 30 Oct 2026, BEFORE the
first run. This file was written from that table, not from any output.

    phase1_loopback_analyse.py OUTDIR

Per phase it reads: check/eval.csv + check/sent.csv (the hexapod checker), check.txt (its
rc), ros1.jsonl (scripts/phase1_ros1_probe.py), ros2.jsonl (scripts/fake_go2_phase1.py),
t_kill_*.txt (S6). Every timestamp is time.time() on ONE host (all containers use
--network host and the host clock), so cross-side differences are real latencies.

LABEL SEQUENCES: eval.csv's cls, rows before the first accepted status dropped (reason
"no /go2/robot_status accepted yet"), consecutive duplicates collapsed.

Exit 0 iff every CHECK passes (the plan's label table and its "also required" column).
PREDICTION verdicts (PFC-*) are printed as MATCHED / NOT MATCHED with their numbers and do
NOT change the exit code: a falsified prediction is a result, not a broken test. Never re-scored.
(Corrected 30 Oct after run 2: this said "every registered expectation", and PFC-1a missed with exit 0.)
"""
import csv
import gzip
import json
import os
import statistics
import sys

OUT = sys.argv[1]
HW = "B42N6000PBCD7AG9"
NO_STATUS = "no /go2/robot_status accepted yet"
STALE_S = 1.0

checks, fails = [], []
facts = {}          # numbers for the PFC verdicts


def check(phase, label, ok, detail=""):
    checks.append((phase, label))
    print("  %s %-4s %s%s" % ("ok  " if ok else "FAIL", phase, label, "" if not detail else "  -- " + str(detail)[:400]))
    if not ok:
        fails.append((phase, label))
    return ok


def info(phase, text):
    print("  info %-4s %s" % (phase, text))


def jl(path):
    """A .jsonl, or its .jsonl.gz (committed runs are gzipped: 16.5 MB -> 0.8 MB, 30 Oct)."""
    if os.path.exists(path):
        opener = open
    elif os.path.exists(path + ".gz"):
        path, opener = path + ".gz", gzip.open
    else:
        return []
    rows = []
    with opener(path, "rt") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def rcsv(path):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return list(csv.DictReader(f))


def tfile(P, name):
    p = os.path.join(P, name)
    return float(open(p).read().strip()) if os.path.exists(p) else None


def collapse(seq):
    out = []
    for x in seq:
        if not out or out[-1] != x:
            out.append(x)
    return out


def st(stamp):
    return "%d.%09d" % (int(stamp[0]), int(stamp[1]))


def exact(a, b):
    """Field-exact: floats by repr (so -0.0 != 0.0), everything else by ==, recursively."""
    if isinstance(a, float) or isinstance(b, float):
        return isinstance(a, (int, float)) and isinstance(b, (int, float)) and repr(float(a)) == repr(float(b))
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(exact(x, y) for x, y in zip(a, b))
    return a == b


class Phase(object):

    def __init__(self, name):
        self.n = name
        self.P = os.path.join(OUT, name)
        self.ev = rcsv(os.path.join(self.P, "check", "eval.csv")) or []
        self.sent = rcsv(os.path.join(self.P, "check", "sent.csv"))
        self.r1 = jl(os.path.join(self.P, "ros1.jsonl"))
        self.r2 = jl(os.path.join(self.P, "ros2.jsonl"))
        txt = os.path.join(self.P, "check.txt")
        self.check_txt = open(txt).read() if os.path.exists(txt) else ""
        self.rc = None
        for line in self.check_txt.splitlines():
            if line.startswith("rc="):
                self.rc = int(line[3:])
        self.valid = [r for r in self.ev if r["reason"] != NO_STATUS]
        for r in self.ev:
            r["wall"] = float(r["wall"])
            r["received"] = r["received"] == "True"
        self.seq = collapse([r["cls"] for r in self.valid])
        self.requests = [r for r in self.r2 if r["kind"] == "request"]
        self.events = dict((r["what"], r["t"]) for r in self.r2 if r["kind"] == "event")
        self.self_sent = [r for r in (self.sent or []) if r["source"] == "self"]

    def present(self):
        return os.path.isdir(self.P)

    def first(self, pred, t0=-1e18):
        for r in self.valid:
            if r["wall"] >= t0 and pred(r):
                return r
        return None

    def nonmove(self):
        return [r["name"] for r in self.requests if r["name"] != "MOVE"]

    def ros2_status(self):
        out = []
        for r in self.r2:
            if r["kind"] == "robot_status" and r["status"] and r["status"][0]["hardware_id"] == HW:
                out.append((r["t"], dict(r["status"][0]["values"]), r))
        return out


def common_send_checks(p, expect_seq):
    check(p.n, "checker exited 0", p.rc == 0, p.rc)
    check(p.n, "label sequence %s" % ",".join(expect_seq), p.seq == expect_seq, p.seq)
    check(p.n, "RECEIVED at some point", any(r["received"] for r in p.valid))


def stamps_checks(p, all_delivered=True):
    """PFC-1, the stamp half: exact to the nanosecond across ROS 1 -> ROS 2 and back."""
    sent = set(r["stamp"] for r in p.self_sent)
    on2 = [st(r["stamp"]) for r in p.r2 if r["kind"] == "cmd"]
    ok_sub = check(p.n, "PFC-1: every stamp seen on ROS 2 is one the checker sent (%d seen)" % len(on2),
                   len(on2) > 0 and set(on2) <= sent, sorted(set(on2) - sent)[:3])
    ok_all = True
    if all_delivered:
        ok_all = check(p.n, "PFC-1: every stamp the checker sent arrived on ROS 2", sent <= set(on2),
                       sorted(sent - set(on2))[:3])
    echoes = set(r["echo"] for r in p.ev if r["echo"])
    ok_echo = check(p.n, "PFC-1: every echo the checker saw is a stamp it sent (%d distinct)" % len(echoes),
                    len(echoes) > 0 and echoes <= sent, sorted(echoes - sent)[:3])
    facts.setdefault("pfc1_phases", []).append(ok_sub and ok_all and ok_echo)
    lats = {}
    for r in p.ev:
        if r["echo"] and r["echo_latency_s"]:
            lats[r["echo"]] = float(r["echo_latency_s"])
    facts.setdefault("echo_lat", []).extend(lats.values())


def main():
    print("phase1 stage 2 analysis: %s" % OUT)
    names = ["C", "X", "S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8a", "S8b", "S8c", "S9"]
    ph = dict((n, Phase(n)) for n in names)
    ran = [n for n in names if ph[n].present()]
    print("phases present: %s" % " ".join(ran))

    # ---------------------------------------------------------------- C ---
    p = ph["C"]
    if p.present():
        print("C    control: ros1_bridge on domain 42")
        check("C", "checker refused (exit 2)", p.rc == 2, p.rc)
        check("C", "nothing sent (sent.csv header only)", p.sent is not None and len(p.sent) == 0,
              None if p.sent is None else len(p.sent))
        check("C", "ROS 2 saw no /go2/cmd_vel_stamped", not any(r["kind"] == "cmd" for r in p.r2))
        check("C", "ROS 1 saw no /go2/robot_status", not any(r["kind"] == "robot_status" for r in p.r1))
        n2 = len(p.ros2_status())
        check("C", "...while the Go2 side's own status WAS alive (%d on ROS 2): silence is the bridge" % n2, n2 >= 20, n2)
        check("C", "every checker reading UNKNOWN 'no status accepted yet'",
              len(p.ev) > 0 and all(r["cls"] == "UNKNOWN" and r["reason"] == NO_STATUS for r in p.ev), collapse(
                  [r["cls"] for r in p.ev]))

    # ---------------------------------------------------------------- X ---
    p = ph["X"]
    if p.present():
        print("X    PFC-0 / PFC-1 injections")
        probe = next((r["node"] for r in p.r1 if r["kind"] == "start"), "/phase1_ros1_probe")
        ok0, ok0a = [], []
        s1 = [r for r in p.r1 if r["kind"] == "pfc0_sent"]
        for s in [r for r in s1 if r["what"] == "cmd"]:
            got = [r for r in p.r2 if r["kind"] == "cmd" and r["stamp"] == s["header"]["stamp"]]
            ok0.append(check("X", "ROS1->ROS2 TwistStamped %s: field-exact" % st(s["header"]["stamp"]),
                             len(got) >= 1 and all(g["frame_id"] == s["header"]["frame_id"] and exact(g["twist"], s["twist"])
                                                   for g in got), got[:1] or "never arrived"))
            ok0a.append(check("X", "  ...arrived exactly once (PFC-0a)", len(got) == 1, len(got)))
            back = [r for r in p.r1 if r["kind"] == "cmd" and r["header"]["stamp"] == s["header"]["stamp"]
                    and r["callerid"] != probe]
            ok0a.append(check("X", "  ...and did not come back to ROS 1 (PFC-0a)", not back, [b["callerid"] for b in back]))
        for s in [r for r in s1 if r["what"] == "diag"]:
            got = [r for r in p.r2 if r["kind"] == "robot_status" and r["stamp"] == s["header"]["stamp"]]
            want = {"level": s["level"], "name": "go2", "message": s["message"], "hardware_id": s["hardware_id"],
                    "values": s["values"]}
            ok0.append(check("X", "ROS1->ROS2 DiagnosticArray %s: field-exact" % st(s["header"]["stamp"]),
                             len(got) >= 1 and all(len(g["status"]) == 1 and exact(
                                 [g["status"][0][k] for k in sorted(want)], [want[k] for k in sorted(want)]) for g in got),
                             got[:1] or "never arrived"))
            ok0a.append(check("X", "  ...arrived exactly once (PFC-0a)", len(got) == 1, len(got)))
            back = [r for r in p.r1 if r["kind"] == "robot_status" and r["header"]["stamp"] == s["header"]["stamp"]
                    and r["callerid"] != probe]
            ok0a.append(check("X", "  ...and did not come back to ROS 1 (PFC-0a)", not back, [b["callerid"] for b in back]))
        s2 = [r for r in p.r2 if r["kind"] == "pfc0_sent"]
        for s in s2:
            got = [r for r in p.r1 if r["kind"] == "cmd" and r["header"]["stamp"] == s["stamp"] and r["callerid"] != probe]
            ok0.append(check("X", "ROS2->ROS1 TwistStamped %s: field-exact" % st(s["stamp"]),
                             len(got) >= 1 and all(g["header"]["frame_id"] == s["frame_id"] and exact(g["twist"], s["twist"])
                                                   for g in got), got[:1] or "never arrived"))
            ok0a.append(check("X", "  ...arrived exactly once (PFC-0a)", len(got) == 1, len(got)))
            own = [r for r in p.r2 if r["kind"] == "cmd" and r["stamp"] == s["stamp"]]
            ok0a.append(check("X", "  ...and did not come back to ROS 2 (own delivery <= 1) (PFC-0a)", len(own) <= 1, len(own)))
        check("X", "all injections were sent (3 + 2 from ROS 1, 3 from ROS 2)",
              len(s1) == 5 and len(s2) == 3, (len(s1), len(s2)))
        # ROS 2 -> ROS 1: every go2_state_node status while the probe was hearing them.
        r1s = [r for r in p.r1 if r["kind"] == "robot_status" and r["callerid"] != probe
               and r["status"] and r["status"][0]["hardware_id"] == HW]
        r2s = [r for t, v, r in p.ros2_status()]
        if r1s:
            t_lo, t_hi = r1s[0]["t"], r1s[-1]["t"]
            window = [r for r in r2s if t_lo - 0.05 <= r["t"] <= t_hi - 0.05]
            by_stamp = {}
            for r in r1s:
                by_stamp.setdefault(tuple(r["header"]["stamp"]), []).append(r)
            matched = [(r, by_stamp[tuple(r["stamp"])]) for r in window if tuple(r["stamp"]) in by_stamp]
            frac = len(matched) / float(len(window)) if window else 0.0
            exact_all = all(len(b) == 1 and exact(b[0]["status"], a["status"]) for a, b in matched)
            ok0b1 = check("X", "PFC-0b: %d/%d ROS 2 statuses seen on ROS 1 (%.1f %%, >= 95 %%)" % (len(matched), len(window), 100 * frac),
                          frac >= 0.95)
            ok0b2 = check("X", "PFC-0b: ...each exactly once and field-exact", exact_all and len(matched) > 0)
            facts["pfc0b"] = ok0b1 and ok0b2
            ok0.append(ok0b2)
            dup2 = len(r2s) - len(set(tuple(r["stamp"]) for r in r2s))
            ok0a.append(check("X", "PFC-0a: no ROS 2 status stamp seen twice on ROS 2 (no 2->1->2 loop)", dup2 == 0, dup2))
        else:
            facts["pfc0b"] = False
            ok0.append(check("X", "ROS 1 heard go2_state_node's status at all", False))
        # Odom / IMU: not in a registered prediction. Reported, never scored.
        for kind, keys in (("odom", ("child_frame_id", "pose", "twist", "pose_cov", "twist_cov")),
                           ("imu", ("orientation", "angular_velocity", "linear_acceleration", "orientation_cov"))):
            a = dict((tuple(r["stamp"]), r) for r in p.r2 if r["kind"] == kind)
            b = [r for r in p.r1 if r["kind"] == kind and r["callerid"] != probe]
            m = [(a[tuple(r["header"]["stamp"])], r) for r in b if tuple(r["header"]["stamp"]) in a]
            ex = sum(1 for x, y in m if x["frame_id"] == y["header"]["frame_id"] and all(exact(x[k], y[k]) for k in keys))
            info("X", "%s ROS2->ROS1 (not registered): %d on ROS 1, %d matched by stamp, %d field-exact" % (kind, len(b), len(m), ex))
        facts["pfc0"] = all(ok0) and bool(ok0)
        facts["pfc0a"] = all(ok0a) and bool(ok0a)
        # PFC-1a: header.seq on what ros1_bridge publishes into ROS 1
        seqs = [r["header"]["seq"] for r in p.r1 if r["kind"] in ("cmd", "robot_status", "odom", "imu") and r["callerid"] != probe]
        callers = sorted(set(r["callerid"] for r in p.r1 if r.get("callerid") and r["callerid"] != probe))
        zero = sum(1 for s in seqs if s == 0)
        info("X", "bridged-in callerid(s): %s; seq: %d messages, %d zero, min %s max %s"
             % (callers, len(seqs), zero, min(seqs) if seqs else "-", max(seqs) if seqs else "-"))
        facts["pfc1a"] = bool(seqs) and zero == len(seqs)
        facts["pfc1a_detail"] = "%d/%d zero, range %s..%s" % (zero, len(seqs), min(seqs) if seqs else "-", max(seqs) if seqs else "-")
        sent_seq = [r["header"]["seq"] for r in s1]
        info("X", "seq rospy put on the ROS 1 injections: %s (ROS 2's Header has no seq field)" % sent_seq)

    # --------------------------------------------------------------- S1 ---
    p = ph["S1"]
    if p.present():
        print("S1   no command, robot lying")
        check("S1", "checker exited 0", p.rc == 0, p.rc)
        check("S1", "label sequence STOPPED", p.seq == ["STOPPED"], p.seq)
        check("S1", "never RECEIVED", not any(r["received"] for r in p.valid))
        check("S1", "warn names bridge_waiting", len(p.valid) > 0 and all("bridge_waiting" in r["warn"] for r in p.valid),
              collapse([r["warn"] for r in p.valid])[:3])

    # --------------------------------------------------------------- S2 ---
    p = ph["S2"]
    if p.present():
        print("S2   zero commands")
        common_send_checks(p, ["STOPPED"])
        check("S2", "no MOVE in the bridge's requests", not any(r["name"] == "MOVE" for r in p.requests),
              collapse([r["name"] for r in p.requests]))
        stamps_checks(p)

    # ------------------------------------------------------------ S3 / S4 ---
    for n, what in (("S3", "0.3 m/s straight"), ("S4", "turn 1.0 rad/s")):
        p = ph[n]
        if not p.present():
            continue
        print("%-4s %s" % (n, what))
        common_send_checks(p, ["STOPPED", "MOVING", "STOPPED"])
        check(n, "RECEIVED while MOVING", any(r["received"] and r["cls"] == "MOVING" for r in p.valid))
        stamps_checks(p)
        t0 = float(p.self_sent[0]["wall"]) if p.self_sent else None
        tl = float(p.self_sent[-1]["wall"]) if p.self_sent else None
        rm = p.first(lambda r: r["cls"] == "MOVING", t0 or 0)
        rs = p.first(lambda r: r["cls"] == "STOPPED", tl or 0)
        if rm and rs:
            info(n, "MOVING %.2f s after the first send; STOPPED %.2f s after the last (model 0.74 / 1.27; PFC-8 is the robot's)"
                 % (rm["wall"] - t0, rs["wall"] - tl))

    # --------------------------------------------------------------- S5 ---
    p = ph["S5"]
    if p.present():
        print("S5   hexapod publisher stops mid-walk (walk_03_silent)")
        common_send_checks(p, ["STOPPED", "MOVING", "STOPPED"])
        check("S5", "never UNKNOWN", "UNKNOWN" not in p.seq, p.seq)
        check("S5", "exactly one STOPMOVE (the watchdog)", p.nonmove().count("STOPMOVE") == 1, p.nonmove())
        stamps_checks(p)

    # --------------------------------------------------------------- S6 ---
    p = ph["S6"]
    if p.present():
        print("S6   ros1_bridge SIGKILLed mid-walk")
        kb, ka = tfile(p.P, "t_kill_before.txt"), tfile(p.P, "t_kill_after.txt")
        check("S6", "checker exited 0", p.rc == 0, p.rc)
        check("S6", "label sequence STOPPED,MOVING,UNKNOWN", p.seq == ["STOPPED", "MOVING", "UNKNOWN"], p.seq)
        if kb is not None:
            after = [r for r in p.valid if r["wall"] > kb]
            check("S6", "⛔ never STOPPED after the kill", not any(r["cls"] == "STOPPED" for r in after),
                  collapse([r["cls"] for r in after]))
            ru = p.first(lambda r: r["cls"] == "UNKNOWN", kb)
            lat_b = None if ru is None else ru["wall"] - kb
            lat_a = None if ru is None else ru["wall"] - ka
            facts["s6_unknown"] = (lat_a, lat_b)
            facts["pfc7_s6"] = check("S6", "PFC-7: UNKNOWN <= STALE_S + 0.2 s after the kill (scored from t_kill_before)",
                                     ru is not None and lat_b <= STALE_S + 0.2, (lat_a, lat_b))
            if ru is not None:
                info("S6", "UNKNOWN %.3f s after the kill returned, %.3f s after it was issued" % (lat_a, lat_b))
            req_after = [r["name"] for r in p.requests if r["t"] > kb and r["name"] != "MOVE"]
            check("S6", "Go2 side: one STOPMOVE after the kill (the watchdog)", req_after.count("STOPMOVE") == 1, req_after)
            st2 = p.ros2_status()
            last = st2[-1][1] if st2 else {}
            check("S6", "Go2 side: body at rest at the end (motion STOPPED, speed < 0.01)",
                  last.get("motion") == "STOPPED" and abs(float(last.get("speed_mps", "9"))) < 0.01,
                  (last.get("motion"), last.get("speed_mps")))
        stamps_checks(p, all_delivered=False)
        if kb is not None:
            sent_before = set(r["stamp"] for r in p.self_sent if float(r["wall"]) < kb - 0.05)
            on2 = set(st(r["stamp"]) for r in p.r2 if r["kind"] == "cmd")
            check("S6", "PFC-1: every stamp sent before the kill (-50 ms) arrived on ROS 2", sent_before <= on2,
                  sorted(sent_before - on2)[:3])

    # --------------------------------------------------------------- S7 ---
    p = ph["S7"]
    if p.present():
        print("S7   handset yield mid-walk")
        common_send_checks(p, ["STOPPED", "MOVING", "FAULT"])
        th = p.events.get("handset keys 288")
        rf = p.first(lambda r: r["cls"] == "FAULT" and r["reason"] == "bridge_yielded", th or 0)
        check("S7", "FAULT reason bridge_yielded", rf is not None)
        if rf is not None:
            tail = [r for r in p.valid if r["wall"] >= rf["wall"]]
            check("S7", "latched: FAULT bridge_yielded to the end",
                  all(r["cls"] == "FAULT" and r["reason"] == "bridge_yielded" for r in tail),
                  collapse([(r["cls"], r["reason"]) for r in tail]))
            facts["s7_fault"] = rf["wall"] - th
            info("S7", "FAULT(bridge_yielded) %.3f s after the handset message (model 0.25 s)" % facts["s7_fault"])
        stamps_checks(p, all_delivered=True)

    # -------------------------------------------------------------- S8a ---
    p = ph["S8a"]
    if p.present():
        print("S8a  RR_hip motor mode 0")
        check("S8a", "checker exited 0", p.rc == 0, p.rc)
        check("S8a", "label sequence STOPPED,FAULT", p.seq == ["STOPPED", "FAULT"], p.seq)
        check("S8a", "FAULT reason motor_mode:RR_hip",
              all(r["reason"] == "motor_mode:RR_hip" for r in p.valid if r["cls"] == "FAULT") and "FAULT" in p.seq)

    # -------------------------------------------------------------- S8b ---
    p = ph["S8b"]
    if p.present():
        print("S8b  RR_hip 51 C mid-walk")
        common_send_checks(p, ["STOPPED", "MOVING", "FAULT"])
        lastr = p.valid[-1]["reason"] if p.valid else ""
        check("S8b", "final reason has bridge_aborted AND rear_hip_hot:51",
              "bridge_aborted" in lastr and "rear_hip_hot:51" in lastr, lastr)
        check("S8b", "non-Move requests are STOPMOVE, STANDDOWN", p.nonmove() == ["STOPMOVE", "STANDDOWN"], p.nonmove())
        stamps_checks(p)

    # -------------------------------------------------------------- S8c ---
    p = ph["S8c"]
    if p.present():
        print("S8c  /sportmodestate frozen mid-walk")
        check("S8c", "checker exited 0", p.rc == 0, p.rc)
        ok_seq = p.seq in (["STOPPED", "MOVING", "UNKNOWN", "FAULT"], ["STOPPED", "MOVING", "FAULT"])
        check("S8c", "label sequence STOPPED,MOVING,(UNKNOWN),FAULT", ok_seq, p.seq)
        check("S8c", "RECEIVED at some point", any(r["received"] for r in p.valid))
        lastr = p.valid[-1]["reason"] if p.valid else ""
        check("S8c", "final reason bridge_aborted:/sportmodestate silent", "bridge_aborted:/sportmodestate silent" in lastr, lastr)
        info("S8c", "UNKNOWN %s sampled" % ("was" if "UNKNOWN" in p.seq else "was NOT"))
        stamps_checks(p)

    # --------------------------------------------------------------- S9 ---
    p = ph["S9"]
    if p.present():
        print("S9   replayed stamp")
        check("S9", "checker exited 0", p.rc == 0, p.rc)
        check("S9", "label sequence STOPPED", p.seq == ["STOPPED"], p.seq)
        reps = [r for r in p.self_sent if r["accepted_by_checker"] == "False"]
        check("S9", "replays sent: 10 +/- 1", 9 <= len(reps) <= 11, len(reps))
        bst = [r["status"] for r in p.r2 if r["kind"] == "bridge_status"]
        dropped = bst[-1]["counts"].get("dropped_nonmonotonic") if bst else None
        check("S9", "bridge dropped_nonmonotonic == replays sent (%s vs %d)" % (dropped, len(reps)), dropped == len(reps))
        ech = [v["cmd_last_stamp"] for t, v, r in p.ros2_status() if v.get("cmd_last_stamp")]
        tup = [tuple(int(x) for x in e.split(".")) for e in ech]
        check("S9", "the echo never moves backwards (%d echoes on ROS 2)" % len(tup), all(a <= b for a, b in zip(tup, tup[1:])))
        if reps:
            tr = float(reps[0]["wall"])
            late = [r for r in p.valid if r["wall"] >= tr + 0.15 and r["received"]]
            check("S9", "never RECEIVED after the replay starts (+0.15 s)", not late, len(late))
        stamps_checks(p)

    # ----------------------------------------------------------- verdicts ---
    print()
    print("PREDICTIONS (registered: phase1_go2_feedback_plan.md §5 and §9 stage 2)")

    def verdict(pid, ok, text):
        print("  %-6s %s  %s" % (pid, "MATCHED    " if ok else "NOT MATCHED", text))

    s_phases = ["S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8a", "S8b", "S8c", "S9"]
    if "X" in ran:
        verdict("PFC-0", facts.get("pfc0", False), "TwistStamped + DiagnosticArray both ways, field-exact")
        verdict("PFC-0a", facts.get("pfc0a", False), "each injection exactly once, no loop back")
        verdict("PFC-0b", facts.get("pfc0b", False), ">= 95 % of ROS 2 statuses on ROS 1, field-exact")
        verdict("PFC-1a", facts.get("pfc1a", False), "seq = 0 on everything ros1_bridge publishes into ROS 1 (%s)"
                % facts.get("pfc1a_detail", "-"))
    if facts.get("pfc1_phases"):
        verdict("PFC-1", all(facts["pfc1_phases"]), "stamps exact to the ns, ROS 1 -> ROS 2 and back in the echo (%d phases)"
                % len(facts["pfc1_phases"]))
    if all(n in ran for n in s_phases):
        bad = sorted(set(ph_ for ph_, _ in fails if ph_ in s_phases))
        verdict("PFC-7", not bad and facts.get("pfc7_s6", False),
                "S1-S9 labels 100 %% as registered; S6 UNKNOWN within 1.2 s, never STOPPED%s"
                % ("" if not bad else "  (failing: %s)" % ", ".join(bad)))
    else:
        print("  PFC-7  not scored: needs every S phase (ran: %s)" % " ".join(n for n in s_phases if n in ran))
    lat = facts.get("echo_lat", [])
    if lat:
        med = statistics.median(lat)
        verdict("PFC-L", med <= 0.40, "echo latency median %.3f s (n=%d, min %.3f, max %.3f; registered <= 0.40)"
                % (med, len(lat), min(lat), max(lat)))
    if "s7_fault" in facts:
        verdict("PFC-L", facts["s7_fault"] <= 0.45, "FAULT(bridge_yielded) %.3f s after the handset (registered <= 0.45)"
                % facts["s7_fault"])
    if "s6_unknown" in facts and facts["s6_unknown"][0] is not None:
        a, b = facts["s6_unknown"]
        verdict("PFC-L", a >= 0.8 and b <= 1.2, "UNKNOWN %.3f-%.3f s after the S6 kill (registered 0.8-1.2)" % (a, b))
    print()
    print("%d checks, %d failure(s)" % (len(checks), len(fails)))
    for ph_, label in fails:
        print("  FAIL %s %s" % (ph_, label))
    return 1 if fails or not checks else 0


if __name__ == "__main__":
    sys.exit(main())
