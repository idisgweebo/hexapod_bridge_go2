#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Hexapod-side Go2 feedback checker (phase 1). rospy shell; every decision is in
go2_feedback_core.py.

Plan: phase1_go2_feedback_plan.md. Runs on the Nano (Melodic; Python 2.7 or 3) or on
any ROS 1 client of the hexapod's master. Copied to the Nano by scp; ⛔ NOT installed
into the vendor catkin workspace.

  LISTEN (default)  subscribes /go2/robot_status and /go2/cmd_vel_stamped (to learn the
                    stamps someone else -- e.g. `rostopic pub` -- sends). Publishes NOTHING.
  --send PROFILE    publishes /go2/cmd_vel_stamped itself at 10 Hz, then listens.
                    ⛔ Whether that moves the Go2 is decided by go2_cmd_bridge (armed or
                    dry). An ARMED run is Doug's to start. --expect-armed is required and
                    must match the bridge's own report before anything is sent.

    profile        what it sends                                           scenario
    zero           vx 0 for 2.5 s, then three zeros                        S2
    walk_03        0.3 m/s for 2.5 s, then three zeros                     S3
    turn_10        1.0 rad/s for 2.0 s, then three zeros                   S4
    walk_03_silent 0.3 m/s for 1.5 s, then NOTHING (the bridge's watchdog) S5
    replay         vx 0 for 1.0 s, then the stamp from 3 sends back, 1 s   S9

Every evaluation (10 Hz) goes to OUTDIR/eval.csv; every send to OUTDIR/sent.csv.
A change of reading is printed; UNKNOWN is printed loudly.

    go2_feedback_check.py [--send PROFILE --expect-armed true|false] [--duration S] [--outdir DIR]
"""
from __future__ import division, print_function

import argparse
import csv
import os
import sys
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import go2_feedback_core as fc                                    # noqa: E402

try:
    monotonic = time.monotonic
except AttributeError:          # Python 2.7: times(2) elapsed ticks, monotonic on Linux, 10 ms resolution
    def monotonic():
        return os.times()[4]

PROFILES = {
    # name: [(vx, wz, seconds), ...], send trailing zeros?
    "zero": ([(0.0, 0.0, 2.5)], True),
    "walk_03": ([(0.3, 0.0, 2.5)], True),
    "turn_10": ([(0.0, 1.0, 2.0)], True),
    "walk_03_silent": ([(0.3, 0.0, 1.5)], False),
    "replay": ([(0.0, 0.0, 1.0)], False),     # the replay second is handled below
}
SEND_HZ = 10.0
EVAL_HZ = 10.0
EVAL_FIELDS = ["wall", "mono", "received", "cls", "reason", "echo", "echo_latency_s", "status_age_s", "skew_s",
               "motion", "fault", "warn", "bridge_state", "bridge_armed", "bridge_input", "level"]


def open_csv(path):
    if sys.version_info[0] < 3:
        return open(path, "wb")
    return open(path, "w", newline="")


class Checker(object):

    def __init__(self, args):
        self.args = args
        self.core = fc.FeedbackChecker(stale_s=args.stale)
        os.makedirs(args.outdir)
        self.eval_f = open_csv(os.path.join(args.outdir, "eval.csv"))
        self.eval_w = csv.DictWriter(self.eval_f, EVAL_FIELDS)
        self.eval_w.writeheader()
        self.sent_f = open_csv(os.path.join(args.outdir, "sent.csv"))
        self.sent_w = csv.writer(self.sent_f)
        self.sent_w.writerow(["wall", "mono", "stamp", "vx", "wz", "accepted_by_checker", "source"])
        self.last_reading = None
        self.last_stamp = None
        self.n_bumped = 0
        rospy.Subscriber("/go2/robot_status", DiagnosticArray, self.on_status, queue_size=5)
        self.pub = None
        if args.send:
            self.pub = rospy.Publisher("/go2/cmd_vel_stamped", TwistStamped, queue_size=1)
        else:
            rospy.Subscriber("/go2/cmd_vel_stamped", TwistStamped, self.on_heard_send, queue_size=20)

    # ------------------------------------------------------------- inputs ---
    def on_status(self, m):
        mono, wall = monotonic(), time.time()
        for st in m.status:
            if st.name != "go2":
                continue
            level = st.level if isinstance(st.level, int) else ord(st.level)
            values = dict((kv.key, kv.value) for kv in st.values)
            self.core.on_status(mono, m.header.stamp.secs, m.header.stamp.nsecs, st.hardware_id,
                                level, values, wall=wall)

    def on_heard_send(self, m):
        s = m.header.stamp
        ok = self.core.on_sent(monotonic(), s.secs, s.nsecs)
        self.sent_w.writerow(["%.6f" % time.time(), "%.6f" % monotonic(), fc.fmt_stamp(s.secs, s.nsecs),
                              m.twist.linear.x, m.twist.angular.z, ok, "heard"])

    # -------------------------------------------------------------- send ---
    def next_stamp(self):
        """rospy.Time.now(), forced strictly increasing. If the clock steps back (the
        Nano's dead RTC, then chrony), stamps continue 1 ns apart and it is counted:
        go2_cmd_bridge refuses non-increasing stamps (D3), so a backwards stamp would
        silently stop every command."""
        t = rospy.Time.now()
        if self.last_stamp is not None and (t.secs, t.nsecs) <= (self.last_stamp.secs, self.last_stamp.nsecs):
            t = self.last_stamp + rospy.Duration(0, 1)
            self.n_bumped += 1
        self.last_stamp = t
        return t

    def send(self, vx, wz, stamp=None):
        m = TwistStamped()
        m.header.stamp = stamp if stamp is not None else self.next_stamp()
        m.header.frame_id = "base_link"
        m.twist.linear.x, m.twist.angular.z = vx, wz
        self.pub.publish(m)
        ok = self.core.on_sent(monotonic(), m.header.stamp.secs, m.header.stamp.nsecs)
        self.sent_w.writerow(["%.6f" % time.time(), "%.6f" % monotonic(),
                              fc.fmt_stamp(m.header.stamp.secs, m.header.stamp.nsecs), vx, wz, ok, "self"])
        return m.header.stamp

    # ------------------------------------------------------------ evaluate ---
    def evaluate(self):
        r = self.core.evaluate(monotonic())
        row = dict((k, r.get(k, "")) for k in EVAL_FIELDS)
        row["wall"], row["mono"] = "%.6f" % time.time(), "%.6f" % monotonic()
        self.eval_w.writerow(row)
        reading = (r["received"], r["cls"], r["reason"])
        if reading != self.last_reading:
            lat = r["echo_latency_s"]
            line = "%s  %-8s %s%s%s" % (time.strftime("%H:%M:%S"), r["cls"], "RECEIVED " if r["received"] else "",
                                        r["reason"], "  (echo latency %.3f s)" % lat if lat is not None else "")
            if r["cls"] == fc.UNKNOWN:
                line = "!!!!! " + line + "  !!!!!"
            print(line)
            sys.stdout.flush()
            self.last_reading = reading
        return r

    def close(self):
        self.eval_f.close()
        self.sent_f.close()


def wait_until(chk, until_mono):
    rate = rospy.Rate(EVAL_HZ)
    while not rospy.is_shutdown() and monotonic() < until_mono:
        chk.evaluate()
        rate.sleep()


def preflight(chk, args):
    """Nothing is sent until a fresh status shows the bridge on the stamped input,
    armed exactly as expected, and in IDLE (or WAITING with --allow-waiting)."""
    deadline = monotonic() + 10.0
    why = "no status yet"
    while not rospy.is_shutdown() and monotonic() < deadline:
        r = chk.evaluate()
        if r["status_age_s"] is not None and r["status_age_s"] <= args.stale:
            if r["bridge_input"] != "twist_stamped":
                why = "bridge input is %r, not 'twist_stamped'" % r["bridge_input"]
            elif r["bridge_armed"] != args.expect_armed:
                why = "bridge reports armed=%r, expected %r" % (r["bridge_armed"], args.expect_armed)
            elif r["bridge_state"] == "IDLE" or (args.allow_waiting and r["bridge_state"] == "WAITING"):
                return True, "bridge %s, armed=%s" % (r["bridge_state"], r["bridge_armed"])
            else:
                why = "bridge state %r" % r["bridge_state"]
        rospy.sleep(0.1)
    return False, why


def run_profile(chk, name):
    steps, zeros = PROFILES[name]
    period = 1.0 / SEND_HZ
    stamps = []
    for vx, wz, secs in steps:
        end = monotonic() + secs
        while not rospy.is_shutdown() and monotonic() < end:
            stamps.append(chk.send(vx, wz))
            wait_until(chk, monotonic() + period)
    if name == "replay" and len(stamps) >= 3:
        old = stamps[-3]
        end = monotonic() + 1.0
        while not rospy.is_shutdown() and monotonic() < end:
            chk.send(0.0, 0.0, stamp=old)
            wait_until(chk, monotonic() + period)
    if zeros:
        for _ in range(3):
            chk.send(0.0, 0.0)
            wait_until(chk, monotonic() + period)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--send", choices=sorted(PROFILES))
    ap.add_argument("--expect-armed", choices=["true", "false"])
    ap.add_argument("--allow-waiting", action="store_true", help="send even if the bridge is WAITING (dry tests)")
    ap.add_argument("--duration", type=float, default=10.0, help="listen time (after a profile, if any)")
    ap.add_argument("--stale", type=float, default=1.0, help="STALE_S (D5)")
    ap.add_argument("--outdir", default=os.path.expanduser(time.strftime("~/go2_feedback_%Y%m%dT%H%M%SZ", time.gmtime())))
    args = ap.parse_args(rospy.myargv()[1:])
    if args.send and args.expect_armed is None:
        ap.error("--send needs --expect-armed true|false: say what you think the bridge is")

    rospy.init_node("go2_feedback_check", anonymous=True)
    chk = Checker(args)
    print("go2_feedback_check: %s; stale %.2f s; logging to %s"
          % ("SEND %s (expect armed=%s)" % (args.send, args.expect_armed) if args.send else "LISTEN only",
             args.stale, args.outdir))
    rc = 0
    try:
        if args.send:
            ok, why = preflight(chk, args)
            if not ok:
                print("REFUSING to send: %s" % why)
                rc = 2
            else:
                print("preflight ok: %s -- sending %s" % (why, args.send))
                run_profile(chk, args.send)
        if rc == 0:
            wait_until(chk, monotonic() + args.duration)
    finally:
        chk.close()
        lats = sorted(chk.core.echo_latency.values())
        med = lats[len(lats) // 2] if lats else None
        print("done: counts %s; stamps bumped %d; echo latency n=%d median %s"
              % (chk.core.counts, chk.n_bumped, len(lats), "%.3f s" % med if med is not None else "-"))
    return rc


if __name__ == "__main__":
    sys.exit(main())
