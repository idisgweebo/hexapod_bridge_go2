#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Hexapod-side Go2 feedback checker: the decision logic. PURE -- no ROS, no clock, no I/O.

Plan: phase1_go2_feedback_plan.md §0 (the pass criterion) and D2 (the split): the Go2
side derives motion, faults and the echo; THIS side compares the echo with its own
commands and applies staleness. go2_feedback_check.py is the rospy shell.

Runs on Python 2.7 (Melodic on the Nano -- tagged as inference until read on the Nano)
and on Python 3: no f-strings, no keyword-only arguments, no nonlocal.

ONE READING = (received, cls)
  received  True iff the Go2 side's echo is one of OUR commands, sent within stale_s,
            and our most recent send was not a replay
  cls       MOVING | STOPPED | FAULT | UNKNOWN
            UNKNOWN whenever /go2/robot_status is older than stale_s by OUR clock.
            ⛔ A dead link must never read as STOPPED (the thermal-watch defect, 15 Oct).

WHY THE ECHO IS MATCHED AGAINST A HISTORY, NOT THE LAST SEND
  At 10 Hz the hexapod sends a new stamp every 0.1 s, and the echo crosses the bridge's
  5 Hz status, the 5 Hz robot_status and Wi-Fi. Mid-stream the echo is always a few
  sends behind, so "echo == last sent" would never be true while commanding. What we
  need to know is that the Go2 side accepted a command of ours, recently.

WHY STAMPS ARE COMPARED AS STRINGS
  The echo arrives as "sec.nnnnnnnnn" (a KeyValue string). Formatting our own stamps
  the same way and comparing strings means no float can round two IDs together.
"""
from __future__ import division, print_function

import collections

MOVING, STOPPED, FAULT, UNKNOWN = "MOVING", "STOPPED", "FAULT", "UNKNOWN"

HARDWARE_ID = "B42N6000PBCD7AG9"
SCHEMA_VERSION = "1"
# Inputs whose staleness makes the reading UNKNOWN rather than FAULT: without them the
# Go2 side cannot see the body, which is a gap in knowledge, not a reported fault.
# Every other fault item -- bridge_stale included -- is a FAULT: the Go2 side is
# telling us it cannot or will not execute.
BLIND_FAULTS = ("sport_stale", "low_stale")


def fmt_stamp(sec, nanosec):
    return "%d.%09d" % (int(sec), int(nanosec))


class FeedbackChecker(object):

    def __init__(self, stale_s=1.0, hardware_id=HARDWARE_ID, history=200):
        if not stale_s > 0:
            raise ValueError("stale_s must be > 0")
        self.stale_s = float(stale_s)
        self.hardware_id = hardware_id
        self.sent = collections.OrderedDict()      # "sec.nsec" -> local send time
        self.history = int(history)
        self.max_sent = None                       # (sec, nsec) of our newest send
        self.last_send_replay = False
        self.status_t = None                       # local arrival of the last ACCEPTED status
        self.status_stamp = None                   # its header stamp (sec, nsec)
        self.values = {}
        self.level = None
        self.echo_latency = {}                     # echoed stamp -> first-seen latency, s
        self.skew = None
        self.counts = {"sent": 0, "replay_sent": 0, "status": 0, "rejected_hardware_id": 0,
                       "rejected_schema": 0, "rejected_stamp_not_newer": 0}

    # ------------------------------------------------------------- inputs ---
    def on_sent(self, now, sec, nsec):
        """Record one of our commands. Returns False for a replay (not newer than our
        newest send): it is counted and NOT added, so it can never be echoed as new."""
        stamp = (int(sec), int(nsec))
        if self.max_sent is not None and stamp <= self.max_sent:
            self.counts["replay_sent"] += 1
            self.last_send_replay = True
            return False
        self.max_sent = stamp
        self.last_send_replay = False
        self.sent[fmt_stamp(*stamp)] = now
        while len(self.sent) > self.history:
            self.sent.popitem(last=False)
        self.counts["sent"] += 1
        return True

    def on_status(self, now, stamp_sec, stamp_nsec, hardware_id, level, values, wall=None):
        """One /go2/robot_status DiagnosticStatus. values: dict key -> str.
        `now` is a MONOTONIC clock (staleness must survive a chrony step on the Nano);
        `wall` is the wall clock at arrival, used only for the cross-clock skew.
        Returns True if accepted. A rejected status does NOT refresh freshness."""
        if hardware_id != self.hardware_id:
            self.counts["rejected_hardware_id"] += 1
            return False
        if values.get("schema_version") != SCHEMA_VERSION:
            self.counts["rejected_schema"] += 1
            return False
        stamp = (int(stamp_sec), int(stamp_nsec))
        # The bridge carries /go2/robot_status BOTH ways (parameter_bridge), so a stray
        # ROS 1 publisher could inject a "healthy" status. A non-increasing stamp from
        # the one legitimate publisher cannot happen without a clock step on its host.
        if self.status_stamp is not None and stamp <= self.status_stamp:
            self.counts["rejected_stamp_not_newer"] += 1
            return False
        self.status_t, self.status_stamp = now, stamp
        # Our wall clock minus the Go2 side's, at arrival (so it includes the link
        # delay). Meaningful only once D7 syncs the Nano; reported, never used to classify.
        self.skew = (now if wall is None else wall) - (stamp[0] + stamp[1] / 1e9)
        self.values, self.level = dict(values), level
        self.counts["status"] += 1
        echo = self.values.get("cmd_last_stamp", "")
        if echo in self.sent and echo not in self.echo_latency:
            self.echo_latency[echo] = now - self.sent[echo]
        return True

    # ------------------------------------------------------------- output ---
    def evaluate(self, now):
        """Return a dict: received, cls, reason, and the evidence behind them."""
        v = self.values
        out = {"received": False, "cls": UNKNOWN, "reason": "", "echo": v.get("cmd_last_stamp", ""),
               "echo_latency_s": None, "status_age_s": None, "skew_s": None,
               "motion": v.get("motion", ""), "fault": v.get("fault", ""), "warn": v.get("warn", ""),
               "bridge_state": v.get("bridge_state", ""), "bridge_armed": v.get("bridge_armed", ""),
               "bridge_input": v.get("bridge_input", ""), "level": self.level}
        if self.status_t is None:
            out["reason"] = "no /go2/robot_status accepted yet"
            return out
        age = now - self.status_t
        out["status_age_s"] = age
        out["skew_s"] = self.skew
        if age > self.stale_s:
            out["reason"] = "status stale: %.2f s > %.2f s" % (age, self.stale_s)
            return out

        echo = out["echo"]
        if echo in self.echo_latency:
            out["echo_latency_s"] = self.echo_latency[echo]
        if self.last_send_replay:
            out["received"] = False
        elif echo in self.sent and now - self.sent[echo] <= self.stale_s:
            out["received"] = True

        faults = [f for f in v.get("fault", "none").split(";") if f and f != "none"]
        hard = [f for f in faults if f not in BLIND_FAULTS]
        motion = v.get("motion", "")
        if hard:
            out["cls"], out["reason"] = FAULT, ";".join(hard)
        elif faults or motion not in (MOVING, STOPPED):
            out["cls"] = UNKNOWN
            out["reason"] = ";".join(faults) if faults else "motion %r" % motion
        else:
            out["cls"], out["reason"] = motion, ""
        return out
