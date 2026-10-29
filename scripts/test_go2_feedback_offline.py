#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Offline tests for hexapod/go2_feedback_core.py -- NO ROS, NO robot.

Runs under Python 2.7 (the Nano's Melodic, via the ros:melodic image) AND Python 3,
because the checker has to. Plan: phase1_go2_feedback_plan.md §0, §6.

  1  staleness: no status / stale status -> UNKNOWN, never STOPPED
  2  the echo: history match, the stale_s window, replays, latency
  3  classification: FAULT vs UNKNOWN vs motion
  4  status rejection: hardware_id, schema, non-increasing stamp
"""
from __future__ import print_function

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "hexapod")))
import go2_feedback_core as fc                                    # noqa: E402

n_checks = [0]
fails = []


def check(label, ok, detail=""):
    n_checks[0] += 1
    print(("  ok   " if ok else "  FAIL ") + label + ("" if ok else "  -- " + str(detail)[:300]))
    if not ok:
        fails.append(label)


def status(motion="STOPPED", fault="none", echo="", schema="1", **kw):
    v = {"schema_version": schema, "motion": motion, "fault": fault, "warn": "none",
         "cmd_last_stamp": echo, "bridge_state": "IDLE"}
    v.update(kw)
    return v


print("python %s" % sys.version.split()[0])

print("1    staleness")
c = fc.FeedbackChecker()
r = c.evaluate(10.0)
check("no status yet -> UNKNOWN, not received", r["cls"] == fc.UNKNOWN and not r["received"], r)
c.on_status(10.0, 10, 0, fc.HARDWARE_ID, 0, status())
check("fresh STOPPED status -> STOPPED", c.evaluate(10.5)["cls"] == fc.STOPPED)
r = c.evaluate(11.01)
check("⛔ the same status 1.01 s later -> UNKNOWN, NEVER STOPPED", r["cls"] == fc.UNKNOWN and "stale" in r["reason"], r)
c = fc.FeedbackChecker()
c.on_status(10.0, 2000000000, 0, fc.HARDWARE_ID, 0, status(), wall=1500000000.25)
r = c.evaluate(10.9)
check("staleness is by OUR monotonic arrival clock: a header stamp years off does not matter",
      r["cls"] == fc.STOPPED, r)
check("skew = wall at arrival - header stamp (the Nano's 2018 clock reads hugely negative)",
      abs(r["skew_s"] - (1500000000.25 - 2000000000)) < 1e-6, r["skew_s"])

print("2    the echo")
c = fc.FeedbackChecker()
for i in range(10):                                      # 10 Hz stream, stamps 500.000..500.900
    c.on_sent(20.0 + i * 0.1, 500, i * 100000000)
c.on_status(20.95, 30, 0, fc.HARDWARE_ID, 0, status(motion="MOVING", echo="500.600000000"))
r = c.evaluate(21.0)
check("echo of an earlier send mid-stream -> RECEIVED (history match)", r["received"] and r["cls"] == fc.MOVING, r)
check("echo latency recorded on first sighting (20.95 - 20.6 = 0.35 s)",
      abs(r["echo_latency_s"] - 0.35) < 1e-9, r["echo_latency_s"])
c.on_status(21.15, 30, 200000000, fc.HARDWARE_ID, 0, status(motion="MOVING", echo="500.600000000"))
check("...and NOT overwritten by a later sighting", abs(c.evaluate(21.2)["echo_latency_s"] - 0.35) < 1e-9)
r = c.evaluate(21.61)
check("echoed command sent > stale_s ago -> not received (status itself still fresh)",
      not r["received"] and r["cls"] == fc.MOVING, r)
c.on_status(21.7, 30, 300000000, fc.HARDWARE_ID, 0, status(echo="999.000000000"))
check("echo of a stamp we never sent -> not received", not c.evaluate(21.7)["received"])
c.on_status(21.8, 30, 400000000, fc.HARDWARE_ID, 0, status(echo=""))
check("empty echo -> not received", not c.evaluate(21.8)["received"])

c = fc.FeedbackChecker()
check("first send accepted", c.on_sent(1.0, 700, 5))
c.on_status(1.3, 40, 0, fc.HARDWARE_ID, 0, status(echo="700.000000005"))
check("S9 setup: echoed -> received", c.evaluate(1.3)["received"])
check("S9: re-sending the SAME stamp is refused as a replay", not c.on_sent(1.4, 700, 5)
      and c.counts["replay_sent"] == 1)
check("S9: an OLDER stamp is refused too", not c.on_sent(1.5, 699, 0) and c.counts["replay_sent"] == 2)
c.on_status(1.6, 40, 100000000, fc.HARDWARE_ID, 0, status(echo="700.000000005"))
check("⛔ S9: after a replay send, the old echo does NOT count as received", not c.evaluate(1.6)["received"])
c.on_sent(1.7, 701, 0)
c.on_status(1.9, 40, 200000000, fc.HARDWARE_ID, 0, status(echo="701.000000000"))
check("a fresh send after the replay is received again", c.evaluate(1.9)["received"])
c = fc.FeedbackChecker(history=5)
for i in range(8):
    c.on_sent(1.0 + i * 0.01, 800, i + 1)
check("history is bounded (oldest dropped)", len(c.sent) == 5 and "800.000000001" not in c.sent)

print("3    classification")


def cls_of(**kw):
    c = fc.FeedbackChecker()
    c.on_status(5.0, 5, 0, fc.HARDWARE_ID, 0, status(**kw))
    return c.evaluate(5.0)


check("MOVING passes through", cls_of(motion="MOVING")["cls"] == fc.MOVING)
r = cls_of(motion="MOVING", fault="bridge_yielded")
check("bridge_yielded -> FAULT even while MOVING (handset driving)", r["cls"] == fc.FAULT and r["reason"] == "bridge_yielded", r)
r = cls_of(fault="bridge_aborted:rear hip 50 C >= 50 C")
check("bridge_aborted -> FAULT, reason kept", r["cls"] == fc.FAULT and r["reason"].startswith("bridge_aborted:"), r)
check("bridge_stale -> FAULT (cannot execute)", cls_of(fault="bridge_stale")["cls"] == fc.FAULT)
check("motor_mode -> FAULT", cls_of(fault="motor_mode:RR_hip")["cls"] == fc.FAULT)
r = cls_of(motion="UNKNOWN", fault="sport_stale")
check("sport_stale alone -> UNKNOWN (blind, not a fault)", r["cls"] == fc.UNKNOWN and r["reason"] == "sport_stale", r)
r = cls_of(motion="STOPPED", fault="low_stale")
check("⛔ low_stale with a STOPPED motion -> UNKNOWN, not STOPPED", r["cls"] == fc.UNKNOWN, r)
r = cls_of(motion="UNKNOWN", fault="sport_stale;bridge_aborted:x")
check("a hard fault outranks blindness -> FAULT, reason lists only the hard one",
      r["cls"] == fc.FAULT and r["reason"] == "bridge_aborted:x", r)
check("motion UNKNOWN with no fault (detector warm-up) -> UNKNOWN", cls_of(motion="UNKNOWN")["cls"] == fc.UNKNOWN)
check("motion garbage -> UNKNOWN", cls_of(motion="")["cls"] == fc.UNKNOWN)

print("4    status rejection")
c = fc.FeedbackChecker()
check("wrong hardware_id rejected", not c.on_status(1.0, 1, 0, "SOMEONE_ELSE", 0, status())
      and c.counts["rejected_hardware_id"] == 1 and c.evaluate(1.0)["cls"] == fc.UNKNOWN)
check("wrong schema_version rejected", not c.on_status(1.0, 1, 0, fc.HARDWARE_ID, 0, status(schema="2"))
      and c.counts["rejected_schema"] == 1)
check("good status accepted", c.on_status(1.0, 1, 0, fc.HARDWARE_ID, 0, status()))
check("same header stamp again rejected", not c.on_status(1.5, 1, 0, fc.HARDWARE_ID, 0, status(motion="MOVING")))
check("older header stamp rejected", not c.on_status(1.6, 0, 999999999, fc.HARDWARE_ID, 0, status(motion="MOVING")))
r = c.evaluate(2.05)
check("⛔ rejected statuses do NOT refresh freshness (stale 1.05 s after the good one)",
      r["cls"] == fc.UNKNOWN and c.counts["rejected_stamp_not_newer"] == 2, r)

print()
print("%d checks, %d failure(s)" % (n_checks[0], len(fails)))
if n_checks[0] < 30:
    print("FAIL: fewer checks ran than written -- refusing to report success")
    sys.exit(1)
print("ALL GO2 FEEDBACK CHECKER OFFLINE TESTS PASS" if not fails else "FAILURES:\n  - " + "\n  - ".join(fails))
sys.exit(1 if fails else 0)
