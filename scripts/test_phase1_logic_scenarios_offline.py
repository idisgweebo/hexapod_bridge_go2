#!/usr/bin/env python3
"""Phase 1 scenario suite S1-S9 at the LOGIC level: the three pure cores chained in
simulated time. NO ROS, NO container, NO robot.

    hexapod checker  --TwistStamped 10 Hz, link-->  go2_cmd_bridge core (tick 10 Hz,
    status 5 Hz, JSON round-trip)  -->  fake body  -->  go2_adapter state core (sport/low
    50 Hz, status 5 Hz)  --link-->  hexapod checker (evaluated at 10 Hz)

WHAT THIS IS AND IS NOT
  It checks that the three cores agree on their contracts (JSON keys, the stamp's wire
  form, which states mean what) and that S1-S9 come out right given the real rates.
  It does NOT score PFC-7: that prediction is defined on the ROS loopback (stage 2),
  where DDS, ros1_bridge and real message types sit between the cores. The latencies
  printed here are this model's, with an assumed one-way link delay of LINK_S --
  ==inference== about what stage 2 will see, not a measurement.

The fake body is deliberately crude: it follows the last Move with a first-order lag,
and decays to rest when Moves stop for MOVE_LAPSE_S (an assumption shaped like s13's
coast, not a model of it).

Plan: phase1_go2_feedback_plan.md §6.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "go2_cmd_bridge"))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "go2_adapter"))
sys.path.insert(0, os.path.join(ROOT, "hexapod"))
from go2_cmd_bridge import bridge_core as bc                      # noqa: E402
from go2_adapter import state_core as sc                          # noqa: E402
import go2_feedback_core as fc                                    # noqa: E402

DT = 0.005
LINK_S = 0.02            # one-way dogspider delay, ASSUMED (15 Oct Twist max gap was 204.5 ms)
MOVE_LAPSE_S = 0.3
TAU_S = 0.2
STALE_S = 1.0

n_checks = 0
fails = []


def check(label, ok, detail=""):
    global n_checks
    n_checks += 1
    print(("  ok   " if ok else "  FAIL ") + label + ("" if ok else "  -- " + str(detail)[:300]))
    if not ok:
        fails.append(label)


class World(object):

    def __init__(self, code=100, hips=(35.0, 34.0)):
        self.t = 1000.0
        self.bridge = bc.BridgeCore()
        self.state = sc.Go2StateCore()
        self.state.on_clock(-0.84, "CONVERGED")
        self.check = fc.FeedbackChecker(stale_s=STALE_S)
        self.code, self.hips, self.modes = code, list(hips), [1] * 12
        self.x = self.y = self.yaw = self.v = self.w = 0.0
        self.cmd, self.last_move = (0.0, 0.0), None
        self.link_up, self.sport_frozen, self.hex_sending = True, False, False
        self.profile = (0.0, 0.0)
        self.stamp_ns = 1761700000 * 10 ** 9    # the hexapod's clock, in ns
        self.replay_stamp = None
        self.q = []                              # (deliver_t, fn)
        self.next = {"send": 0.0, "tick": 0.0, "sport": 0.0, "status": 0.0, "eval": 0.0}
        self.n_tick = 0
        self.log = []                            # (t, received, cls, reason)
        self.requests = []

    # --- links ---
    def deliver(self, delay, fn):
        if self.link_up:
            self.q.append((self.t + delay, fn))

    def _hex_send(self):
        if self.replay_stamp is not None:
            sec, ns = self.replay_stamp
        else:
            self.stamp_ns += 100000000
            sec, ns = divmod(self.stamp_ns, 10 ** 9)
        self.check.on_sent(self.t, sec, ns)
        vx, wz = self.profile
        self.deliver(LINK_S, lambda now, s=sec, n=ns: self.bridge.on_twist_stamped(now, s, n, vx, 0, 0, 0, 0, wz))

    def _status_to_hex(self):
        level, _msg, values = self.state.status(self.t)
        stamp = int(self.t * 1e9)
        sec, ns = divmod(stamp, 10 ** 9)
        v = dict(values)
        self.deliver(LINK_S, lambda now: self.check.on_status(now, sec, ns, fc.HARDWARE_ID, level, v))

    # --- body ---
    def _body(self):
        moving = self.last_move is not None and self.t - self.last_move <= MOVE_LAPSE_S
        tv, tw = self.cmd if moving else (0.0, 0.0)
        a = DT / TAU_S
        self.v += (tv - self.v) * a
        self.w += (tw - self.w) * a
        self.yaw += self.w * DT
        self.x += self.v * DT * math.cos(self.yaw)
        self.y += self.v * DT * math.sin(self.yaw)

    def _route(self, reqs):
        for name, _api, param in reqs:
            self.requests.append((self.t, name))
            if name == "MOVE":
                p = json.loads(param)
                self.cmd, self.last_move = (p["x"], p["z"]), self.t
            elif name == "STOPMOVE":
                self.cmd = (0.0, 0.0)
            elif name == "STANDDOWN":
                self.code = 2006

    def run(self, secs):
        end = self.t + secs
        while self.t < end - 1e-9:
            self.t = round(self.t + DT, 6)
            due = [x for x in self.q if x[0] <= self.t]
            self.q = [x for x in self.q if x[0] > self.t]
            for _, fn in due:
                fn(self.t)
            self._body()
            if self.hex_sending and self.t >= self.next["send"]:
                self._hex_send()
                self.next["send"] = self.t + 0.1
            if self.t >= self.next["sport"]:
                if not self.sport_frozen:
                    vx_b = self.v * math.cos(self.yaw)
                    vy_b = self.v * math.sin(self.yaw)
                    self.bridge.on_sport(self.t, self.code, self.x, self.y, vx_b, vy_b, self.w)
                    self.state.on_sport(self.t, self.code, vx_b, vy_b, self.w, self.x, self.y, self.yaw)
                self.bridge.on_low(self.t, *self.hips)
                self.state.on_low(self.t, self.modes, *self.hips)
                self.next["sport"] = self.t + 0.02
            if self.t >= self.next["tick"]:
                self._route(self.bridge.tick(self.t))
                self.n_tick += 1
                if self.n_tick % 2 == 0:          # bridge_node STATUS_EVERY = 2 -> 5 Hz
                    # The real node json.dumps()es and go2_state_node json.loads() it:
                    # the round trip is part of the contract under test.
                    st = self.bridge.status(self.t)
                    st["armed"], st["input_type"] = True, "twist_stamped"
                    self.state.on_bridge(self.t, json.loads(json.dumps(st)))
                self.next["tick"] = self.t + 0.1
            if self.t >= self.next["status"]:
                self._status_to_hex()
                self.next["status"] = self.t + 0.2
            if self.t >= self.next["eval"]:
                r = self.check.evaluate(self.t)
                self.log.append((self.t, r["received"], r["cls"], r["reason"]))
                self.next["eval"] = self.t + 0.1
        return self

    def labels(self, t0=-1e18, t1=1e18):
        return [(t, rc, c, why) for t, rc, c, why in self.log if t0 <= t < t1]

    def first(self, t0, pred):
        for t, rc, c, why in self.log:
            if t >= t0 and pred(rc, c, why):
                return t
        return None


def classes(rows):
    return sorted({c for _, _, c, _ in rows})


print("S1   no command, robot standing")
w = World().run(3.0)
rows = w.labels(w.t - 2.0)
check("S1: STOPPED after warm-up, never RECEIVED", classes(rows) == ["STOPPED"] and not any(r[1] for r in rows),
      classes(rows))
print("S1b  no command, robot lying (bridge WAITING)")
w = World(code=1001).run(3.0)
rows = w.labels(w.t - 2.0)
check("S1b: STOPPED (WAITING is a warning, not a fault), never RECEIVED",
      classes(rows) == ["STOPPED"] and not any(r[1] for r in rows), classes(rows))

print("S2   zero commands")
w = World().run(2.0)
w.hex_sending, w.profile = True, (0.0, 0.0)
t0 = w.t
w.run(2.5)
rows = w.labels(t0)
check("S2: RECEIVED at some point", any(r[1] for r in rows))
check("S2: always STOPPED, never MOVING", classes(rows) == ["STOPPED"], classes(rows))
check("S2: the bridge sent no Move", not any(n == "MOVE" for _, n in w.requests))

print("S3   0.3 m/s straight, 2.5 s")
w = World().run(2.0)
w.hex_sending, w.profile = True, (0.3, 0.0)
t0 = w.t
w.run(2.5)
w.hex_sending = False
t_last = w.t
w.run(3.0)
t_rcv = w.first(t0, lambda rc, c, why: rc)
t_mov = w.first(t0, lambda rc, c, why: c == "MOVING")
t_stop = w.first(t_last, lambda rc, c, why: c == "STOPPED")
t_unrcv = w.first(t_last, lambda rc, c, why: not rc)
print("     model latencies: RECEIVED %.2f s, MOVING %.2f s after the first send; STOPPED %.2f s, "
      "not-received %.2f s after the last" % (t_rcv - t0, t_mov - t0, t_stop - t_last, t_unrcv - t_last))
check("S3: RECEIVED", t_rcv is not None)
check("S3: MOVING within 0.8 s of the first send (PFC-8's budget, in this model)",
      t_mov is not None and t_mov - t0 <= 0.8, None if t_mov is None else t_mov - t0)
check("S3: STOPPED within 1.5 s of the last send (PFC-8's budget, in this model)",
      t_stop is not None and t_stop - t_last <= 1.5, None if t_stop is None else t_stop - t_last)
check("S3: RECEIVED lapses once commands stop (within stale_s + one status period)",
      t_unrcv is not None and t_unrcv - t_last <= STALE_S + 0.5, None if t_unrcv is None else t_unrcv - t_last)
check("S3: never UNKNOWN or FAULT after warm-up", set(classes(w.labels(t0))) <= {"MOVING", "STOPPED"},
      classes(w.labels(t0)))

print("S4   turn 1.0 rad/s, 2 s")
w = World().run(2.0)
w.hex_sending, w.profile = True, (0.0, 1.0)
t0 = w.t
w.run(2.0)
check("S4: MOVING via yaw rate", w.first(t0, lambda rc, c, why: c == "MOVING") is not None)

print("S5   hexapod publisher killed mid-walk")
w = World().run(2.0)
w.hex_sending, w.profile = True, (0.3, 0.0)
w.run(1.5)
w.hex_sending = False
t_kill = w.t
w.run(3.0)
rows = w.labels(t_kill)
t_stop = w.first(t_kill, lambda rc, c, why: c == "STOPPED")
check("S5: MOVING -> STOPPED (the bridge's watchdog)", t_stop is not None, classes(rows))
check("S5: one StopMove from the watchdog", [n for _, n in w.requests].count("STOPMOVE") == 1)
check("S5: the STATUS never goes stale (only the echo does)", "UNKNOWN" not in classes(rows), classes(rows))

print("S6   link (ros1_bridge) killed mid-walk")
w = World().run(2.0)
w.hex_sending, w.profile = True, (0.3, 0.0)
w.run(1.5)
w.link_up = False
t_dead = w.t
w.run(3.0)
rows = w.labels(t_dead)
t_unk = w.first(t_dead, lambda rc, c, why: c == "UNKNOWN")
print("     model: UNKNOWN %.2f s after the link died" % (t_unk - t_dead))
check("S6: UNKNOWN within STALE_S + 0.2 s of the link dying (PFC-7's bound, in this model)",
      t_unk is not None and t_unk - t_dead <= STALE_S + 0.2, None if t_unk is None else t_unk - t_dead)
check("⛔ S6: NEVER STOPPED between the link dying and UNKNOWN",
      not any(c == "STOPPED" for t, _, c, _ in rows if t < t_unk))
check("S6: stays UNKNOWN while the link is dead",
      all(c == "UNKNOWN" for t, _, c, _ in rows if t >= t_unk), classes([r for r in rows if r[0] >= t_unk]))
check("S6: on the Go2 side the watchdog stopped it (one StopMove)",
      [n for _, n in w.requests].count("STOPMOVE") == 1 and abs(w.v) < 0.01)

print("S7   handset yield mid-walk")
w = World().run(2.0)
w.hex_sending, w.profile = True, (0.3, 0.0)
w.run(1.5)
w.bridge.on_handset(w.t, 0.0, 0.0, 0.0, 0.0, 32)
t_y = w.t
w.run(2.0)
t_f = w.first(t_y, lambda rc, c, why: c == "FAULT" and why == "bridge_yielded")
print("     model: FAULT(bridge_yielded) %.2f s after the yield" % (t_f - t_y))
check("S7: FAULT bridge_yielded reaches the hexapod", t_f is not None)
check("S7: latched -- FAULT from then on", all(c == "FAULT" for t, _, c, _ in w.labels(t_f)))
check("S7: within 0.2 + 0.2 + 2 x LINK_S + 0.1 s (bridge 5 Hz + status 5 Hz + eval 10 Hz)",
      t_f is not None and t_f - t_y <= 0.2 + 0.2 + 2 * LINK_S + 0.1 + 1e-6, None if t_f is None else t_f - t_y)

print("S8   (fake only) faults")
w = World().run(2.0)
w.modes[6] = 0
w.run(0.5)
check("S8a: motor mode 0 on RR_hip -> FAULT motor_mode:RR_hip",
      w.log[-1][2] == "FAULT" and w.log[-1][3] == "motor_mode:RR_hip", w.log[-1])
w = World().run(2.0)
w.hex_sending, w.profile = True, (0.3, 0.0)
w.run(1.0)
w.hips[0] = 51.0
w.run(1.5)          # StandDown follows the abort by STANDDOWN_DELAY_S = 1.0 s (r1 ran 1.0 s: too short)
check("S8b: rear hip 51 C mid-walk -> FAULT naming the bridge abort AND the hot hip",
      w.log[-1][2] == "FAULT" and "bridge_aborted" in w.log[-1][3] and "rear_hip_hot:51" in w.log[-1][3], w.log[-1])
check("S8b: the bridge sent StopMove then StandDown",
      [n for _, n in w.requests if n != "MOVE"] == ["STOPMOVE", "STANDDOWN"], [n for _, n in w.requests if n != "MOVE"])
w = World().run(2.0)
w.hex_sending, w.profile = True, (0.3, 0.0)
w.run(1.0)
w.sport_frozen = True
t_fr = w.t
w.run(1.5)
# Expected order: UNKNOWN first (the state core is blind after sport_stale_s), then FAULT once
# the bridge's abort arrives on its status (2 Hz then; 5 Hz since 29 Oct). r1 expected FAULT only -- my expectation, too narrow.
rows = w.labels(t_fr + sc.DEFAULTS["sport_stale_s"] + 0.2 + 2 * LINK_S + 0.1)
check("S8c: /sportmodestate frozen mid-walk -> only UNKNOWN or FAULT once stale, never MOVING/STOPPED",
      set(classes(rows)) <= {"UNKNOWN", "FAULT"}, classes(rows))
check("S8c: ...ending in FAULT naming the bridge's abort",
      rows[-1][2] == "FAULT" and "bridge_aborted:/sportmodestate silent" in rows[-1][3], rows[-1])

print("S9   (fake only) a stale command stamp re-sent")
w = World().run(2.0)
w.hex_sending, w.profile = True, (0.0, 0.0)
w.run(1.0)
old = divmod(w.stamp_ns - 300000000, 10 ** 9)
w.replay_stamp = old
t_r = w.t
w.run(1.5)
check("S9: the bridge refused every replayed send", w.bridge.counts["dropped_nonmonotonic"] >= 14,
      w.bridge.counts["dropped_nonmonotonic"])
check("S9: the checker never reports RECEIVED for the replay",
      not any(rc for t, rc, _, _ in w.log if t >= t_r + 0.15), [r for r in w.log if r[0] >= t_r and r[1]][:3])

print()
print("%d checks, %d failure(s)" % (n_checks, len(fails)))
if n_checks < 28:
    print("FAIL: fewer checks ran than written -- refusing to report success")
    sys.exit(1)
print("ALL PHASE 1 LOGIC SCENARIOS PASS" if not fails else "FAILURES:\n  - " + "\n  - ".join(fails))
sys.exit(1 if fails else 0)
