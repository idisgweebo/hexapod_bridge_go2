#!/usr/bin/env python3
"""Offline tests for go2_action_bridge -- NO ROS, NO container, NO robot.

Plan: Claude_Research friday_dance_demo_plan.md §3.2 stage S0. Run before the Go2 is powered.

  1  allowlist: only dance1 -> 1022 can be built; nothing else, no motion/abort ids
  2  params: limits can be tightened, never loosened; nonsense refuses
  3  the state machine on synthetic time, one rule per scenario
  4  REPLAY of the real 27 Oct handset dances (dance_a/b_state.csv) through the core
  5  package audit (AST): one robot publisher, literal; one build site; no other ids
  6  anti-vacuity: the auditor FAILS deliberately bad sources
  7  clean interpreter: the core imports with no rclpy anywhere
"""
import ast
import csv
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, "..", "ros2_ws", "src", "go2_action_bridge"))
CAPTURES = os.path.normpath(os.path.join(HERE, "..", "logs", "go2", "dance_20261027"))
sys.path.insert(0, PKG)
from go2_action_bridge import action_core as ac                      # noqa: E402

n_checks = 0
n_replay = 0
fails = []


def check(label, ok, detail=""):
    global n_checks
    n_checks += 1
    print(("  ok   " if ok else "  FAIL ") + label + ("" if ok else "  -- " + str(detail)[:300]))
    if not ok:
        fails.append(label)


def raises(fn, exc=Exception):
    try:
        fn()
    except exc:
        return True
    return False


# ------------------------------------------------------------- 1 allowlist ---
print("1    allowlist")
check("allowlist is exactly dance1 -> (DANCE1, 1022)", ac.ACTIONS == {"dance1": ("DANCE1", 1022)}, ac.ACTIONS)
check("dance1 builds (DANCE1, 1022, '')", ac.build_request("dance1") == ("DANCE1", 1022, ""))
for bad in ("dance2", "DANCE1", "Dance1", "dance", "move", "stopmove", "standdown", "damp", "", "dance1 "):
    check("%r is unbuildable" % bad, raises(lambda b=bad: ac.build_request(b), ac.RequestRejected))
# Every id from Gate 6's allowlist plus Damp and the acrobatics: none may be reachable.
FORBIDDEN_IDS = {1001, 1002, 1003, 1004, 1005, 1006, 1008, 1009, 1010, 1023, 1027, 1030, 1031, 1032, 2055}
check("no motion/abort/acrobatic id is in the allowlist",
      not (set(v[1] for v in ac.ACTIONS.values()) & FORBIDDEN_IDS))

# --------------------------------------------------------------- 2 params ---
print("2    params")
check("defaults validate", ac.validate_params() == ac.DEFAULTS)
check("max_sends comes back an int", isinstance(ac.validate_params({"max_sends": 2.0})["max_sends"], int))
check("max_sends 4 REFUSES (hard cap 3)", raises(lambda: ac.validate_params({"max_sends": 4}), ac.ConfigError))
check("max_sends 1.5 REFUSES", raises(lambda: ac.validate_params({"max_sends": 1.5}), ac.ConfigError))
check("max_sends 0 REFUSES", raises(lambda: ac.validate_params({"max_sends": 0}), ac.ConfigError))
check("hip_arm_max_c 46 REFUSES (cap 45)", raises(lambda: ac.validate_params({"hip_arm_max_c": 46}), ac.ConfigError))
check("hip_arm_max_c 40 is allowed", ac.validate_params({"hip_arm_max_c": 40})["hip_arm_max_c"] == 40)
check("lockout_s 4 REFUSES (floor 5)", raises(lambda: ac.validate_params({"lockout_s": 4}), ac.ConfigError))
check("handset_quiet_s 0.5 REFUSES (floor 1)",
      raises(lambda: ac.validate_params({"handset_quiet_s": 0.5}), ac.ConfigError))
check("unknown parameter refuses", raises(lambda: ac.validate_params({"lockout": 10}), ac.ConfigError))
check("zero / negative / NaN refuse",
      all(raises(lambda v=v: ac.validate_params({"stale_s": v}), ac.ConfigError) for v in (0, -1, float("nan"))))


# --------------------------------------------------------- 3 state machine ---
class Sim(object):
    """Telemetry at 50 Hz, tick at 10 Hz, like the node. Records every request."""

    def __init__(self, params=None, error_code=100, hips=(30, 30), motors_ok=12):
        self.c = ac.ActionCore(params)
        self.t = 1000.0
        self.error_code, self.hips, self.motors_ok = error_code, hips, motors_ok
        self.telemetry = True
        self.sent = []
        self.next_tick = self.t

    def run(self, secs):
        end = self.t + secs
        while self.t < end - 1e-9:
            if self.telemetry:
                self.c.on_sport(self.t, self.error_code)
                self.c.on_low(self.t, self.hips[0], self.hips[1], self.motors_ok)
            if self.t >= self.next_tick - 1e-9:
                self.c.tick(self.t)
                self.next_tick += 0.1
            self.t += 0.02

    def trigger(self, text="dance1"):
        r = self.c.on_trigger(self.t, text)
        self.sent.extend((round(self.t, 3), x[1]) for x in r)
        return r

    def dance(self, dur=17.0, end_code=1013):
        """The robot dances as measured: 1006 for dur s, then end_code."""
        self.run(0.1)
        self.error_code = 1006
        self.run(dur)
        self.error_code = end_code


print("3    state machine")
c = ac.ActionCore()
c.tick(1.0)
check("fresh core: WAITING", c.state == "WAITING")
check("trigger with no telemetry -> nothing", c.on_trigger(1.0, "dance1") == [] and c.counts["ignored_not_ready"] == 1)

sim = Sim()
sim.run(1.0)
check("balance stand, all healthy -> READY", sim.c.state == "READY", sim.c.status())
r = sim.trigger()
check("trigger in READY -> exactly one (DANCE1, 1022)", r == [("DANCE1", 1022, "")], r)
check("...state SENT", sim.c.state == "SENT")
sim.run(0.05)
check("a second trigger right after -> NOTHING", sim.trigger() == [])
sim.dance(17.0)
check("1006 -> DANCING", sim.c.state == "DANCING", sim.c.status())
for _ in range(50):
    sim.trigger()
    sim.run(0.1)
check("50 triggers during the dance at 10 Hz -> still ONE request in total", len(sim.sent) == 1, sim.sent)
check("...counted as ignored_not_ready", sim.c.counts["ignored_not_ready"] >= 50, sim.c.counts)
sim.error_code = 1013
sim.run(0.3)
check("1006 -> 1013 -> LOCKOUT, reason names 1013", sim.c.state == "LOCKOUT" and "1013" in sim.c.reason,
      sim.c.status())
sim.run(10.5)
check("max_sends 1: after the lockout -> SPENT (latched)", sim.c.state == "SPENT", sim.c.status())
sim.error_code = 100
sim.run(3.0)
check("SPENT survives the robot returning to 100", sim.c.state == "SPENT" and sim.trigger() == [])

# a rostopic pub -r 10 burst from before READY through the whole dance
sim = Sim()
sim.run(1.0)
for _ in range(20):
    sim.trigger()
    sim.run(0.1)
check("a 10 Hz burst of 20 triggers in READY -> ONE request (no 1006 yet, SENT blocks the rest)",
      len(sim.sent) == 1, sim.sent)

# max_sends 2: second send needs lockout AND a return to 100 by the operator
sim = Sim(params={"max_sends": 2})
sim.run(1.0)
sim.trigger()
sim.dance(17.0)
sim.run(0.3)
sim.run(10.5)
check("max_sends 2, after lockout, robot still in 1013 -> WAITING naming 1013 (never starts from 1013)",
      sim.c.state == "WAITING" and "1013" in sim.c.reason, sim.c.status())
check("...and a trigger there sends nothing", sim.trigger() == [])
sim.error_code = 100
sim.run(0.5)
check("operator brings it to 100 -> READY", sim.c.state == "READY", sim.c.status())
check("...second trigger sends", len(sim.trigger()) == 1 and len(sim.sent) == 2)
sim = Sim(params={"max_sends": 2})
sim.run(1.0)
sim.trigger()
sim.dance(17.0, end_code=100)
sim.run(5.0)
check("lockout holds 10 s even when the dance ends straight in 100", sim.c.state == "LOCKOUT" and sim.trigger() == [])
sim.run(5.5)
check("...then READY", sim.c.state == "READY", sim.c.status())

# start failures
sim = Sim()
sim.run(1.0)
sim.trigger()
sim.run(3.5)
check("no 1006 within 3 s -> NO_START (latched) -- the dry-run outcome", sim.c.state == "NO_START", sim.c.status())
sim.run(5.0)
check("NO_START survives, triggers ignored", sim.c.state == "NO_START" and sim.trigger() == [])
sim = Sim()
sim.run(1.0)
sim.trigger()
sim.run(2.5)
sim.dance(5.0)
sim.run(0.3)
check("1006 at 2.5 s (inside the 3 s window) -> DANCING path, not NO_START", sim.c.state == "LOCKOUT", sim.c.status())
sim = Sim()
sim.run(1.0)
sim.trigger()
sim.dance(61.0)
check("1006 for > 60 s -> OVERRUN (latched)", sim.c.state == "OVERRUN", sim.c.status())

# response
sim = Sim()
sim.run(1.0)
sim.trigger()
sim.c.on_response(sim.t + 0.01, 1022, 3104)
sim.run(0.5)
check("response code != 0 for 1022 -> REFUSED (latched)", sim.c.state == "REFUSED" and "3104" in sim.c.reason,
      sim.c.status())
sim = Sim()
sim.run(1.0)
sim.trigger()
sim.c.on_response(sim.t + 0.01, 1027, 7)
sim.c.on_response(sim.t + 0.01, 1022, 0)
sim.run(0.2)
check("someone else's non-zero response is ignored; our code 0 is recorded",
      sim.c.state == "SENT" and sim.c.last_response == (1022, 0), sim.c.status())

# late response. 27 Oct S3 (n = 1): the 1022 response came 17.342 s after the send, at the END of
# the dance, 12 ms BEFORE 1006 -> 1013 (PAB-2 falsified on timing). 12 ms is a race: the order can
# flip, so every check below runs with the response just before AND just after the end.


def late(code, after_end, params=None):
    s = Sim(params=params)
    s.run(1.0)
    s.trigger()
    s.run(0.1)
    s.error_code = 1006
    s.run(17.2)
    st_mid = (s.c.state, s.c.last_response)
    if not after_end:
        s.c.on_response(s.t, 1022, code)
    s.run(0.02)
    s.error_code = 1013
    s.run(0.15)    # > one 10 Hz tick: the core changes state on tick, not on telemetry (28 Oct test bug:
    if after_end:  # 0.06 s here left it DANCING, so "after 1013" silently re-tested "before")
        s.at_response = s.c.state
        s.c.on_response(s.t, 1022, code)
    s.run(0.2)
    return s, st_mid


for after_end in (False, True):
    when = "after 1013" if after_end else "before 1013"
    s, st_mid = late(0, after_end)
    check("late: 17 s DANCING with no response -> still DANCING (silence is not a failure)",
          st_mid == ("DANCING", None), st_mid)
    if after_end:
        check("late: (anti-vacuity) the 'after 1013' response really lands in LOCKOUT",
              s.at_response == "LOCKOUT", s.at_response)
    check("late: code 0 %s -> LOCKOUT, recorded, never REFUSED" % when,
          s.c.state == "LOCKOUT" and s.c.last_response == (1022, 0), s.c.status())
    s.run(10.0)
    check("late: code 0 %s -> SPENT after the lockout, one send" % when,
          s.c.state == "SPENT" and len(s.sent) == 1, s.c.status())
    s, _ = late(3104, after_end, params={"max_sends": 2})
    check("late: code 3104 %s -> REFUSED (latched), whichever side of the 12 ms race" % when,
          s.c.state == "REFUSED" and "3104" in s.c.reason, s.c.status())
    s.run(12.0)
    check("late: ...and max_sends 2 still cannot send again", s.c.state == "REFUSED" and s.trigger() == []
          and len(s.sent) == 1, s.c.status())
s, _ = late(0, True)
s.run(10.0)
s.c.on_response(s.t, 1022, 3104)
check("late: a non-zero response after SPENT is recorded, SPENT stays (latches do not overwrite)",
      s.c.state == "SPENT" and s.c.last_response == (1022, 3104), s.c.status())

# preconditions, each on its own
for label, kw, needle in (("stand-lock 1002", {"error_code": 1002}, "1002"),
                          ("lying 1001", {"error_code": 1001}, "1001"),
                          ("post-dance 1013", {"error_code": 1013}, "1013"),
                          ("dance code 1006", {"error_code": 1006}, "1006"),
                          ("11 motors in mode 1", {"motors_ok": 11}, "11 of 12"),
                          ("rear hip 45 C", {"hips": (30, 45)}, "45")):
    sim = Sim(**kw)
    sim.run(1.0)
    check("%s -> WAITING naming it, trigger sends nothing" % label,
          sim.c.state == "WAITING" and needle in sim.c.reason and sim.trigger() == [], sim.c.status())
sim = Sim(hips=(44, 44))
sim.run(1.0)
check("rear hip 44 C -> READY", sim.c.state == "READY")
sim = Sim()
sim.telemetry = False
sim.run(1.0)
check("no telemetry -> WAITING", sim.c.state == "WAITING" and "sportmodestate" in sim.c.reason)
sim = Sim()
sim.run(1.0)
sim.telemetry = False
sim.t += 0.6
check("telemetry 0.6 s old at the trigger -> refused even though the last tick said READY",
      sim.trigger() == [] and "fresh" in sim.c.reason, sim.c.status())

# handset
sim = Sim()
sim.run(1.0)
sim.c.on_handset(sim.t, 0, 0, 0, 0, 288)
sim.run(1.0)
check("handset key 1 s ago -> refused", sim.trigger() == [] and "handset" in sim.c.reason, sim.c.reason)
sim.run(1.2)   # 1.2, not 1.1: the 10 Hz tick must land past the 2.0 s window to SHOW READY (27 Oct: 1.99 s)
check("2.2 s after the key -> READY again (handset does NOT latch)", sim.c.state == "READY" and len(sim.trigger()) == 1)
sim = Sim()
sim.run(1.0)
sim.c.on_handset(sim.t, 0, 0.15, 0, 0, 0)
check("stick inside 0.2 is not activity", len(sim.trigger()) == 1)
sim = Sim()
sim.run(1.0)
sim.c.on_handset(sim.t, 0, 0.9, 0, 0, 0)
check("stick at 0.9 is activity", sim.trigger() == [])

# foreign requests
sim = Sim()
sim.run(1.0)
sim.c.on_robot_request(sim.t, 1008)
sim.run(1.0)
check("a Move from someone else 1 s ago -> refused, naming 1008",
      sim.trigger() == [] and "1008" in sim.c.reason, sim.c.reason)
sim.run(1.1)
check("2.1 s later -> sends", len(sim.trigger()) == 1)
sim = Sim(params={"max_sends": 2})
sim.run(1.0)
sim.trigger()
sim.c.on_robot_request(sim.t + 0.003, 1022)
check("our own 1022 seen on our subscription -> own_echo, not foreign",
      sim.c.counts["own_echo"] == 1 and sim.c.foreign_t is None, sim.c.counts)
sim.c.on_robot_request(sim.t + 1.5, 1022)
check("a 1022 1.5 s after ours -> foreign", sim.c.foreign_t is not None)

# go2_cmd_bridge on the graph
sim = Sim()
sim.run(1.0)
sim.c.set_cmd_bridge_present(True)
sim.run(0.2)
check("go2_cmd_bridge present -> WAITING, refused", sim.c.state == "WAITING" and "cmd_bridge" in sim.c.reason
      and sim.trigger() == [], sim.c.reason)
sim.c.set_cmd_bridge_present(False)
sim.run(0.2)
check("...gone -> READY", sim.c.state == "READY")

# unknown words
sim = Sim()
sim.run(1.0)
for w in ("dance2", "flip", "Dance1", "", "dance1; rm -rf /"):
    sim.trigger(w)
check("unknown words send nothing and leave READY untouched",
      sim.sent == [] and sim.c.state == "READY" and sim.c.counts["ignored_unknown"] == 5, sim.c.counts)
check("' dance1\\n' is accepted (stripped, as rostopic may pad)", len(sim.trigger(" dance1\n")) == 1)

# hot during the dance: a warning, never a request
sim = Sim()
sim.run(1.0)
sim.trigger()
sim.run(0.1)
sim.error_code = 1006         # NOT sim.dance(): that ends the dance before the hips heat (27 Oct test bug)
sim.run(1.0)
sim.hips = (51, 40)
sim.run(0.5)
check("rear hip 51 C mid-dance -> status warning, still DANCING, no request",
      "warning" in sim.c.status() and sim.c.state == "DANCING" and len(sim.sent) == 1, sim.c.status())


# -------------------------------------------- 4 replay of the real dances ---
print("4    replay: the 27 Oct handset dances through the core (send placed at the handset trigger)")


def replay(path, t_send):
    """Feed the measured error_code / hips / motors stream; send at t_send. Returns (core, states)."""
    core = ac.ActionCore()
    states = []
    next_tick = None
    sent = False
    with open(path) as fh:
        for row in csv.DictReader(fh):
            t = float(row["t_wall"])
            core.on_sport(t, int(row["error_code"]))
            core.on_low(t, float(row["rr_hip_c"]), float(row["rl_hip_c"]), int(row["motors_mode1"]))
            if not sent and t >= t_send:
                sent = core.on_trigger(t, "dance1") != []
                states.append(("trigger", core.state, core.reason))
            if next_tick is None or t >= next_tick:
                core.tick(t)
                if not states or states[-1][1] != core.state:
                    states.append((round(t - t_send, 1), core.state, core.reason))
                next_tick = t + 0.1
    return core, states


for name, t_send, exp_dur in (("dance_a", 1793137074.0, 17.2), ("dance_b", 1793137459.0, 36.9)):
    path = os.path.join(CAPTURES, name + "_state.csv")
    if not os.path.exists(path):
        print("  SKIP %s: %s not present (not a failure; replay needs the 27 Oct capture)" % (name, path))
        continue
    core, states = replay(path, t_send)
    print("     %s: %s" % (name, [(s[0], s[1]) for s in states]))
    seq = [s[1] for s in states]
    n0 = n_checks
    check("%s: READY at the trigger, then SENT -> DANCING -> LOCKOUT -> SPENT" % name,
          ("trigger", "SENT") == states[[s[0] for s in states].index("trigger")][:2]
          and [x for x in seq if x in ("SENT", "DANCING", "LOCKOUT", "SPENT")][-4:]
          == ["SENT", "DANCING", "LOCKOUT", "SPENT"], states)
    d = [s for s in states if s[1] == "LOCKOUT"]
    check("%s: dance duration in the reason ~%.1f s" % (name, exp_dur),
          d and abs(float(d[0][2].split("after ")[1].split(" s")[0]) - exp_dur) < 0.5, d)
    check("%s: ends in 1013" % name, d and "1013" in d[0][2], d)
    n_replay += n_checks - n0

# The ARMED S3 send of 27 Oct, through the core: state CSV + every request, response and handset
# sample from s3_a_events.csv, merged on time. The trigger is placed at the 1022's wire time.
print("4b   replay: the 27 Oct armed S3 send (state + requests + response + handset) through the core")
S3 = os.path.normpath(os.path.join(HERE, "..", "logs", "dance", "20261027"))
S3_SEND = 1793140704.5006583      # first /api/sport/request 1022 on the wire (s3_a_events.csv)
S3_NODE_READY = 1793140694.1368   # bridge.txt: "state WAITING -> READY"
s3_state, s3_events = os.path.join(S3, "s3_a_state.csv"), os.path.join(S3, "s3_a_events.csv")
if not (os.path.exists(s3_state) and os.path.exists(s3_events)):
    print("  SKIP S3 replay: %s not present (not a failure; needs the 27 Oct S3 capture)" % S3)
else:
    feed = []
    with open(s3_state) as fh:
        for row in csv.DictReader(fh):
            feed.append((float(row["t_wall"]), "state", row))
    with open(s3_events) as fh:
        for row in csv.DictReader(fh):
            feed.append((float(row["t_wall"]), row["topic"], row))
    feed.sort(key=lambda x: x[0])
    core = ac.ActionCore()
    n0 = n_checks
    ready_t = resp_state = None
    states, sends, next_tick = [], [], None
    for t, kind, row in feed:
        if kind == "state":
            core.on_sport(t, int(row["error_code"]))
            core.on_low(t, float(row["rr_hip_c"]), float(row["rl_hip_c"]), int(row["motors_mode1"]))
        elif kind == "/wirelesscontroller":
            core.on_handset(t, float(row["lx"]), float(row["ly"]), float(row["rx"]), float(row["ry"]),
                            int(row["keys"]))
        elif kind == "/api/sport/request":
            if not sends:   # the wire copy of the send marks the trigger: send first, then see the echo
                sends.extend(core.on_trigger(t - 1e-4, "dance1"))
            core.on_robot_request(t, int(row["api_id"]))
        elif kind == "/api/sport/response":
            core.on_response(t, int(row["api_id"]), int(row["code"]))
            resp_state = (round(t - S3_SEND, 3), core.state)
        if next_tick is None or t >= next_tick:
            core.tick(t)
            if not states or states[-1][1] != core.state:
                states.append((round(t - S3_SEND, 2), core.state, core.reason))
                if core.state == "READY" and ready_t is None:
                    ready_t = t
            next_tick = t + 0.1
    print("     S3: %s" % [(s[0], s[1]) for s in states])
    print("     S3: response at %s" % (resp_state,))
    check("S3: READY within 0.2 s of the real node's READY (handset quiet 2 s after Start)",
          ready_t is not None and abs(ready_t - S3_NODE_READY) < 0.2, (ready_t, S3_NODE_READY))
    check("S3: exactly one (DANCE1, 1022) at the send", sends == [("DANCE1", 1022, "")], sends)
    check("S3: the 1022 on the wire counted as our own echo, not foreign",
          core.counts["own_echo"] == 1 and core.foreign_t is None, core.counts)
    check("S3: the response (1022, 0) arrived ~17.34 s after the send, while DANCING",
          resp_state is not None and abs(resp_state[0] - 17.342) < 0.01 and resp_state[1] == "DANCING", resp_state)
    seq = [s[1] for s in states]
    check("S3: ... -> DANCING -> LOCKOUT -> SPENT, never REFUSED/NO_START/OVERRUN",
          seq[-3:] == ["DANCING", "LOCKOUT", "SPENT"] and not set(seq) & {"REFUSED", "NO_START", "OVERRUN"}, states)
    d = [s for s in states if s[1] == "LOCKOUT"]
    check("S3: dance 17.3 s, ends in 1013 (bridge.txt: 'after 17.3 s in error_code 1013')",
          d and "after 17.3 s in error_code 1013" in d[0][2], d)
    check("S3: last_response (1022, 0)", core.last_response == (1022, 0), core.last_response)
    n_replay += n_checks - n0

# --------------------------------------------------------- 5 package audit ---
ROBOT_TOPIC = "/api/sport/request"
FORBIDDEN = ["/lowcmd", "/arm_Command", "/utlidar/switch", "bashrunner", "programming_actuator",
             "motion_switcher", "sport_lease"]


def audit(sources):
    """sources: {name: text}. Returns (problems, publishers_found)."""
    probs, pubs, build_sites = [], [], []
    for name, text in sources.items():
        for bad in FORBIDDEN:
            if bad in text:
                probs.append("%s: forbidden substring %r" % (name, bad))
        i = text.find("/api/")
        while i >= 0:
            if not (text.startswith(ROBOT_TOPIC, i) or text.startswith("/api/sport/response", i)):
                probs.append("%s: /api/ outside the sport request/response topics: %r" % (name, text[i:i + 40]))
            i = text.find("/api/", i + 1)
        tree = ast.parse(text)
        for fn in [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]:
            for node in ast.walk(fn):
                if isinstance(node, ast.Call) and getattr(node.func, "id", getattr(node.func, "attr", "")) == "build_request":
                    build_sites.append("%s:%s" % (name, fn.name))
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "create_publisher":
                if len(node.args) < 2:
                    probs.append("%s: create_publisher without a positional topic" % name)
                    continue
                topic = node.args[1]
                if not (isinstance(topic, ast.Constant) and isinstance(topic.value, str)):
                    probs.append("%s: create_publisher topic is not a string literal" % name)
                    continue
                pubs.append(topic.value)
                if topic.value != ROBOT_TOPIC and not topic.value.startswith("/go2/"):
                    probs.append("%s: publisher %r outside /go2/" % (name, topic.value))
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                mod = node.module if isinstance(node, ast.ImportFrom) else node.names[0].name
                if mod and (mod.startswith("unitree_") or mod.startswith("rclpy")) and "node" not in name:
                    probs.append("%s: %s imported outside the node" % (name, mod))
            # a Request built by hand must take its api_id from build_request's tuple, never a literal
            if (isinstance(node, ast.Assign) and any(getattr(t, "attr", "") == "api_id" for t in node.targets)
                    and isinstance(node.value, ast.Constant)):
                probs.append("%s: api_id assigned a literal (%r)" % (name, node.value.value))
    if pubs.count(ROBOT_TOPIC) != 1:
        probs.append("expected exactly ONE publisher on %s, found %d" % (ROBOT_TOPIC, pubs.count(ROBOT_TOPIC)))
    if build_sites and sorted(set(build_sites)) != ["action_core.py:on_trigger"]:
        probs.append("build_request called outside ActionCore.on_trigger: %s" % sorted(set(build_sites)))
    return probs, pubs


print("5    package audit")
srcs = {}
for root, _, files in os.walk(PKG):
    for f in files:
        if f.endswith(".py") and "__pycache__" not in root and f != "setup.py":
            with open(os.path.join(root, f)) as fh:
                srcs[f] = fh.read()
print("     scanned: " + ", ".join(sorted(srcs)))
check("scanned the core and the node (anti-vacuity)", "action_core.py" in srcs and "action_node.py" in srcs, sorted(srcs))
probs, pubs = audit(srcs)
print("     publishers: %s" % pubs)
check("package audit clean", probs == [], probs)
core_tree = ast.parse(srcs.get("action_core.py", ""))
ints = sorted(set(n.value for n in ast.walk(core_tree) if isinstance(n, ast.Constant)
                  and isinstance(n.value, int) and 1000 <= n.value <= 2100))
check("the only api-range integers in the core are 1022 (DANCE1) and 1006 (the dance state code)",
      set(ints) <= {1006, 1022}, ints)

# ------------------------------------------------------ 6 anti-vacuity ---
print("6    anti-vacuity: the auditor must reject bad sources")
GOOD_NODE = "self.pub = self.create_publisher(Request, '/api/sport/request', 10)\n"
GOOD_CORE = "def on_trigger(self):\n    build_request('dance1')\n"
check("auditor passes a minimal good pair", audit({"action_node.py": GOOD_NODE, "action_core.py": GOOD_CORE})[0] == [])
bad_cases = {
    "two robot publishers": {"action_node.py": GOOD_NODE * 2},
    "no robot publisher": {"action_node.py": "x = 1\n"},
    "topic from a variable": {"action_node.py": "T = '/api/sport/request'\nself.create_publisher(Request, T, 10)\n"},
    "a publisher outside /go2/": {"action_node.py": GOOD_NODE + "self.create_publisher(Twist, '/cmd_vel', 10)\n"},
    "another /api/ topic": {"action_node.py": GOOD_NODE + "# see /api/bashrunner/request\n"},
    "/lowcmd mentioned": {"action_node.py": GOOD_NODE + "s = '/lowcmd'\n"},
    "unitree_api in the core": {"action_node.py": GOOD_NODE, "action_core.py": "from unitree_api.msg import Request\n"},
    "rclpy in the core": {"action_node.py": GOOD_NODE, "action_core.py": "import rclpy\n"},
    "build_request in tick": {"action_node.py": GOOD_NODE,
                              "action_core.py": GOOD_CORE + "def tick(self):\n    build_request('dance1')\n"},
    "build_request in the node": {"action_node.py": GOOD_NODE + "def route(self):\n    build_request('dance1')\n",
                                  "action_core.py": GOOD_CORE},
    "literal api_id": {"action_node.py": GOOD_NODE + "r.header.identity.api_id = 1003\n"},
}
for label, src in bad_cases.items():
    check("auditor REJECTS: " + label, audit(src)[0] != [])

# ------------------------------------------------- 7 clean interpreter ---
print("7    clean interpreter")
probe = ("import sys; sys.path.insert(0, %r)\n"
         "from go2_action_bridge import action_core\n"
         "assert 'rclpy' not in sys.modules\n"
         "print('CLEAN')\n" % PKG)
p = subprocess.run([sys.executable, "-E", "-s", "-c", probe], capture_output=True, text=True)
check("action_core imports without rclpy", p.returncode == 0 and "CLEAN" in p.stdout, p.stderr[-300:])

print()
print("%d checks (%d from the replay), %d failure(s)" % (n_checks, n_replay, len(fails)))
if n_checks - n_replay < 70:
    print("FAIL: fewer checks ran than written -- refusing to report success")
    sys.exit(1)
print("ALL ACTION BRIDGE OFFLINE TESTS PASS" if not fails else "FAILURES:\n  - " + "\n  - ".join(fails))
sys.exit(1 if fails else 0)
