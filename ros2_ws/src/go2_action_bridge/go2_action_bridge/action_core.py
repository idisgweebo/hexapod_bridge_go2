#!/usr/bin/env python3
"""The decision logic of the Go2 action bridge. PURE: no ROS, no clock, no I/O.

Plan: Claude_Research friday_dance_demo_plan.md §3.1-3.2. A word on /go2/action
("dance1") becomes ONE preset request on the Go2, sent only from a measured resting
state, then watched -- never steered -- until the robot is back at rest.

Every input arrives as a method call carrying its own `now`. on_trigger() is the ONLY
place a request is built; tick(now) advances timeouts and never sends. The node
(action_node.py) owns the clock and the DDS endpoints; tests run this file offline
(scripts/test_action_bridge_offline.py) on synthetic time.

As in go2_cmd_bridge, the core never knows whether it is armed: the node routes what
on_trigger() returns either to the robot or to /go2/action_bridge/requests only.

STATES
------
    WAITING    a precondition fails (named in `reason`)
    READY      every precondition holds; a trigger now sends
    SENT       request sent; waiting for error_code 1006 (start_timeout_s)
    DANCING    error_code 1006; watching only (max_dance_s)
    LOCKOUT    the dance ended; lockout_s before anything can be sent again
  latched -- cleared only by restarting the process:
    NO_START   1006 never appeared after a send (in a dry run this is expected)
    OVERRUN    1006 lasted longer than max_dance_s
    REFUSED    the sport server answered the request with a non-zero code
    SPENT      max_sends dances done

⛔ THERE IS NO ABORT PATH. The bridge sends exactly one api_id and nothing else: no
StopMove, no StandDown, no Damp. Every interaction with a running dance is
unmeasured; the handset is the operator's override (L2+B, last resort).

MEASURED BASIS (27 Oct 2026, v1.1.15, two handset dances, n = 2)
---------------------------------------------------------------
* both dances started from error_code 100 and ran under error_code 1006;
* both ended in 1013, which holds until the operator acts (so after a dance the
  bridge WAITS on "error_code 1013, not 100" -- it never starts from 1013);
* handset sticks reach /api/sport/request as Move (1008) + SwitchJoystick (1027)
  from an on-robot process: a request we did not send means someone else is
  driving, so it blocks a send for foreign_quiet_s.
* 1022 itself has NEVER been seen or sent; it is from unitree_ros2 @ 668d1ec5.

Python 3.8-compatible (Foxy on the Orin is a target): no match, no dict | dict,
no builtin generics in annotations.
"""
import math

# ---------------------------------------------------------------------------
# The allowlist: one action. unitree_ros2 @ 668d1ec5 ros2_sport_client.h:
# ROBOT_SPORT_API_ID_DANCE1 = 1022. Adding an action is a code change and a commit.
# ---------------------------------------------------------------------------
ACTIONS = {
    "dance1": ("DANCE1", 1022),
}

BALANCE_STAND = 100     # measured: both handset dances started here
DANCE_CODE = 1006       # measured: both handset dances ran under this code
OWN_ECHO_S = 1.0        # our own request, seen on our own subscription, within this window

HARD_MAX_SENDS = 3
HARD_HIP_ARM_MAX_C = 45.0
MIN_LOCKOUT_S = 5.0
MIN_HANDSET_QUIET_S = 1.0

DEFAULTS = {
    "stale_s": 0.5,           # /sportmodestate is ~300 Hz
    "hip_arm_max_c": 45.0,    # rear hips: no dance starts at or above this
    "handset_quiet_s": 2.0,   # no handset activity for this long before a send
    "handset_stick": 0.2,
    "foreign_quiet_s": 2.0,   # no request from anyone else for this long before a send
    "start_timeout_s": 3.0,   # 1006 must appear within this after the send
    "max_dance_s": 60.0,      # longest dance seen: 36.9 s
    "lockout_s": 10.0,        # after the dance ends
    "max_sends": 1,
}


class ConfigError(ValueError):
    pass


class RequestRejected(ValueError):
    pass


def validate_params(overrides=None):
    """Merge overrides onto DEFAULTS and refuse anything unsafe. Returns a dict."""
    p = dict(DEFAULTS)
    for k, v in (overrides or {}).items():
        if k not in DEFAULTS:
            raise ConfigError("unknown parameter %r" % (k,))
        p[k] = float(v)
    for k, v in p.items():
        if not math.isfinite(v) or v <= 0:
            raise ConfigError("%s must be a positive finite number, got %r" % (k, v))
    if p["max_sends"] != int(p["max_sends"]) or p["max_sends"] > HARD_MAX_SENDS:
        raise ConfigError("max_sends must be a whole number 1..%d, got %r" % (HARD_MAX_SENDS, p["max_sends"]))
    p["max_sends"] = int(p["max_sends"])
    if p["hip_arm_max_c"] > HARD_HIP_ARM_MAX_C + 1e-9:
        raise ConfigError("hip_arm_max_c %.1f exceeds the hard cap %.1f" % (p["hip_arm_max_c"], HARD_HIP_ARM_MAX_C))
    if p["lockout_s"] < MIN_LOCKOUT_S - 1e-9:
        raise ConfigError("lockout_s %.1f is below the floor %.1f" % (p["lockout_s"], MIN_LOCKOUT_S))
    if p["handset_quiet_s"] < MIN_HANDSET_QUIET_S - 1e-9:
        raise ConfigError("handset_quiet_s %.1f is below the floor %.1f" % (p["handset_quiet_s"], MIN_HANDSET_QUIET_S))
    return p


def build_request(action):
    """Return (name, api_id, parameter). The ONLY way a request is made."""
    if action not in ACTIONS:
        raise RequestRejected("%r is not in the allowlist %s" % (action, sorted(ACTIONS)))
    name, api_id = ACTIONS[action]
    return name, api_id, ""


class ActionCore(object):

    LATCHED = ("NO_START", "OVERRUN", "REFUSED", "SPENT")

    def __init__(self, params=None):
        self.p = validate_params(params)
        self.state = "WAITING"
        self.reason = "no telemetry yet"
        self.sport_t = self.low_t = None
        self.error_code = None
        self.hips = None
        self.motors_ok = None         # how many of the 12 motors report mode 1
        self.handset_t = None         # last handset ACTIVITY
        self.foreign_t = None         # last /api/sport/request we did not send
        self.foreign_api = None
        self.cmd_bridge_present = False
        self.sent_api = None
        self.sent_t = None
        self.dance_t = None           # first 1006 after the send
        self.end_t = None             # first non-1006 after dance_t
        self.end_code = None
        self.n_sends = 0
        self.last_response = None     # (api_id, code)
        self.last_ignored = None
        self.counts = {"triggers": 0, "sent": 0, "ignored_unknown": 0, "ignored_not_ready": 0,
                       "foreign_requests": 0, "own_echo": 0}

    # ------------------------------------------------------------- inputs ---
    def on_sport(self, now, error_code):
        self.sport_t, self.error_code = now, int(error_code)
        if self.state == "SENT" and self.error_code == DANCE_CODE and self.dance_t is None:
            self.dance_t = now
        elif self.state == "DANCING" and self.error_code != DANCE_CODE and self.end_t is None:
            self.end_t, self.end_code = now, self.error_code

    def on_low(self, now, rr_c, rl_c, motors_ok):
        self.low_t, self.hips, self.motors_ok = now, (float(rr_c), float(rl_c)), int(motors_ok)

    def on_handset(self, now, lx, ly, rx, ry, keys):
        if keys != 0 or max(abs(lx), abs(ly), abs(rx), abs(ry)) > self.p["handset_stick"]:
            self.handset_t = now

    def on_robot_request(self, now, api_id):
        """Every /api/sport/request on the wire, including our own."""
        if (self.sent_t is not None and api_id == self.sent_api and 0 <= now - self.sent_t <= OWN_ECHO_S):
            self.counts["own_echo"] += 1
            return
        self.counts["foreign_requests"] += 1
        self.foreign_t, self.foreign_api = now, int(api_id)

    def on_response(self, now, api_id, code):
        if api_id != self.sent_api:
            return
        self.last_response = (int(api_id), int(code))
        if code != 0 and self.state in ("SENT", "DANCING"):
            self.state, self.reason = "REFUSED", "sport server answered api_id %d with code %d" % (api_id, code)

    def set_cmd_bridge_present(self, present):
        self.cmd_bridge_present = bool(present)

    def on_trigger(self, now, text):
        """A message on /go2/action. Returns the list of build_request() tuples to send NOW
        (at most one). Never queues: a trigger that cannot be acted on is dropped."""
        self.counts["triggers"] += 1
        action = (text or "").strip()
        if action not in ACTIONS:
            self.counts["ignored_unknown"] += 1
            self.last_ignored = "unknown action %r" % action[:40]
            return []
        if self.state not in ("WAITING", "READY"):
            self.counts["ignored_not_ready"] += 1
            self.last_ignored = "state %s" % self.state
            return []
        wait = self._wait_reason(now)
        if wait:
            self.state, self.reason = "WAITING", wait
            self.counts["ignored_not_ready"] += 1
            self.last_ignored = wait
            return []
        req = build_request(action)
        self.sent_api, self.sent_t = req[1], now
        self.dance_t = self.end_t = self.end_code = None
        self.n_sends += 1
        self.counts["sent"] += 1
        self.state, self.reason = "SENT", "%s sent, waiting for error_code %d" % (req[0], DANCE_CODE)
        return [req]

    # --------------------------------------------------------------- tick ---
    def tick(self, now):
        """Advance timeouts and state. Never sends."""
        if self.state in self.LATCHED:
            return
        if self.state == "SENT":
            if self.dance_t is not None:
                self.state, self.reason = "DANCING", "error_code %d since %.1f s after the send" % (
                    DANCE_CODE, self.dance_t - self.sent_t)
            elif now - self.sent_t > self.p["start_timeout_s"]:
                self.state, self.reason = "NO_START", "error_code %d not seen within %.1f s of the send" % (
                    DANCE_CODE, self.p["start_timeout_s"])
                return
            else:
                return
        if self.state == "DANCING":
            if self.end_t is not None:
                self.state, self.reason = "LOCKOUT", "dance ended after %.1f s in error_code %s" % (
                    self.end_t - self.dance_t, self.end_code)
            elif now - self.dance_t > self.p["max_dance_s"]:
                self.state, self.reason = "OVERRUN", "error_code %d for more than %.0f s" % (
                    DANCE_CODE, self.p["max_dance_s"])
                return
            else:
                return
        if self.state == "LOCKOUT":
            if now - self.end_t < self.p["lockout_s"]:
                return
            if self.n_sends >= self.p["max_sends"]:
                self.state, self.reason = "SPENT", "max_sends %d reached" % self.p["max_sends"]
                return
        wait = self._wait_reason(now)
        self.state, self.reason = ("WAITING", wait) if wait else ("READY", "all preconditions hold")

    def _wait_reason(self, now):
        if self.n_sends >= self.p["max_sends"]:
            return "max_sends %d reached" % self.p["max_sends"]
        if self.cmd_bridge_present:
            return "go2_cmd_bridge is on the graph (never run both)"
        if self.sport_t is None or now - self.sport_t > self.p["stale_s"]:
            return "no fresh /sportmodestate"
        if self.low_t is None or now - self.low_t > self.p["stale_s"]:
            return "no fresh /lowstate (no thermal watch)"
        if self.error_code != BALANCE_STAND:
            return "error_code %s, not %d (balance stand)" % (self.error_code, BALANCE_STAND)
        if self.motors_ok != 12:
            return "%s of 12 motors in mode 1" % self.motors_ok
        if max(self.hips) >= self.p["hip_arm_max_c"]:
            return "rear hip %.0f C >= %.0f C" % (max(self.hips), self.p["hip_arm_max_c"])
        if self.handset_t is not None and now - self.handset_t < self.p["handset_quiet_s"]:
            return "handset active %.1f s ago" % (now - self.handset_t)
        if self.foreign_t is not None and now - self.foreign_t < self.p["foreign_quiet_s"]:
            return "foreign request api_id %d %.1f s ago" % (self.foreign_api, now - self.foreign_t)
        return None

    def status(self, now=None):
        st = {"state": self.state, "reason": self.reason, "n_sends": self.n_sends,
              "error_code": self.error_code, "hips": self.hips, "motors_ok": self.motors_ok,
              "last_response": self.last_response, "counts": dict(self.counts),
              "last_ignored": self.last_ignored}
        if self.state == "DANCING" and self.hips is not None and max(self.hips) >= 50.0:
            st["warning"] = "rear hip %.0f C during a dance -- the bridge cannot stop it; handset" % max(self.hips)
        return st
