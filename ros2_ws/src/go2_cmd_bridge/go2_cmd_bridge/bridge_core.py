#!/usr/bin/env python3
"""The decision logic of the Go2 velocity bridge. PURE: no ROS, no clock, no I/O.

Plan: gate7_go2_cmdvel_bridge_plan.md. Every input arrives as a method call that
carries its own `now`, and tick(now) returns the requests this 0.1 s tick would
send. The node (bridge_node.py) owns the clock and the DDS endpoints; this file
owns every decision, so every decision is testable offline
(scripts/test_cmd_bridge_offline.py) with synthetic time.

WHY DRY RUN IS DECIDED OUTSIDE THIS FILE
----------------------------------------
The core never knows whether it is armed. It always computes what it WOULD send,
and the node routes that either to the robot or to /go2/cmd_bridge/would_send.
So the dry run is not a simulation of the armed bridge -- it IS the armed bridge,
with one routing decision changed. A dry run that took a different code path would
prove nothing about the armed one.

STATES
------
    WAITING   a precondition fails (telemetry stale, not error_code 100, hips hot)
    IDLE      ready, no fresh command
    DRIVING   a fresh, non-zero command: one Move per tick
    ABORTED   latched: StopMove now, StandDown STANDDOWN_DELAY_S later, then silence
    YIELDED   latched: the handset moved. Nothing is ever sent again.

Latched states clear only by restarting the process. There is no re-arm input on
purpose: a re-arm topic would be a second command source.

Python 3.8-compatible (Foxy on the Orin is the deployment target): no match, no
dict | dict, no builtin generics in annotations.
"""
import json
import math

# ---------------------------------------------------------------------------
# The allowlist. Three api_ids, from unitree_ros2 @ 668d1ec5 ros2_sport_client.h
# (vendor source), all three flown in session 12 with code 0.
# No StandUp, no BalanceStand (decision D1: posture is the handset's), no Damp.
# ---------------------------------------------------------------------------
API = {
    "MOVE": 1008,
    "STOPMOVE": 1003,
    "STANDDOWN": 1005,      # abort path ONLY -- see _abort()
}

# Hard caps: the values flown in session 12 with Doug's approval. YAML may LOWER
# the clamps; a YAML value above these refuses to construct the core. Raising a
# cap is a code change and a commit, as it was in Gate 6.
HARD_VX_MAX = 0.3       # m/s
HARD_VYAW_MAX = 1.0     # rad/s

# Measured in session 12: error_code 100 = balance stand, the state every walk ran in.
BALANCE_STAND = 100

STANDDOWN_DELAY_S = 1.0     # the probe's StopMove -> StandDown gap, flown in s12
SUSTAIN_S = 0.1             # s12: a single 0.637 m/s sample on 7.6 mm of motion

DEFAULTS = {
    "vx_max": 0.3,
    "vyaw_max": 1.0,
    "cmd_timeout": 0.4,     # cmd_guard's watchdog_timeout
    "stale_s": 0.5,         # /sportmodestate is ~300 Hz; 0.5 s is ~150 lost samples
    "hip_abort_c": 50.0,    # rule 7 -- a ceiling on the SENSOR
    "hip_arm_max_c": 45.0,
    "geofence_m": 1.0,      # D4: the cable
    "handset_stick": 0.2,
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
    if p["vx_max"] > HARD_VX_MAX + 1e-9:
        raise ConfigError("vx_max %.3f exceeds the hard cap %.3f" % (p["vx_max"], HARD_VX_MAX))
    if p["vyaw_max"] > HARD_VYAW_MAX + 1e-9:
        raise ConfigError("vyaw_max %.3f exceeds the hard cap %.3f" % (p["vyaw_max"], HARD_VYAW_MAX))
    if p["hip_arm_max_c"] >= p["hip_abort_c"]:
        raise ConfigError("hip_arm_max_c must be below hip_abort_c")
    return p


def build_request(name, vx=0.0, vyaw=0.0):
    """Return (name, api_id, parameter_json). The ONLY way a request is made.
    Raises for anything outside the allowlist or the hard caps -- never clips:
    by the time a value reaches here it has already been clipped to the params."""
    if name not in API:
        raise RequestRejected("%r is not in the allowlist %s" % (name, sorted(API)))
    if name == "MOVE":
        for label, v, cap in (("vx", vx, HARD_VX_MAX), ("vyaw", vyaw, HARD_VYAW_MAX)):
            if not math.isfinite(v) or abs(v) > cap + 1e-9:
                raise RequestRejected("MOVE %s=%r outside hard cap +/-%s" % (label, v, cap))
        # Vendor keys x, y, z (ros2_sport_client.cpp). y is always 0: no strafing.
        return name, API[name], json.dumps({"x": float(vx), "y": 0.0, "z": float(vyaw)})
    if vx or vyaw:
        raise RequestRejected("%s takes no velocity" % name)
    return name, API[name], ""


def _clip(v, lim):
    return max(-lim, min(lim, v))


class SustainedGuard(object):
    """Trips only when |value| > limit continuously for sustain_s."""

    def __init__(self, limit, sustain_s=SUSTAIN_S):
        self.limit, self.sustain_s, self.since = limit, sustain_s, None

    def update(self, t, value):
        if abs(value) > self.limit:
            if self.since is None:
                self.since = t
            return t - self.since >= self.sustain_s
        self.since = None
        return False


class BridgeCore(object):

    def __init__(self, params=None):
        self.p = validate_params(params)
        self.state = "WAITING"
        self.reason = "no telemetry yet"
        self.engaged = False          # has ever been IDLE/DRIVING
        self.origin = None            # (px, py) when first engaged -- the geofence centre
        # latest inputs
        self.cmd = None               # (vx, vyaw) after clipping
        self.cmd_t = None
        self.sport_t = None
        self.low_t = None
        self.error_code = None
        self.pos = None
        self.hips = None              # (rr, rl)
        self.g_speed = SustainedGuard(2 * self.p["vx_max"])
        self.g_yaw = SustainedGuard(2 * self.p["vyaw_max"])
        self.trip = None              # an abort reason latched by an input callback
        self.standdown_at = None
        self.counts = {"twist": 0, "clipped": 0, "zeroed_axes": 0, "dropped_nonfinite": 0,
                       "MOVE": 0, "STOPMOVE": 0, "STANDDOWN": 0}

    # ------------------------------------------------------------- inputs ---
    def on_twist(self, now, lx, ly, lz, ax, ay, az):
        self.counts["twist"] += 1
        vals = (lx, ly, lz, ax, ay, az)
        if not all(math.isfinite(float(v)) for v in vals):
            self.counts["dropped_nonfinite"] += 1
            return
        if ly or lz or ax or ay:
            self.counts["zeroed_axes"] += 1
        vx, vyaw = _clip(float(lx), self.p["vx_max"]), _clip(float(az), self.p["vyaw_max"])
        if vx != lx or vyaw != az:
            self.counts["clipped"] += 1
        self.cmd, self.cmd_t = (vx, vyaw), now

    def on_sport(self, now, error_code, px, py, vx, vy, yaw_speed):
        self.sport_t, self.error_code, self.pos = now, int(error_code), (float(px), float(py))
        if self.g_speed.update(now, math.hypot(vx, vy)):
            self._latch("speed > %.2f m/s for %.1f s" % (self.g_speed.limit, SUSTAIN_S))
        if self.g_yaw.update(now, yaw_speed):
            self._latch("|yaw_speed| > %.2f rad/s for %.1f s" % (self.g_yaw.limit, SUSTAIN_S))

    def on_low(self, now, rr_c, rl_c):
        self.low_t, self.hips = now, (float(rr_c), float(rl_c))

    def on_handset(self, now, lx, ly, rx, ry, keys):
        if keys != 0 or max(abs(lx), abs(ly), abs(rx), abs(ry)) > self.p["handset_stick"]:
            # ⛔ Takes effect immediately and beats everything, including a pending
            # StandDown: a request landing after an L2+B could fight the operator.
            if self.state != "YIELDED":
                self.state, self.reason = "YIELDED", "handset activity (keys=%d)" % keys

    def _latch(self, reason):
        if self.trip is None:
            self.trip = reason

    # --------------------------------------------------------------- tick ---
    def tick(self, now):
        """Advance one tick. Returns the list of build_request() tuples to send."""
        if self.state == "YIELDED":
            return []
        if self.state == "ABORTED":
            if self.standdown_at is not None and now >= self.standdown_at:
                self.standdown_at = None
                return self._emit([build_request("STANDDOWN")])
            return []

        abort = self._abort_reason(now)
        if abort:
            return self._abort(now, abort)

        wait = self._wait_reason(now)
        if wait:
            # Leaving DRIVING because a precondition broke (e.g. error_code left 100)
            # sends NOTHING: the robot is no longer in the state Move was validated in,
            # and a StopMove into an unknown state is itself unmeasured.
            self.state, self.reason = "WAITING", wait
            return []

        if not self.engaged:
            self.engaged, self.origin = True, self.pos

        fresh = self.cmd_t is not None and now - self.cmd_t <= self.p["cmd_timeout"]
        moving = fresh and (self.cmd[0] != 0.0 or self.cmd[1] != 0.0)
        if moving:
            self.state, self.reason = "DRIVING", "fresh command"
            return self._emit([build_request("MOVE", vx=self.cmd[0], vyaw=self.cmd[1])])
        was_driving = self.state == "DRIVING"
        self.state = "IDLE"
        self.reason = "zero command" if fresh else "no fresh command"
        # Exactly ONE StopMove per DRIVING -> IDLE edge (D3), never one per tick.
        return self._emit([build_request("STOPMOVE")]) if was_driving else []

    def _abort_reason(self, now):
        if self.trip:
            return self.trip
        if self.hips is not None and max(self.hips) >= self.p["hip_abort_c"]:
            return "rear hip %.0f C >= %.0f C" % (max(self.hips), self.p["hip_abort_c"])
        if self.engaged:
            for name, t in (("/sportmodestate", self.sport_t), ("/lowstate", self.low_t)):
                if now - t > self.p["stale_s"]:
                    return "%s silent > %.1f s" % (name, self.p["stale_s"])
            if self.origin is not None and self.pos is not None:
                d = math.hypot(self.pos[0] - self.origin[0], self.pos[1] - self.origin[1])
                if d > self.p["geofence_m"]:
                    return "displacement %.2f m > geofence %.2f m" % (d, self.p["geofence_m"])
        return None

    def _wait_reason(self, now):
        if self.sport_t is None or now - self.sport_t > self.p["stale_s"]:
            return "no fresh /sportmodestate"
        if self.low_t is None or now - self.low_t > self.p["stale_s"]:
            return "no fresh /lowstate (no thermal watch)"
        if self.error_code != BALANCE_STAND:
            return "error_code %s, not %d (balance stand)" % (self.error_code, BALANCE_STAND)
        if not self.engaged and max(self.hips) >= self.p["hip_arm_max_c"]:
            return "rear hip %.0f C >= %.0f C arm limit" % (max(self.hips), self.p["hip_arm_max_c"])
        return None

    def _abort(self, now, reason):
        self.state, self.reason = "ABORTED", reason
        if not self.engaged:
            # Never commanded anything: the robot's posture is the operator's, and a
            # StandDown into an unknown state is not ours to send. Latch silently.
            return []
        self.standdown_at = now + STANDDOWN_DELAY_S
        return self._emit([build_request("STOPMOVE")])

    def _emit(self, reqs):
        for name, _, _ in reqs:
            self.counts[name] += 1
        return reqs

    def status(self):
        return {"state": self.state, "reason": self.reason, "engaged": self.engaged,
                "error_code": self.error_code, "hips": self.hips, "counts": dict(self.counts)}
