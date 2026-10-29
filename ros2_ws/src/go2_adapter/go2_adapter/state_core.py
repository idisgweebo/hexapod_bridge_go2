#!/usr/bin/env python3
"""Go2 -> hexapod feedback: motion, fault and echo derivation. PURE LOGIC -- imports no rclpy.

Plan: phase1_go2_feedback_plan.md -- §3 (/go2/robot_status key schema v1) and D10
(the motion rule). go2_state_node.py is the thin rclpy shell; every decision is
here so scripts/test_go2_state_offline.py can exercise it with synthetic time, and
scripts/go2_motion_replay.py can run it over recorded walks.

WHAT "MOVING" RESTS ON (D10)
----------------------------
Measured motion only: SportModeState's velocity estimate, CORROBORATED by the
displacement of SportModeState's own position/yaw over the same window. Never the
commanded velocity (that is what RECEIVED is for) and never dq (gotcha #21).
Each velocity claim needs its own displacement: speed with translation, yaw rate
with heading change. A balance-stand sway can show a yaw-rate blip and a
centimetre of translation at once; pairing them stops one vouching for the other.
⚠️ Both quantities come from the Go2's own state estimator. This is a check against
a single-sample spike (s12: one 0.637 m/s sample on 7.6 mm of motion), NOT an
independent measurement of the body. Self-consistency is not accuracy.

WHY A STALE INPUT IS "UNKNOWN", NEVER "STOPPED"
-----------------------------------------------
The thermal-watch defect (15 Oct): a dead feed read as a calm robot. Here a stale
/sportmodestate makes motion UNKNOWN and the level STALE, and the hexapod applies
its own staleness on top for the case where this whole node goes quiet.

Python 3.8-compatible (Foxy is still a deployment target for this package).
"""
import collections
import math

SCHEMA_VERSION = 1

MOVING, STOPPED, UNKNOWN = "MOVING", "STOPPED", "UNKNOWN"

# diagnostic_msgs/DiagnosticStatus levels. STALE is native to the message.
OK, WARN, ERROR, STALE = 0, 1, 2, 3

# SportModeState.error_code is a STATE code, not a fault code (CLAUDE.md rule 9).
# An unseen value is a WARN "unrecognised", never an ERROR.
KNOWN_SPORT_CODES = {
    100: "balance stand",               # rule 9
    1001: "lying",                      # rule 9
    1002: "stand-lock",                 # rule 9
    2006: "standing down",              # rule 9
    1006: "seen during Dance 1",        # 27/28 Oct captures, v1.1.15; meaning not established
    1013: "seen at the end of Dance 1", # 28 Oct S3/S4, v1.1.15; meaning not established
}

JOINTS = ("FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
          "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf")
HEALTHY_MOTOR_MODE = 1      # rule 9: health = all twelve motor_state[i].mode == 1

DEFAULTS = {
    # D10 -- STARTING values. ⛔ Fixed by --replay over the s13 walks and then frozen
    # before stage 5; see scripts/go2_motion_replay.py and the plan's PFC-11.
    "v_mov": 0.05,          # m/s
    "w_mov": 0.10,          # rad/s
    "d_mov": 0.01,          # m of translation over window_s
    "a_mov": 0.02,          # rad of heading change over window_s
    "window_s": 0.3,        # the D10 sustain window, both edges
    # Input staleness. /sportmodestate and /lowstate match go2_cmd_bridge's stale_s.
    "sport_stale_s": 0.5,
    "low_stale_s": 0.5,
    "bridge_stale_s": 1.5,  # the bridge's status is 5 Hz since 29 Oct (was 2): ~7 missed publishes
    # Thermal, matching go2_cmd_bridge: 45 C arm limit (WARN here), 50 C abort (rule 8).
    "hip_warn_c": 45.0,
    "hip_fault_c": 50.0,
}


class ConfigError(ValueError):
    pass


def validate_params(overrides=None):
    p = dict(DEFAULTS)
    for k, v in (overrides or {}).items():
        if k not in DEFAULTS:
            raise ConfigError("unknown parameter %r" % (k,))
        p[k] = float(v)
    for k, v in p.items():
        if not math.isfinite(v) or v <= 0:
            raise ConfigError("%s must be a positive finite number, got %r" % (k, v))
    if p["hip_warn_c"] >= p["hip_fault_c"]:
        raise ConfigError("hip_warn_c must be below hip_fault_c")
    return p


def wrap_pi(a):
    """Angle into (-pi, pi]."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def fmt_stamp(sec, nanosec):
    """The echo's wire form, "sec.nnnnnnnnn". The hexapod formats its own stamps the
    same way and compares STRINGS, so no float rounding can make two IDs collide."""
    return "%d.%09d" % (int(sec), int(nanosec))


class Throttle(object):
    """Publish-time rate limit on a faster input, slot-based so the AVERAGE rate is
    the target. A plain "1/rate since the last publish" limiter loses up to one input
    period per output (150 Hz in, 20 Hz target -> ~17.6 Hz out)."""

    def __init__(self, rate_hz):
        if not rate_hz > 0:
            raise ConfigError("rate must be > 0")
        self.period = 1.0 / float(rate_hz)
        self.next_t = None

    def allow(self, now):
        if self.next_t is None or now >= self.next_t:
            # Fell behind by more than a period (input gap): resynchronise rather
            # than emit a burst to "catch up".
            if self.next_t is None or now - self.next_t > self.period:
                self.next_t = now + self.period
            else:
                self.next_t += self.period
            return True
        return False


class MotionDetector(object):
    """D10: MOVING iff a velocity condition AND its own displacement agree, and the
    state changes only after the candidate has disagreed with it for window_s."""

    def __init__(self, p):
        self.p = p
        self.hist = collections.deque()     # (t, px, py, yaw)
        self.state = UNKNOWN
        self.since = None                   # when the candidate began to disagree
        self.last_t = None
        self.candidate = None
        self.detail = {}

    def reset(self):
        self.hist.clear()
        self.state, self.since, self.candidate = UNKNOWN, None, None

    def update(self, t, vx, vy, yaw_speed, px, py, yaw):
        p = self.p
        if self.last_t is not None and t - self.last_t > p["window_s"]:
            # A gap longer than the window: the history no longer describes one
            # continuous motion. Start over from UNKNOWN rather than bridge it.
            self.reset()
        self.last_t = t
        self.hist.append((t, px, py, yaw))
        while len(self.hist) > 1 and self.hist[1][0] <= t - p["window_s"]:
            self.hist.popleft()
        t0, x0, y0, yaw0 = self.hist[0]
        trans = math.hypot(px - x0, py - y0)
        turn = abs(wrap_pi(yaw - yaw0))
        speed = math.hypot(vx, vy)
        walking = speed > p["v_mov"] and trans >= p["d_mov"]
        turning = abs(yaw_speed) > p["w_mov"] and turn >= p["a_mov"]
        cand = MOVING if (walking or turning) else STOPPED
        self.candidate = cand
        self.detail = {"speed": speed, "yaw_speed": yaw_speed, "trans": trans,
                       "turn": turn, "span": t - t0}

        if cand == self.state:
            self.since = None
        elif self.since is None:
            self.since = t
        elif t - self.since >= p["window_s"] - 1e-9:
            self.state, self.since = cand, None
        return self.state


class Go2StateCore(object):
    """Holds the latest inputs; status(now) composes /go2/robot_status schema v1.

    Inputs carry their own arrival time `now`, as in go2_cmd_bridge's core."""

    def __init__(self, params=None):
        self.p = validate_params(params)
        self.motion = MotionDetector(self.p)
        self.sport_t = None
        self.sport = None           # dict: error_code, vx, vy, yaw_speed
        self.low_t = None
        self.modes = None
        self.hips = None            # (rr, rl)
        self.bridge_t = None
        self.bridge = None          # the bridge's status JSON, parsed
        self.bridge_bad = 0         # unparseable bridge status messages
        self.clock_offset = None
        self.clock_state = None
        self.counts = {"sport": 0, "low": 0, "bridge": 0}

    # ------------------------------------------------------------- inputs ---
    def on_sport(self, now, error_code, vx, vy, yaw_speed, px, py, yaw):
        self.sport_t = now
        self.sport = {"error_code": int(error_code), "vx": float(vx), "vy": float(vy),
                      "yaw_speed": float(yaw_speed)}
        self.counts["sport"] += 1
        self.motion.update(now, float(vx), float(vy), float(yaw_speed),
                           float(px), float(py), float(yaw))

    def on_low(self, now, modes, rr_c, rl_c):
        self.low_t = now
        self.modes = [int(m) for m in modes]
        self.hips = (float(rr_c), float(rl_c))
        self.counts["low"] += 1

    def on_bridge(self, now, status):
        """status: the bridge's JSON already parsed to a dict, or None if it did not
        parse. A bad message counts and does NOT refresh the bridge's freshness."""
        if not isinstance(status, dict) or "state" not in status:
            self.bridge_bad += 1
            return
        self.bridge_t, self.bridge = now, status
        self.counts["bridge"] += 1

    def on_clock(self, offset, state):
        self.clock_offset, self.clock_state = offset, state

    # ------------------------------------------------------------- output ---
    def status(self, now):
        """Return (level, message, values): values is an ordered list of (key, str)."""
        p = self.p
        faults, warns = [], []
        level = OK

        sport_fresh = self.sport_t is not None and now - self.sport_t <= p["sport_stale_s"]
        low_fresh = self.low_t is not None and now - self.low_t <= p["low_stale_s"]
        bridge_fresh = self.bridge_t is not None and now - self.bridge_t <= p["bridge_stale_s"]

        if not sport_fresh:
            faults.append("sport_stale")
            level = max(level, STALE)
        if not low_fresh:
            faults.append("low_stale")
            level = max(level, STALE)
        if not bridge_fresh:
            # Without a live bridge nothing the hexapod sends can be executed, and
            # the echo cannot be read. On a read-only session this is the truth.
            faults.append("bridge_stale")
            level = max(level, STALE)

        motion = self.motion.state if sport_fresh else UNKNOWN

        if low_fresh:
            bad = [JOINTS[i] for i, m in enumerate(self.modes) if m != HEALTHY_MOTOR_MODE]
            if len(self.modes) != len(JOINTS):
                bad = ["motor_count_%d" % len(self.modes)]
            if bad:
                faults.append("motor_mode:" + ",".join(bad))
                level = max(level, ERROR)
            hot = max(self.hips)
            if hot >= p["hip_fault_c"]:
                faults.append("rear_hip_hot:%.0f" % hot)
                level = max(level, ERROR)
            elif hot >= p["hip_warn_c"]:
                warns.append("rear_hip_warm:%.0f" % hot)
                level = max(level, WARN)

        code = self.sport["error_code"] if self.sport else None
        if sport_fresh and code not in KNOWN_SPORT_CODES:
            warns.append("sport_code_unrecognised:%s" % code)
            level = max(level, WARN)

        b = self.bridge if bridge_fresh else None
        if b is not None:
            if b.get("state") == "ABORTED":
                faults.append("bridge_aborted:%s" % b.get("reason", ""))
                level = max(level, ERROR)
            elif b.get("state") == "YIELDED":
                faults.append("bridge_yielded")
                level = max(level, ERROR)
            elif b.get("state") == "WAITING":
                # Not a fault: WAITING is the normal state of a bridge whose robot is
                # lying down. It is reported so the hexapod can see WHY nothing moves.
                warns.append("bridge_waiting:%s" % b.get("reason", ""))
                level = max(level, WARN)

        if self.clock_state == "FAULTED":
            faults.append("clock_step")
            level = max(level, ERROR)
        elif self.clock_state != "CONVERGED":
            warns.append("clock_converging")
            level = max(level, WARN)

        stamp = b.get("cmd_last_stamp") if b else None
        age = b.get("cmd_age_s") if b else None
        sp = self.sport if sport_fresh else None
        values = [
            ("schema_version", str(SCHEMA_VERSION)),
            ("cmd_last_stamp", fmt_stamp(*stamp) if stamp else ""),
            ("cmd_age_s", "" if age is None else "%.3f" % age),
            ("bridge_state", b.get("state", "") if b else "NONE"),
            ("bridge_reason", b.get("reason", "") if b else ""),
            ("bridge_input", b.get("input_type", "") if b else ""),
            # So a hexapod-side sender can refuse to stream into a bridge armed when it
            # expected a dry one (or the reverse), as cmd_vel_profile.py's preflight does.
            ("bridge_armed", ("true" if b.get("armed") else "false") if b and "armed" in b else ""),
            ("motion", motion),
            ("speed_mps", "%.3f" % math.hypot(sp["vx"], sp["vy"]) if sp else ""),
            ("yaw_rate_rps", "%.3f" % sp["yaw_speed"] if sp else ""),
            ("sport_state_code", "" if code is None or not sport_fresh else str(code)),
            ("fault", ";".join(faults) if faults else "none"),
            ("warn", ";".join(warns) if warns else "none"),
            ("rr_hip_c", "%.0f" % self.hips[0] if low_fresh else ""),
            ("rl_hip_c", "%.0f" % self.hips[1] if low_fresh else ""),
            ("go2_clock_offset_s", "" if self.clock_offset is None else "%.4f" % self.clock_offset),
            ("clock_state", self.clock_state or "NONE"),
            ("odom_covariance_known", "false"),     # D8: the zeros pass through, and we say so
        ]
        message = ("%s; motion %s" % (";".join(faults), motion)) if faults else ("motion %s" % motion)
        return level, message, values
