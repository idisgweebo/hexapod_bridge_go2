#!/usr/bin/env python3
"""Header re-stamping helpers. PURE LOGIC -- imports no rclpy.

The adapter's whole job on a standard-typed topic is to rewrite the header and
leave the payload alone. These helpers do the arithmetic; the nodes do the I/O.
"""

NANOS = 1_000_000_000


def to_seconds(sec, nanosec):
    """(sec, nanosec) -> float seconds."""
    return float(sec) + float(nanosec) / NANOS


def to_sec_nanosec(seconds):
    """float seconds -> (sec, nanosec), nanosec always in [0, 1e9).

    Uses divmod on an integer nanosecond count rather than int()/fmod so that
    negative inputs floor correctly instead of truncating toward zero. A stamp
    is never negative in practice, but a helper that quietly mangles one input
    class is a trap for whoever reuses it next."""
    total = int(round(float(seconds) * NANOS))
    sec, nanosec = divmod(total, NANOS)
    return int(sec), int(nanosec)


def corrected(sec, nanosec, offset_s):
    """Apply the Go2->local clock offset to a stamp.

    t_local_equivalent = t_go2 + offset, with offset positive (~+1396 s).
    Raises on a None offset rather than defaulting to zero: publishing an
    uncorrected Go2 stamp looks like a 23-minute-stale message to every ROS
    consumer, and silently substituting 0 would hide the bug."""
    if offset_s is None:
        raise ValueError("refusing to re-stamp with no converged clock offset")
    return to_sec_nanosec(to_seconds(sec, nanosec) + float(offset_s))
