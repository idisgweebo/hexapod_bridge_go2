#!/usr/bin/env python3
"""Go2 <-> local clock offset estimation. PURE LOGIC -- imports no rclpy.

WHY THIS IS A SEPARATE MODULE
-----------------------------
Everything here is arithmetic over plain floats, so the offline test suite can
exercise the real code with no ROS, no container and no robot. The node wrapper
(clock_offset_node.py) is a thin rclpy shell that holds no logic of its own.

THE PROBLEM
-----------
The Go2's clock runs far behind the local one -- measured 1325.669 s on 18 Sep
2026 and 1396.079 s on 28 Sep, i.e. it moves ~7 s/day between sessions. Any
stamp taken from a Go2 message is therefore ~23 minutes stale by local reckoning,
and a default 10 s tf2 buffer rejects it outright.

  t_local_equivalent = t_go2 + offset        (offset is POSITIVE, ~+1396 s)

WHY THE MINIMUM, NOT THE MEAN
-----------------------------
Each sample's (arrival - stamp) is the true offset PLUS a non-negative transport
delay. Noise is therefore one-sided, and the minimum is the sample that suffered
least delay -- the standard NTP-style estimator. The mean would be biased high by
exactly the average transport delay, and the Go2's delivery is measurably bursty
(the IMU shows 33 % jitter against a 4 ms mean) so that bias is not small.

WHY IT FREEZES
--------------
Measured within-session drift is ~42 ms/hour (0.0417 s/h powered ON, two
instruments). Over a 4-hour session that is ~170 ms -- far inside a 10 s tf2
buffer. A continuously-updating offset would instead inject a slow ramp into
every stamp we publish, which is worse than a small constant error: a constant
offset is a calibration, a ramp is a corruption.

WHY A STEP IS A FAULT, NOT AN UPDATE
------------------------------------
Between sessions the offset moves by seconds, and each Go2 power cycle adds a
per-boot step of roughly a quarter second on top of the free-running drift. If
the robot reboots mid-session, silently re-converging would produce a dataset
whose stamps are consistent and wrong. We latch a fault instead and require an
explicit re-arm, because a visible stop is recoverable and a plausible-looking
wrong dataset is not.
"""

CONVERGING = "CONVERGING"
CONVERGED = "CONVERGED"
FAULTED = "FAULTED"


class ClockOffsetEstimator:
    """Estimate a frozen Go2->local clock offset, and fault on a step.

    Usage:
        est = ClockOffsetEstimator()
        est.add(arrival_s, go2_stamp_s)   -> current state
        est.offset                        -> None until CONVERGED
    """

    def __init__(self, window=100, step_threshold=0.5, step_consecutive=10):
        if window < 2:
            raise ValueError("window must be >= 2")
        self.window = int(window)
        self.step_threshold = float(step_threshold)
        self.step_consecutive = int(step_consecutive)
        self._deltas = []
        self._offset = None
        self._state = CONVERGING
        self._bad_run = 0
        self.n_samples = 0
        self.worst_residual = 0.0

    @property
    def offset(self):
        """The frozen offset in seconds, or None if not converged.

        Returns None rather than a partial estimate on purpose: a caller that
        gets a number cannot tell a converged one from a guess, and downstream
        every stamp depends on it."""
        return self._offset if self._state == CONVERGED else None

    @property
    def state(self):
        return self._state

    def add(self, arrival_s, go2_stamp_s):
        """Feed one (local arrival time, Go2 header stamp) pair. Both seconds."""
        self.n_samples += 1
        delta = float(arrival_s) - float(go2_stamp_s)

        if self._state == CONVERGING:
            self._deltas.append(delta)
            if len(self._deltas) >= self.window:
                self._offset = min(self._deltas)
                self._state = CONVERGED
                self._deltas = []
            return self._state

        if self._state == FAULTED:
            return self._state

        # CONVERGED: watch for a step.
        residual = delta - self._offset
        if abs(residual) > abs(self.worst_residual):
            self.worst_residual = residual
        if abs(residual) > self.step_threshold:
            self._bad_run += 1
            if self._bad_run >= self.step_consecutive:
                self._state = FAULTED
        else:
            self._bad_run = 0
        return self._state

    def rearm(self):
        """Explicitly discard the estimate and start again.

        Deliberately NOT automatic. A step means the robot rebooted or the clock
        jumped; a human should know that happened before data flows again."""
        self._deltas = []
        self._offset = None
        self._state = CONVERGING
        self._bad_run = 0
        self.worst_residual = 0.0
        return self._state
