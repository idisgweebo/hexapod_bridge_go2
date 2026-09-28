#!/usr/bin/env python3
"""Gate 5 -- rear-hip cooldown logger. READ-ONLY: subscribes, never publishes.

WHY
---
The whole thermal budget rests on "tau ~45-50 min", which is a guess from TWO
points (gate5_session7_plan.md 4b, flagged there as inference). This logs the
actual curve from a hot sit-down to ambient so tau becomes measured.

RESOLUTION TRICK
----------------
LowState.motor_state[].temperature is an int8 -- 1 C quantisation. But /lowstate
runs ~500 Hz, and if the underlying value dithers between adjacent counts, the
per-second MEAN recovers sub-count resolution. Logging the mean, the min, the max
and n per second lets the analysis tell a dithering channel (min != max) from a
hard-quantised one (min == max), instead of assuming.
"""
import argparse, csv, os, sys, time

JOINT = ["FR_hip","FR_thigh","FR_calf","FL_hip","FL_thigh","FL_calf",
         "RR_hip","RR_thigh","RR_calf","RL_hip","RL_thigh","RL_calf"]
RR_HIP, RL_HIP = 6, 9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/logs/s8_cooldown.csv")
    ap.add_argument("--duration", type=float, default=4200.0, help="seconds (default 70 min)")
    ap.add_argument("--bin", type=float, default=1.0, help="seconds per logged row")
    ap.add_argument("--note", default="")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from unitree_go.msg import LowState

    class Cool(Node):
        def __init__(self):
            super().__init__("go2_cooldown_log")
            q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, depth=50)
            self.buf = []
            self.create_subscription(LowState, "/lowstate", self.cb, q)
        def cb(self, m):
            self.buf.append(([m.motor_state[i].temperature for i in range(12)],
                             int(m.temperature_ntc1), int(m.temperature_ntc2),
                             float(m.power_v), float(m.power_a),
                             max(abs(m.motor_state[i].q) for i in range(12))))

    rclpy.init()
    n = Cool()
    t0 = time.time()
    hdr = (["t_s", "utc", "n"]
           + [f"{j}_mean" for j in JOINT]
           + [f"{j}_min" for j in JOINT] + [f"{j}_max" for j in JOINT]
           + ["ntc1", "ntc2", "power_v", "power_a", "max_abs_q", "note"])
    fresh = not os.path.exists(args.out)
    f = open(args.out, "a", newline="")
    w = csv.writer(f)
    if fresh:
        w.writerow(hdr)
    print(f"cooldown log -> {args.out}  (bin {args.bin}s, up to {args.duration/60:.0f} min)")
    f.flush()

    next_t = t0 + args.bin
    rows = 0
    try:
        while rclpy.ok() and time.time() - t0 < args.duration:
            rclpy.spin_once(n, timeout_sec=0.1)
            now = time.time()
            if now < next_t:
                continue
            batch, n.buf = n.buf, []
            next_t = now + args.bin
            if not batch:
                continue
            k = len(batch)
            means = [sum(b[0][i] for b in batch) / k for i in range(12)]
            mins = [min(b[0][i] for b in batch) for i in range(12)]
            maxs = [max(b[0][i] for b in batch) for i in range(12)]
            w.writerow([f"{now - t0:.2f}", time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)), k]
                       + [f"{v:.4f}" for v in means] + mins + maxs
                       + [f"{sum(b[1] for b in batch)/k:.2f}", f"{sum(b[2] for b in batch)/k:.2f}",
                          f"{sum(b[3] for b in batch)/k:.3f}", f"{sum(b[4] for b in batch)/k:.3f}",
                          f"{sum(b[5] for b in batch)/k:.4f}", args.note])
            f.flush()
            rows += 1
            if rows % 30 == 0:
                print(f"  {(now-t0)/60:5.1f} min  RR_hip {means[RR_HIP]:5.2f}  "
                      f"RL_hip {means[RL_HIP]:5.2f}  (n={k}/bin)", flush=True)
    except KeyboardInterrupt:
        pass
    f.close()
    print(f"done: {rows} rows over {(time.time()-t0)/60:.1f} min")
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
