#!/usr/bin/env python3
"""Gate 8 -- SUBSCRIBE-ONLY recorder of what crosses ros1_bridge onto ROS 2 (and, with --robot, what the Go2 does).

    gate8_ros2_recorder.py OUT.jsonl RUN_S [--robot]
    gate8_ros2_recorder.py --snapshot          # posture-first: one read of the Go2's state, printed, then exit

Writes one JSON line per message, stamped with time.time() on arrival:
  {"t", "topic": "/go2/cmd_vel", "x", "y", "z", "wx", "wy", "wz"}
  {"t", "topic": "/go2/cmd_bridge/requests", "data": {...}}     (go2_cmd_bridge's log of what it sent / would send)
  {"t", "topic": "/go2/cmd_bridge/status",   "data": {...}}
 --robot adds (unitree_go msgs needed):
  {"t", "topic": "/sportmodestate", "ec", "px", "py", "vx", "vy", "yaw", "yaw_speed", "bh"}   every 3rd message (~100 Hz;
      session 14's 5 Hz decimation made release positions interpolations)
  {"t", "topic": "/lowstate", "temps": [12], "modes": [12], "power_v", "tick"}                 2 Hz
  {"t", "topic": "/wirelesscontroller", "keys", "lx", "ly", "rx", "ry"}                         every message (an idle handset sends none)

Creates NO publisher -- it runs on the Go2 cable. Plan: Claude_Research gate8_hexapod_to_go2_plan.md.
Python 3.8-compatible.
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

SPORT_EVERY = 3
LOW_PERIOD = 0.5


def snapshot():
    from unitree_go.msg import SportModeState, LowState
    rclpy.init()
    n = Node("gate8_snapshot")
    got = {}
    n.create_subscription(SportModeState, "/sportmodestate", lambda m: got.setdefault("s", m), 10)
    n.create_subscription(LowState, "/lowstate", lambda m: got.setdefault("l", m), 10)
    t_end = time.time() + 5.0
    while time.time() < t_end and len(got) < 2:
        rclpy.spin_once(n, timeout_sec=0.05)
    print("utc %s" % time.strftime("%H:%M:%SZ", time.gmtime()))
    if "s" in got:
        s = got["s"]
        print("body_height %.4f error_code %d mode %d position %s yaw %.4f"
              % (s.body_height, s.error_code, s.mode, [round(v, 4) for v in s.position], s.imu_state.rpy[2]))
    else:
        print("NO /sportmodestate in 5 s")
    if "l" in got:
        lo = got["l"]
        print("head %s tick %d power_v %.3f" % ([hex(b) for b in lo.head], lo.tick, lo.power_v))
        print("temps %s modes %s" % ([m.temperature for m in lo.motor_state[:12]], [m.mode for m in lo.motor_state[:12]]))
        print("rear hips RR %d RL %d C" % (lo.motor_state[6].temperature, lo.motor_state[9].temperature))
    else:
        print("NO /lowstate in 5 s")
    n.destroy_node()
    rclpy.shutdown()
    return 0 if len(got) == 2 else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?")
    ap.add_argument("run_s", nargs="?", type=float)
    ap.add_argument("--robot", action="store_true")
    ap.add_argument("--snapshot", action="store_true")
    a = ap.parse_args()
    if a.snapshot:
        return snapshot()
    if a.out is None or a.run_s is None:
        ap.error("OUT and RUN_S are required")
    out = open(a.out, "a")

    def write(row):
        out.write(json.dumps(row) + "\n")
        out.flush()

    rclpy.init()
    n = Node("gate8_ros2_recorder")

    def on_twist(m):
        write({"t": time.time(), "topic": "/go2/cmd_vel", "x": m.linear.x, "y": m.linear.y, "z": m.linear.z,
               "wx": m.angular.x, "wy": m.angular.y, "wz": m.angular.z})

    def on_string(topic):
        def cb(m):
            try:
                data = json.loads(m.data)
            except ValueError:
                data = m.data
            write({"t": time.time(), "topic": topic, "data": data})
        return cb

    n.create_subscription(Twist, "/go2/cmd_vel", on_twist, 50)
    n.create_subscription(String, "/go2/cmd_bridge/requests", on_string("/go2/cmd_bridge/requests"), 50)
    n.create_subscription(String, "/go2/cmd_bridge/status", on_string("/go2/cmd_bridge/status"), 50)

    if a.robot:
        from unitree_go.msg import SportModeState, LowState, WirelessController
        cnt = {"s": 0, "l": 0.0}

        def on_sport(m):
            cnt["s"] += 1
            if cnt["s"] % SPORT_EVERY:
                return
            write({"t": time.time(), "topic": "/sportmodestate", "ec": m.error_code, "px": m.position[0], "py": m.position[1],
                   "vx": m.velocity[0], "vy": m.velocity[1], "yaw": m.imu_state.rpy[2], "yaw_speed": m.yaw_speed,
                   "bh": m.body_height})

        def on_low(m):
            now = time.time()
            if now - cnt["l"] < LOW_PERIOD:
                return
            cnt["l"] = now
            write({"t": now, "topic": "/lowstate", "temps": [x.temperature for x in m.motor_state[:12]],
                   "modes": [x.mode for x in m.motor_state[:12]], "power_v": m.power_v, "tick": m.tick})

        def on_hs(m):
            write({"t": time.time(), "topic": "/wirelesscontroller", "keys": m.keys, "lx": m.lx, "ly": m.ly, "rx": m.rx, "ry": m.ry})

        n.create_subscription(SportModeState, "/sportmodestate", on_sport, 50)
        n.create_subscription(LowState, "/lowstate", on_low, 10)
        n.create_subscription(WirelessController, "/wirelesscontroller", on_hs, 50)

    write({"t": time.time(), "topic": "_recorder", "data": "start, run_s %.1f, robot %s" % (a.run_s, a.robot)})
    t_end = time.time() + a.run_s
    try:
        while time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    write({"t": time.time(), "topic": "_recorder", "data": "end"})
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
