#!/usr/bin/env python3
"""Gate 8 -- SUBSCRIBE-ONLY recorder of what crosses ros1_bridge onto ROS 2.

    gate8_ros2_recorder.py OUT.jsonl RUN_S

Writes one JSON line per message, stamped with time.time() on arrival:
  {"t", "topic": "/go2/cmd_vel", "x", "y", "z", "wx", "wy", "wz"}
  {"t", "topic": "/go2/cmd_bridge/requests", "data": {...}}     (go2_cmd_bridge's log of what it sent / would send)
  {"t", "topic": "/go2/cmd_bridge/status",   "data": {...}}

Creates NO publisher (stage 2 runs it with the Go2 cable in). Plan: Claude_Research gate8_hexapod_to_go2_plan.md.
Python 3.8-compatible.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def main():
    out_path, run_s = sys.argv[1], float(sys.argv[2])
    out = open(out_path, "a")

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
    write({"t": time.time(), "topic": "_recorder", "data": "start, run_s %.1f" % run_s})
    t_end = time.time() + run_s
    try:
        while time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    write({"t": time.time(), "topic": "_recorder", "data": "end"})
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
