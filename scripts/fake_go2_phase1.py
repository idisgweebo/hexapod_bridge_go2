#!/usr/bin/env python3
"""Fake Go2 + ROS 2 recorder for phase 1 stage 2 (scripts/phase1_loopback_test.sh). NO ROBOT.

    fake_go2_phase1.py SCENARIO RUN_S OUT.jsonl

THE BODY follows /go2/cmd_bridge/requests -- the DRY go2_cmd_bridge's log of exactly
what an armed bridge would have sent (same core, same tick). MOVE sets the target
(vx, wz) for MOVE_LAPSE_S (a Move does not persist, s12); STOPMOVE zeroes it;
STANDDOWN sets error_code 2006. First-order lag TAU_S, as the logic-level model
(test_phase1_logic_scenarios_offline.py). Crude on purpose: this is about the
feedback chain, not physics.

PUBLISHES (domain 0, loopback): /sportmodestate + /lowstate 50 Hz, /utlidar/robot_odom
150 Hz, /utlidar/imu 250 Hz (stamps GO2_AHEAD_S ahead: v1.1.15 measured -0.840 s on
20 Oct), /wirelesscontroller 10 Hz. All twelve motor modes 1, hips 35 C.

SCENARIOS (t_m = the first dry MOVE heard; t_0 = start):
    healthy        nothing injected                        C, S2-S6, S9
    lying          error_code 1001 throughout              S1
    handset        keys=288 once at t_m + 1.5 s            S7
    mode0          RR_hip motor mode 0 from t_0 + 14 s     S8a
    hip51          RR_hip 51 C from t_m + 1.0 s            S8b
    sport_freeze   /sportmodestate stops at t_m + 1.0 s    S8c
    pfc0           healthy + three TwistStamped published from ROS 2 at t_0 + 14/15/16 s   X

RECORDS (OUT.jsonl, one JSON object per line, t = time.time() at arrival): every
/go2/cmd_vel_stamped, /go2/cmd_bridge/requests, /go2/cmd_bridge/status,
/go2/robot_status, with every field the analyser compares, and "event" lines for each
injection. /go2/odom and /go2/imu ONLY in scenario pfc0 (phase X, the one place they are
compared): recorded in every phase they were ~16 MB of the 17 MB per run (30 Oct).

Plan: phase1_go2_feedback_plan.md §9, stage 2.
"""
import json
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from unitree_go.msg import LowState, SportModeState, WirelessController

from go2_adapter.qos import GO2_INPUT_QOS

GO2_AHEAD_S = 0.8403
MOVE_LAPSE_S = 0.3
TAU_S = 0.2
SCENARIOS = ("healthy", "lying", "handset", "mode0", "hip51", "sport_freeze", "pfc0")
# Distinct, non-round, all six components: a swapped or truncated field cannot pass.
PFC0_FROM_ROS2 = [((1761900000, 222222222), (0.7071067811865476, -0.1428571428571429, 0.3333333333333333,
                                              -2.718281828459045, 1.4142135623730951, -0.0009765625)),
                  ((1761900001, 999999999), (1e-9, -1e9, 0.1, 0.2, 0.30000000000000004, -0.0)),
                  ((1761900002, 1), (-1.5, 2.5, -3.5, 4.5, -5.5, 6.5))]


def stamp_of(h):
    return [h.stamp.sec, h.stamp.nanosec]


def level_int(lvl):
    return lvl[0] if isinstance(lvl, (bytes, bytearray)) else int(lvl)


class Fake(Node):

    def __init__(self, scenario, out):
        super().__init__("fake_go2_phase1")
        self.scenario, self.out = scenario, out
        self.lock = threading.Lock()
        self.t0 = time.time()
        self.t_m = None
        self.code = 1001 if scenario == "lying" else 100
        self.modes = [1] * 12
        self.hips = [35, 35]
        self.sport_frozen = False
        self.handset_sent = False
        self.cmd, self.last_move = (0.0, 0.0), None
        self.x = self.y = self.yaw = self.v = self.w = 0.0
        self.t_last = time.time()
        self.k = 0
        self.pfc0_sent = 0
        self.p_sport = self.create_publisher(SportModeState, "/sportmodestate", GO2_INPUT_QOS)
        self.p_low = self.create_publisher(LowState, "/lowstate", GO2_INPUT_QOS)
        self.p_odom = self.create_publisher(Odometry, "/utlidar/robot_odom", GO2_INPUT_QOS)
        self.p_imu = self.create_publisher(Imu, "/utlidar/imu", GO2_INPUT_QOS)
        self.p_hs = self.create_publisher(WirelessController, "/wirelesscontroller", 10)
        self.p_cmd = self.create_publisher(TwistStamped, "/go2/cmd_vel_stamped", 10) if scenario == "pfc0" else None
        self.create_subscription(String, "/go2/cmd_bridge/requests", self.on_req, 50)
        self.create_subscription(String, "/go2/cmd_bridge/status", self.on_bstatus, 50)
        self.create_subscription(TwistStamped, "/go2/cmd_vel_stamped", self.on_cmd, 50)
        self.create_subscription(DiagnosticArray, "/go2/robot_status", self.on_status, 50)
        if scenario == "pfc0":
            self.create_subscription(Odometry, "/go2/odom", self.on_odom, 50)
            self.create_subscription(Imu, "/go2/imu", self.on_imu, 50)
        self.create_timer(0.1, self.tick_10hz)
        self.write("start", scenario=scenario, go2_ahead_s=GO2_AHEAD_S)

    def write(self, kind, **kw):
        kw.update({"kind": kind, "t": time.time()})
        with self.lock:
            self.out.write(json.dumps(kw) + "\n")
            self.out.flush()

    def event(self, what):
        self.write("event", what=what, rel_t0=time.time() - self.t0,
                   rel_tm=None if self.t_m is None else time.time() - self.t_m)

    # ------------------------------------------------------------ recorder ---
    def on_req(self, m):
        r = json.loads(m.data)
        self.write("request", **{k: r[k] for k in ("name", "routed", "parameter") if k in r})
        with self.lock:
            if r["name"] == "MOVE":
                p = json.loads(r["parameter"])
                self.cmd, self.last_move = (p["x"], p["z"]), time.time()
                first = self.t_m is None
                if first:
                    self.t_m = time.time()
            elif r["name"] == "STOPMOVE":
                self.cmd, first = (0.0, 0.0), False
            elif r["name"] == "STANDDOWN":
                self.code, first = 2006, False
            else:
                first = False
        if first:
            self.event("first MOVE")

    def on_bstatus(self, m):
        self.write("bridge_status", status=json.loads(m.data))

    def on_cmd(self, m):
        t = m.twist
        self.write("cmd", stamp=stamp_of(m.header), frame_id=m.header.frame_id,
                   twist=[t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z])

    def on_status(self, m):
        self.write("robot_status", stamp=stamp_of(m.header), status=[
            {"level": level_int(s.level), "name": s.name, "message": s.message, "hardware_id": s.hardware_id,
             "values": [[kv.key, kv.value] for kv in s.values]} for s in m.status])

    def on_odom(self, m):
        p, q, tw = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        self.write("odom", stamp=stamp_of(m.header), frame_id=m.header.frame_id, child_frame_id=m.child_frame_id,
                   pose=[p.x, p.y, p.z, q.x, q.y, q.z, q.w],
                   twist=[tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.x, tw.angular.y, tw.angular.z],
                   pose_cov=list(m.pose.covariance), twist_cov=list(m.twist.covariance))

    def on_imu(self, m):
        o, w, a = m.orientation, m.angular_velocity, m.linear_acceleration
        self.write("imu", stamp=stamp_of(m.header), frame_id=m.header.frame_id,
                   orientation=[o.x, o.y, o.z, o.w], angular_velocity=[w.x, w.y, w.z],
                   linear_acceleration=[a.x, a.y, a.z], orientation_cov=list(m.orientation_covariance))

    # ---------------------------------------------------------------- body ---
    def go2_stamp(self, msg):
        t = time.time() + GO2_AHEAD_S
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9)) % 1000000000

    def tick_250hz(self):
        now = time.time()
        dt = now - self.t_last
        self.t_last = now
        rel0 = now - self.t0
        with self.lock:
            relm = None if self.t_m is None else now - self.t_m
            moving = self.last_move is not None and now - self.last_move <= MOVE_LAPSE_S
            tv, tw = self.cmd if moving else (0.0, 0.0)
        a = min(1.0, dt / TAU_S)
        self.v += (tv - self.v) * a
        self.w += (tw - self.w) * a
        self.yaw += self.w * dt
        self.x += self.v * dt * math.cos(self.yaw)
        self.y += self.v * dt * math.sin(self.yaw)

        if self.scenario == "mode0" and rel0 >= 14.0 and self.modes[6] == 1:
            self.modes[6] = 0
            self.event("RR_hip mode 0")
        if self.scenario == "hip51" and relm is not None and relm >= 1.0 and self.hips[0] != 51:
            self.hips[0] = 51
            self.event("RR_hip 51 C")
        if self.scenario == "sport_freeze" and relm is not None and relm >= 1.0 and not self.sport_frozen:
            self.sport_frozen = True
            self.event("sportmodestate frozen")

        self.k += 1
        i = Imu()
        self.go2_stamp(i)
        i.header.frame_id = "utlidar_imu"
        i.angular_velocity.z = self.w
        i.orientation.z, i.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.p_imu.publish(i)
        if self.k % 5 in (0, 2, 4):
            o = Odometry()
            self.go2_stamp(o)
            o.header.frame_id, o.child_frame_id = "odom", "base_link"      # MEASURED (s2, v1.1.11)
            o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
            o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
            self.p_odom.publish(o)
        if self.k % 5 == 0:
            if not self.sport_frozen:
                s = SportModeState()
                s.error_code = self.code
                s.body_height = 0.30 if self.code != 1001 else 0.08
                s.position = [float(self.x), float(self.y), 0.0]
                s.velocity = [float(self.v * math.cos(self.yaw)), float(self.v * math.sin(self.yaw)), 0.0]
                s.yaw_speed = float(self.w)
                s.imu_state.rpy = [0.0, 0.0, float(self.yaw)]
                self.p_sport.publish(s)
            low = LowState()
            for j in range(12):
                low.motor_state[j].mode = self.modes[j]
                low.motor_state[j].temperature = 35
            low.motor_state[6].temperature = self.hips[0]         # RR_hip
            low.motor_state[9].temperature = self.hips[1]         # RL_hip
            low.head = [0xFE, 0xEF]
            self.p_low.publish(low)

    def tick_10hz(self):
        rel0 = time.time() - self.t0
        w = WirelessController()
        if self.scenario == "handset" and not self.handset_sent and self.t_m is not None and time.time() - self.t_m >= 1.5:
            self.handset_sent = True
            w.keys = 288                          # L2+A as measured in s12: ONE message
            self.event("handset keys 288")
        self.p_hs.publish(w)
        if self.p_cmd is not None and self.pfc0_sent < len(PFC0_FROM_ROS2) and rel0 >= 14.0 + self.pfc0_sent:
            (sec, ns), tw = PFC0_FROM_ROS2[self.pfc0_sent]
            m = TwistStamped()
            m.header.stamp.sec, m.header.stamp.nanosec = sec, ns
            m.header.frame_id = "pfc0_from_ros2"
            (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z,
             m.twist.angular.x, m.twist.angular.y, m.twist.angular.z) = tw
            self.p_cmd.publish(m)
            self.write("pfc0_sent", stamp=[sec, ns], frame_id=m.header.frame_id, twist=list(tw))
            self.pfc0_sent += 1


def main():
    scenario, run_s, path = sys.argv[1], float(sys.argv[2]), sys.argv[3]
    if scenario not in SCENARIOS:
        sys.exit("unknown scenario %r; one of %s" % (scenario, ", ".join(SCENARIOS)))
    rclpy.init()
    out = open(path, "a")
    n = Fake(scenario, out)
    stop = threading.Event()

    def pump():
        nxt = time.time()
        while not stop.is_set():
            n.tick_250hz()
            nxt += 0.004
            time.sleep(max(0.0, nxt - time.time()))

    th = threading.Thread(target=pump, daemon=True)
    th.start()
    end = time.time() + run_s
    try:
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    stop.set()
    th.join(timeout=1.0)
    n.write("end", k=n.k)
    n.destroy_node()
    out.close()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
