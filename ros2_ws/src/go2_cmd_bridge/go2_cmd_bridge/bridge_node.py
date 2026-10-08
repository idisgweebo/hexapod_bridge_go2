#!/usr/bin/env python3
"""go2_cmd_bridge_node: /go2/cmd_vel (Twist) -> streamed Move on the Go2.

⛔ The SECOND file in this project that can publish to the Go2 (after
scripts/go2_sport_probe.py). Plan: gate7_go2_cmdvel_bridge_plan.md.

THIS FILE DECIDES NOTHING
-------------------------
Every decision -- clamps, watchdog, aborts, yield -- is in bridge_core.py, which is
pure and tested offline. This node only:
  * stamps each arriving message with time.time() and hands it to the core,
  * calls core.tick() at 10 Hz,
  * routes what tick() returns: to /api/sport/request when armed, and ALWAYS to
    /go2/cmd_bridge/requests as a log line,
  * publishes the core's status at 2 Hz on /go2/cmd_bridge/status.

DRY RUN BY DEFAULT
------------------
With armed:=false (the default) the robot publisher is NEVER CREATED -- not created
and unused, absent. A bug in the routing cannot publish through a publisher that
does not exist. The requests topic carries exactly what an armed bridge would have
sent, from the same core and the same tick.

ARMED
-----
The robot publisher must MATCH at least one subscriber (the sport server) before
anything is sent; until then requests are dropped and counted. An unmatched
publish is indistinguishable from a refused one (gate 6, U5).

Python 3.8-compatible (Foxy target).
"""
import json
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from unitree_go.msg import SportModeState, LowState, WirelessController
from unitree_api.msg import Request, Response

from go2_cmd_bridge.bridge_core import BridgeCore, DEFAULTS, ConfigError, build_request

RATE_HZ = 10.0          # session 12: 10 Hz streamed Moves walked
STATUS_EVERY = 5        # ticks -> 2 Hz
RR_HIP, RL_HIP = 6, 9   # motor indices, as go2_thermal_watch / session 6


class BridgeNode(Node):

    def __init__(self):
        super().__init__("go2_cmd_bridge_node")
        self.armed = bool(self.declare_parameter("armed", False).value)
        overrides = {}
        for k, v in DEFAULTS.items():
            overrides[k] = float(self.declare_parameter(k, float(v)).value)
        try:
            self.core = BridgeCore(overrides)
        except ConfigError as e:
            # Loud and fatal: a bridge that logged a bad clamp and carried on with
            # defaults would be worse than one that never started.
            self.get_logger().fatal("refusing to start: %s" % e)
            raise

        self.robot_pub = None
        if self.armed:
            # ⛔ The single robot publisher in this package. Literal topic; audited by
            # scripts/test_cmd_bridge_offline.py.
            self.robot_pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.req_pub = self.create_publisher(String, "/go2/cmd_bridge/requests", 10)
        self.status_pub = self.create_publisher(String, "/go2/cmd_bridge/status", 10)

        self.create_subscription(Twist, "/go2/cmd_vel", self.on_twist, 10)
        self.create_subscription(SportModeState, "/sportmodestate", self.on_sport, 10)
        self.create_subscription(LowState, "/lowstate", self.on_low, 10)
        self.create_subscription(WirelessController, "/wirelesscontroller", self.on_hs, 10)
        self.create_subscription(Response, "/api/sport/response", self.on_resp, 10)

        self.n_tick = 0
        self.closing = False          # set by stop_on_exit BEFORE its StopMove; silences on_tick
        self.dropped_unmatched = 0
        self.last_state = None
        self.create_timer(1.0 / RATE_HZ, self.on_tick)
        self.get_logger().info(
            "go2_cmd_bridge up: %s, params %s" % ("⛔ ARMED" if self.armed else "DRY RUN (robot publisher not created)",
                                                   json.dumps(self.core.p, sort_keys=True)))

    # ------------------------------------------------------------- inputs ---
    def on_twist(self, m):
        self.core.on_twist(time.time(), m.linear.x, m.linear.y, m.linear.z,
                           m.angular.x, m.angular.y, m.angular.z)

    def on_sport(self, m):
        self.core.on_sport(time.time(), m.error_code, m.position[0], m.position[1],
                           m.velocity[0], m.velocity[1], m.yaw_speed)

    def on_low(self, m):
        self.core.on_low(time.time(), m.motor_state[RR_HIP].temperature,
                         m.motor_state[RL_HIP].temperature)

    def on_hs(self, m):
        self.core.on_handset(time.time(), m.lx, m.ly, m.rx, m.ry, m.keys)

    def on_resp(self, m):
        if m.header.status.code != 0:
            self.get_logger().warning("sport response api_id %d code %d: %s"
                                      % (m.header.identity.api_id, m.header.status.code, m.data[:80]))

    # --------------------------------------------------------------- tick ---
    def on_tick(self):
        if self.closing:
            return
        now = time.time()
        for req in self.core.tick(now):
            self.route(now, req)
        st = self.core.status()
        if st["state"] != self.last_state:
            self.get_logger().info("state %s -> %s (%s)" % (self.last_state, st["state"], st["reason"]))
            self.last_state = st["state"]
        self.n_tick += 1
        if self.n_tick % STATUS_EVERY == 0:
            st["armed"] = self.armed
            st["dropped_unmatched"] = self.dropped_unmatched
            self.status_pub.publish(String(data=json.dumps(st)))

    def route(self, now, req):
        name, api_id, parameter = req
        routed = "dry"
        if self.robot_pub is not None:
            if self.robot_pub.get_subscription_count() < 1:
                self.dropped_unmatched += 1
                routed = "dropped_unmatched"
            else:
                r = Request()
                r.header.identity.api_id = api_id
                r.parameter = parameter
                self.robot_pub.publish(r)
                routed = "robot"
        self.req_pub.publish(String(data=json.dumps(
            {"t": round(now, 6), "name": name, "api_id": api_id, "parameter": parameter, "routed": routed})))
        if name != "MOVE":
            self.get_logger().info("%s %s (%s)" % (routed, name, self.core.reason))

    def stop_on_exit(self):
        """Best effort, on a NORMAL stop only: one StopMove if we were driving.
        ⚠️ Not a safety mechanism -- SIGKILL, a crash or a dead link skip it. Whether
        the robot stops without it is PV4, measured, not assumed."""
        # ⛔ Silence the tick FIRST. Integration test K (8 Oct) caught the exit StopMove
        # being followed 3 ms later by three more Moves: the flush spun the executor, the
        # 10 Hz timer fired, and the core -- still DRIVING on fresh Twists -- sent them.
        self.closing = True
        if self.robot_pub is None or self.core.state != "DRIVING":
            return False
        try:
            _, api_id, parameter = build_request("STOPMOVE")
            r = Request()
            r.header.identity.api_id = api_id
            r.parameter = parameter
            self.robot_pub.publish(r)
            return True
        except Exception:     # context may already be shut down by the signal
            return False


def main(args=None):
    """Spin until SIGINT/SIGTERM, then -- with the context STILL ALIVE -- send the
    exit StopMove, then shut down.

    Integration test K found that the first version never sent it: Humble's rclpy
    installs its own SIGINT handler, which shuts the context down BEFORE any
    `finally:` runs, so the publish raised and was swallowed. Every Ctrl+C reported
    "exit StopMove not sent". So rclpy's handlers are disabled (Humble) or overridden
    (Foxy, where SignalHandlerOptions does not exist -- ⚠️ untested there) and the
    signal only sets a flag.
    """
    stop = {"sig": None}

    def on_signal(signum, _frame):
        stop["sig"] = signum

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    node = None
    rc = 0
    try:
        node = BridgeNode()
        while rclpy.ok() and stop["sig"] is None:
            rclpy.spin_once(node, timeout_sec=0.05)
    except ConfigError:
        rc = 2
    finally:
        if node is not None:
            sent = node.stop_on_exit()
            if sent:
                # Give DDS a moment to put it on the wire before the participant goes.
                # A plain sleep, NOT spin_once: spinning would run callbacks (see closing).
                time.sleep(0.3)
            print("go2_cmd_bridge exiting (signal %s): state %s, exit StopMove %s"
                  % (stop["sig"], node.core.state, "sent" if sent else "not sent"), file=sys.stderr)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
