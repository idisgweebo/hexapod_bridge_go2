#!/usr/bin/env python3
"""go2_action_bridge_node: a word on /go2/action -> ONE preset request on the Go2.

⛔ The THIRD file in this project that can publish to the Go2 (after
scripts/go2_sport_probe.py and go2_cmd_bridge). Plan: Claude_Research
friday_dance_demo_plan.md §3.1-3.2.

THIS FILE DECIDES NOTHING
-------------------------
Every decision -- allowlist, preconditions, one-shot, lockout, latches -- is in
action_core.py, which is pure and tested offline. This node only:
  * stamps each arriving message with time.time() and hands it to the core,
  * routes what core.on_trigger() returns: to /api/sport/request when armed, and
    ALWAYS to /go2/action_bridge/requests as a log line,
  * calls core.tick() at 10 Hz, and tells the core whether go2_cmd_bridge is on the
    graph (a publisher on /go2/cmd_bridge/status),
  * publishes the core's status at 2 Hz on /go2/action_bridge/status.

DRY RUN BY DEFAULT
------------------
With armed:=false (the default) the robot publisher is NEVER CREATED. The requests
topic carries exactly what an armed bridge would have sent.

ARMED
-----
The robot publisher must match a subscriber OTHER THAN THIS NODE before anything is
sent: this node subscribes to /api/sport/request itself (to see foreign requests), so
a plain get_subscription_count() would count us. ⚠️ Any other listener (e.g.
go2_dance_capture.py) also counts; the names of the matched subscribers are logged
with every send so the record shows which ones were there.

Python 3.8-compatible (Foxy target).
"""
import json
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from unitree_go.msg import SportModeState, LowState, WirelessController
from unitree_api.msg import Request, Response

from go2_action_bridge.action_core import ActionCore, DEFAULTS, ConfigError

RATE_HZ = 10.0
STATUS_EVERY = 5        # ticks -> 2 Hz
RR_HIP, RL_HIP = 6, 9   # motor indices, as go2_thermal_watch / session 6
CMD_BRIDGE_STATUS = "/go2/cmd_bridge/status"


class ActionNode(Node):

    def __init__(self):
        super().__init__("go2_action_bridge_node")
        self.armed = bool(self.declare_parameter("armed", False).value)
        overrides = {}
        for k, v in DEFAULTS.items():
            # Declared with the default's own type: max_sends is an int, so YAML `max_sends: 1`
            # matches (a double declaration would refuse it).
            overrides[k] = self.declare_parameter(k, v).value
        try:
            self.core = ActionCore(overrides)
        except ConfigError as e:
            self.get_logger().fatal("refusing to start: %s" % e)
            raise

        self.robot_pub = None
        if self.armed:
            # ⛔ The single robot publisher in this package. Literal topic; audited by
            # scripts/test_action_bridge_offline.py.
            self.robot_pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.req_pub = self.create_publisher(String, "/go2/action_bridge/requests", 10)
        self.status_pub = self.create_publisher(String, "/go2/action_bridge/status", 10)

        self.create_subscription(String, "/go2/action", self.on_action, 10)
        self.create_subscription(SportModeState, "/sportmodestate", self.on_sport, 10)
        self.create_subscription(LowState, "/lowstate", self.on_low, 10)
        self.create_subscription(WirelessController, "/wirelesscontroller", self.on_hs, 10)
        self.create_subscription(Request, "/api/sport/request", self.on_robot_req, 10)
        self.create_subscription(Response, "/api/sport/response", self.on_resp, 10)

        self.n_tick = 0
        self.dropped_unmatched = 0
        self.last_state = None
        self.create_timer(1.0 / RATE_HZ, self.on_tick)
        self.get_logger().info(
            "go2_action_bridge up: %s, params %s" % ("⛔ ARMED" if self.armed else "DRY RUN (robot publisher not created)",
                                                     json.dumps(self.core.p, sort_keys=True)))

    # ------------------------------------------------------------- inputs ---
    def on_action(self, m):
        now = time.time()
        reqs = self.core.on_trigger(now, m.data)
        for req in reqs:
            self.route(now, req)
        if not reqs:
            # Throttled: a hexapod `rostopic pub -r 10` would otherwise flood the log.
            self.get_logger().info("trigger %r ignored: %s" % (m.data[:40], self.core.last_ignored),
                                   throttle_duration_sec=1.0)

    def on_sport(self, m):
        self.core.on_sport(time.time(), m.error_code)

    def on_low(self, m):
        ok = sum(1 for i in range(12) if m.motor_state[i].mode == 1)
        self.core.on_low(time.time(), m.motor_state[RR_HIP].temperature,
                         m.motor_state[RL_HIP].temperature, ok)

    def on_hs(self, m):
        self.core.on_handset(time.time(), m.lx, m.ly, m.rx, m.ry, m.keys)

    def on_robot_req(self, m):
        self.core.on_robot_request(time.time(), m.header.identity.api_id)

    def on_resp(self, m):
        self.core.on_response(time.time(), m.header.identity.api_id, m.header.status.code)

    # --------------------------------------------------------------- tick ---
    def on_tick(self):
        now = time.time()
        self.core.set_cmd_bridge_present(self.count_publishers(CMD_BRIDGE_STATUS) > 0)
        self.core.tick(now)
        st = self.core.status()
        if st["state"] != self.last_state:
            self.get_logger().info("state %s -> %s (%s)" % (self.last_state, st["state"], st["reason"]))
            self.last_state = st["state"]
        if st.get("warning"):
            self.get_logger().warning(st["warning"])
        self.n_tick += 1
        if self.n_tick % STATUS_EVERY == 0:
            st["armed"] = self.armed
            st["dropped_unmatched"] = self.dropped_unmatched
            self.status_pub.publish(String(data=json.dumps(st)))

    def other_subscribers(self):
        me = self.get_name()
        return sorted(i.node_name for i in self.get_subscriptions_info_by_topic("/api/sport/request")
                      if i.node_name != me)

    def route(self, now, req):
        name, api_id, parameter = req
        routed = "dry"
        subs = None
        if self.robot_pub is not None:
            subs = self.other_subscribers()
            if not subs:
                self.dropped_unmatched += 1
                routed = "dropped_unmatched"
            else:
                r = Request()
                r.header.identity.api_id = api_id
                r.parameter = parameter
                self.robot_pub.publish(r)
                routed = "robot"
        self.req_pub.publish(String(data=json.dumps(
            {"t": round(now, 6), "name": name, "api_id": api_id, "parameter": parameter,
             "routed": routed, "subscribers": subs})))
        self.get_logger().info("%s %s api_id %d (subscribers: %s)" % (routed, name, api_id, subs))


def main(args=None):
    """Spin until SIGINT/SIGTERM. Nothing is sent on exit: there is nothing to stop --
    a dance in progress is the handset's, not ours (no abort path, by design)."""
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
        node = ActionNode()
        while rclpy.ok() and stop["sig"] is None:
            rclpy.spin_once(node, timeout_sec=0.05)
    except ConfigError:
        rc = 2
    finally:
        if node is not None:
            print("go2_action_bridge exiting (signal %s): state %s, sends %d"
                  % (stop["sig"], node.core.state, node.core.n_sends), file=sys.stderr)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
