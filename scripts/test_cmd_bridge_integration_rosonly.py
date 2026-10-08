#!/usr/bin/env python3
"""END-TO-END test of go2_cmd_bridge. Needs ROS 2 + unitree msgs + the built package.
Does NOT need the robot. Plan: gate7_go2_cmdvel_bridge_plan.md §4.3.

test_cmd_bridge_offline.py proves the core's decisions on synthetic time. It cannot
prove the LIVE path: that the dry run really sends nothing over DDS, that a Twist
on /go2/cmd_vel really becomes a Move on /api/sport/request at 10 Hz, that the
watchdog's StopMove really leaves, that a handset press really silences it, and
what a SIGINT or a SIGKILL of the node actually puts on the wire.

So this file plays the robot AND the command source. One fake process publishes
/sportmodestate, /lowstate, /wirelesscontroller and a scripted /go2/cmd_vel,
subscribes to /api/sport/request (answering on /api/sport/response) and to the
bridge's own /go2/cmd_bridge/{requests,status}, and logs everything with time.time().
Each scenario runs the REAL node (`ros2 run go2_cmd_bridge go2_cmd_bridge_node`)
as a subprocess and asserts on what the FAKE saw.

⛔ ISOLATION, as test_sport_probe_integration_rosonly.py: loopback-only Cyclone
(written here), and REFUSES TO RUN if the Go2 cable interface has carrier.
RUN IT WITH THE CABLE UNPLUGGED, with go2_cmd_bridge built and sourced.
"""
import json
import os
import signal
import subprocess
import sys
import tempfile
import time

CABLE_IFACE = os.environ.get("GO2_CABLE_IFACE", "enp46s0")
LOOPBACK_XML = """<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces><NetworkInterface name="lo"/></Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers><Peer address="127.0.0.1"/></Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>20</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
"""

T0 = 7.0          # s after the fake starts: Twists begin (bridge started at ~2 s; discovery)

# scenario -> (armed, twist (vx, wz), twist duration s, total run s)
SCENARIOS = {
    "dry":        (False, (0.3, 0.0), 2.0, 12),
    "standlock":  (True,  (0.3, 0.0), 2.0, 12),
    "idle":       (True,  None,       0.0, 12),
    "drive":      (True,  (0.3, 0.0), 2.5, 13),
    "turn":       (True,  (0.0, -1.0), 1.0, 11),
    "clip":       (True,  (0.9, 3.0), 1.0, 11),
    "handset":    (True,  (0.3, 0.0), 3.0, 13),
    "hot":        (True,  (0.3, 0.0), 3.0, 13),
    "geofence":   (True,  (0.3, 0.0), 6.0, 16),
    "telemetry":  (True,  (0.3, 0.0), 3.0, 13),
    "sigint":     (True,  (0.3, 0.0), 4.0, 13),
    "sigkill":    (True,  (0.3, 0.0), 4.0, 13),
    # Robot only, no Twist source: used by test_cmd_vel_profile_integration_rosonly.py,
    # where cmd_vel_profile.py is the source. Env FAKE_ERROR_CODE / FAKE_HANDSET_AT override.
    "robot_only": (True,  None,       0.0, 30),
}


# ============================================================ the fake robot ===
def fake(scenario, log_path, run_s):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from unitree_go.msg import SportModeState, LowState, WirelessController
    from unitree_api.msg import Request, Response

    armed, twist, dur, _ = SCENARIOS[scenario]
    log = open(log_path, "a")

    def write(kind, **kw):
        kw.update({"kind": kind, "t": time.time()})
        log.write(json.dumps(kw) + "\n")
        log.flush()

    class Fake(Node):
        def __init__(self):
            super().__init__("fake_go2_and_source")
            self.t_start = time.time()
            self.error_code = 1002 if scenario == "standlock" else int(os.environ.get("FAKE_ERROR_CODE", "100"))
            self.handset_at = float(os.environ.get("FAKE_HANDSET_AT", T0 + 1.0 if scenario == "handset" else -1))
            self.v = 0.0
            self.v_until = 0.0
            self.px = 0.0
            self.t_last = time.time()
            self.hip = 30
            self.keys_once = False
            self.p_sport = self.create_publisher(SportModeState, "/sportmodestate", 10)
            self.p_low = self.create_publisher(LowState, "/lowstate", 10)
            self.p_hs = self.create_publisher(WirelessController, "/wirelesscontroller", 10)
            self.p_resp = self.create_publisher(Response, "/api/sport/response", 10)
            self.p_cmd = self.create_publisher(Twist, "/go2/cmd_vel", 10)
            self.create_subscription(Request, "/api/sport/request", self.on_req, 10)
            self.create_subscription(String, "/go2/cmd_bridge/requests", self.on_breq, 10)
            self.create_subscription(String, "/go2/cmd_bridge/status", self.on_status, 10)
            self.create_timer(0.01, self.tick)
            self.create_timer(0.1, self.tick_10hz)

        def rel(self):
            return time.time() - self.t_start

        def on_req(self, m):
            aid = m.header.identity.api_id
            write("robot_req", api_id=aid, parameter=m.parameter)
            if aid == 1008:
                self.v = json.loads(m.parameter)["x"]
                self.v_until = time.time() + 0.3          # a Move does not persist (s12)
            elif aid == 1003:
                self.v, self.v_until = 0.0, 0.0
            elif aid == 1005:
                self.error_code = 2006
            r = Response()
            r.header.identity.api_id = aid
            r.header.status.code = 0
            self.p_resp.publish(r)

        def on_breq(self, m):
            write("bridge_req", **json.loads(m.data))

        def on_status(self, m):
            write("status", status=json.loads(m.data))

        def tick(self):
            now = time.time()
            if now > self.v_until:
                self.v = 0.0
            self.px += self.v * (now - self.t_last)
            self.t_last = now
            r = self.rel()
            if scenario == "hot" and r > T0 + 1.0 and self.hip != 51:
                self.hip = 51
                write("event", what="hip 51")
            if scenario == "telemetry" and r > T0 + 1.0:
                if not getattr(self, "_died", False):
                    self._died = True
                    write("event", what="telemetry stops")
                return
            s = SportModeState()
            s.error_code = self.error_code
            s.body_height = 0.30
            # geofence scenario: the robot tracks the command fully (worst case)
            s.position = [float(self.px), 0.0, 0.0]
            s.velocity = [float(self.v), 0.0, 0.0]
            self.p_sport.publish(s)
            lo = LowState()
            for i in range(12):
                lo.motor_state[i].temperature = 30
            lo.motor_state[6].temperature = self.hip
            self.p_low.publish(lo)

        def tick_10hz(self):
            r = self.rel()
            w = WirelessController()
            if self.handset_at >= 0 and r > self.handset_at and not self.keys_once:
                self.keys_once = True
                w.keys = 288                          # L2+A as measured in s12: ONE message
                write("event", what="handset keys 288")
            self.p_hs.publish(w)
            if twist is not None and T0 <= r < T0 + dur:
                t = Twist()
                t.linear.x, t.angular.z = float(twist[0]), float(twist[1])
                self.p_cmd.publish(t)
                write("twist", vx=twist[0], wz=twist[1])

    rclpy.init()
    n = Fake()
    t_end = time.time() + run_s
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.005)
    n.destroy_node()
    rclpy.shutdown()


# ============================================================ the harness ===
fails = []
checks = 0


def check(label, ok, detail=""):
    global checks
    checks += 1
    print("  %s %s%s" % ("pass" if ok else "FAIL", label, (" -- " + str(detail)[:400]) if detail and not ok else ""))
    if not ok:
        fails.append(label)


def cable_has_carrier():
    try:
        return open("/sys/class/net/%s/carrier" % CABLE_IFACE).read().strip() == "1"
    except OSError:
        return False


def run_scenario(td, scenario):
    armed, twist, dur, run_s = SCENARIOS[scenario]
    log = os.path.join(td, scenario + ".jsonl")
    open(log, "w").close()
    fk = subprocess.Popen([sys.executable, __file__, "--fake", scenario, log, str(run_s)],
                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    t_fake = time.time()
    time.sleep(2.0)
    out = open(os.path.join(td, scenario + ".bridge.txt"), "w")
    br = subprocess.Popen(["ros2", "run", "go2_cmd_bridge", "go2_cmd_bridge_node", "--ros-args",
                           "-p", "armed:=%s" % ("true" if armed else "false")],
                          stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
    t_sig = None
    if scenario in ("sigint", "sigkill"):
        time.sleep(t_fake + T0 + 1.5 - time.time())
        t_sig = time.time()
        # ros2 run wraps the node; signal the whole process group, as Ctrl+C would
        os.killpg(br.pid, signal.SIGINT if scenario == "sigint" else signal.SIGKILL)
    fk.wait(timeout=run_s + 10)
    if br.poll() is None:
        os.killpg(br.pid, signal.SIGINT)
        try:
            br.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(br.pid, signal.SIGKILL)
    out.close()
    rows = [json.loads(l) for l in open(log) if l.strip()]
    return rows, t_sig, open(os.path.join(td, scenario + ".bridge.txt")).read()


def main():
    if cable_has_carrier():
        print("REFUSING: %s has carrier. Unplug the Go2 cable -- this test runs an ARMED bridge "
              "against robot-named topics on domain 0." % CABLE_IFACE)
        return 2
    base = os.environ.get("CMD_BRIDGE_IT_DIR")
    if base:
        os.makedirs(base, exist_ok=True)
    td = tempfile.mkdtemp(prefix="cmd_bridge_it_", dir=base)
    xml = os.path.join(td, "cyclonedds_loopback.xml")
    open(xml, "w").write(LOOPBACK_XML)
    os.environ["CYCLONEDDS_URI"] = "file://" + xml
    os.environ["ROS_DOMAIN_ID"] = "0"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    print("work dir %s; CYCLONEDDS_URI loopback-only; %s no carrier" % (td, CABLE_IFACE))

    R = {}

    def sub(rows, kind):
        return [r for r in rows if r["kind"] == kind]

    def ids(rows):
        return [r["api_id"] for r in sub(rows, "robot_req")]

    def last_status(rows):
        st = sub(rows, "status")
        return st[-1]["status"] if st else {}

    def ev_t(rows, what):
        e = [r["t"] for r in sub(rows, "event") if r["what"] == what]
        return e[0] if e else None

    print("A  dry run: Twists stream -> robot receives NOTHING; requests topic shows what would go")
    rows, _, out = run_scenario(td, "dry")
    br = sub(rows, "bridge_req")
    check("robot received NOTHING", ids(rows) == [], ids(rows))
    check("bridge logged Moves routed 'dry'", br and all(r["routed"] == "dry" for r in br)
          and sum(r["name"] == "MOVE" for r in br) >= 15, [(r["name"], r["routed"]) for r in br][:5])
    check("...and exactly one StopMove after the stream", [r["name"] for r in br].count("STOPMOVE") == 1)
    check("startup line says DRY RUN", "DRY RUN" in out, out[-300:])

    print("B  stand-lock (1002): armed, Twists stream -> nothing sent, WAITING")
    rows, _, out = run_scenario(td, "standlock")
    check("robot received NOTHING", ids(rows) == [], ids(rows))
    check("status WAITING naming 1002", last_status(rows).get("state") == "WAITING"
          and "1002" in last_status(rows).get("reason", ""), last_status(rows))

    print("C  armed, no Twist at all -> IDLE, nothing sent")
    rows, _, out = run_scenario(td, "idle")
    check("robot received NOTHING", ids(rows) == [], ids(rows))
    check("status IDLE", last_status(rows).get("state") == "IDLE", last_status(rows))

    print("D  drive: 0.3 m/s x 2.5 s, then the source simply stops")
    rows, _, out = run_scenario(td, "drive")
    rq = sub(rows, "robot_req")
    moves = [r for r in rq if r["api_id"] == 1008]
    tw = sub(rows, "twist")
    # 25 Twists, plus up to 4 more ticks while the last one is still fresh (cmd_timeout 0.4 s).
    check("25-30 Moves received (25 Twists + the watchdog tail)", 25 <= len(moves) <= 30, len(moves))
    tail = [m for m in moves if tw and m["t"] > tw[-1]["t"] + 0.02]
    R["tail"] = len(tail)
    check("tail after the source stops: at most 4 Moves (0.4 s at 10 Hz)", len(tail) <= 4, len(tail))
    if len(moves) > 2:
        span = moves[-1]["t"] - moves[0]["t"]
        hz = (len(moves) - 1) / span if span else 0
        check("Move rate 10 +/- 0.5 Hz", 9.5 <= hz <= 10.5, hz)
        gaps = [b["t"] - a["t"] for a, b in zip(moves, moves[1:])]
        check("max Move gap < 0.2 s (no tick lost)", max(gaps) < 0.2, max(gaps))
    check("every Move = {x:0.3, y:0, z:0}",
          all(json.loads(r["parameter"]) == {"x": 0.3, "y": 0.0, "z": 0.0} for r in moves))
    stops = [r for r in rq if r["api_id"] == 1003]
    check("exactly ONE StopMove", len(stops) == 1, ids(rows)[-5:])
    if stops and tw:
        dt = stops[0]["t"] - tw[-1]["t"]
        check("StopMove 0.4-0.55 s after the last Twist (watchdog)", 0.4 <= dt <= 0.55, dt)
        check("nothing at all after the StopMove", rq[-1]["api_id"] == 1003, ids(rows)[-3:])
    lat = []
    for m in moves:
        if m["t"] > tw[-1]["t"] + 0.02:
            continue                      # the watchdog tail is not latency
        prior = [t["t"] for t in tw if t["t"] <= m["t"]]
        if prior:
            lat.append(m["t"] - prior[-1])
    if lat:
        lat.sort()
        R["latency"] = (lat[len(lat) // 2], lat[-1])
        check("Twist -> Move latency: median < 110 ms, max < 210 ms (PV7's software floor)",
              lat[len(lat) // 2] < 0.11 and lat[-1] < 0.21, R["latency"])
    check("status back to IDLE", last_status(rows).get("state") == "IDLE", last_status(rows))

    print("E  turn: wz -1.0 -> Moves carry z = -1.0")
    rows, _, out = run_scenario(td, "turn")
    mv = [json.loads(r["parameter"]) for r in sub(rows, "robot_req") if r["api_id"] == 1008]
    check("turn Moves carry z -1.0, x 0", mv and all(p == {"x": 0.0, "y": 0.0, "z": -1.0} for p in mv), mv[:2])

    print("F  clip: Twist (0.9, 3.0) -> Moves clipped to (0.3, 1.0), counted")
    rows, _, out = run_scenario(td, "clip")
    mv = [json.loads(r["parameter"]) for r in sub(rows, "robot_req") if r["api_id"] == 1008]
    check("every Move clipped to x 0.3, z 1.0", mv and all(p == {"x": 0.3, "y": 0.0, "z": 1.0} for p in mv), mv[:2])
    check("status counts the clips", last_status(rows).get("counts", {}).get("clipped", 0) > 0, last_status(rows))

    print("G  handset: ONE keys=288 message mid-walk -> YIELD, zero requests after it")
    rows, _, out = run_scenario(td, "handset")
    tp = ev_t(rows, "handset keys 288")
    after = [r for r in sub(rows, "robot_req") if tp and r["t"] > tp + 0.02]
    check("the press happened during the walk", tp is not None and any(r["t"] < tp for r in sub(rows, "robot_req")))
    check("ZERO requests after the press (+20 ms for one in flight)", tp is not None and after == [],
          [(r["api_id"], round(r["t"] - tp, 3)) for r in after][:5])
    check("status YIELDED", last_status(rows).get("state") == "YIELDED", last_status(rows))

    print("H  hot: rear hip 51 C mid-walk -> StopMove, StandDown ~1 s later, then nothing")
    rows, _, out = run_scenario(td, "hot")
    th = ev_t(rows, "hip 51")
    post = [r for r in sub(rows, "robot_req") if th and r["t"] > th]
    check("after the event: StopMove, StandDown, nothing else", [r["api_id"] for r in post] == [1003, 1005],
          [r["api_id"] for r in post])
    if len(post) == 2:
        check("StandDown 0.9-1.2 s after StopMove", 0.9 <= post[1]["t"] - post[0]["t"] <= 1.2,
              post[1]["t"] - post[0]["t"])
    check("status ABORTED naming the hip", last_status(rows).get("state") == "ABORTED"
          and "hip" in last_status(rows).get("reason", ""), last_status(rows))

    print("I  geofence: robot tracks 0.3 m/s fully -> abort near 1.0 m")
    rows, _, out = run_scenario(td, "geofence")
    st = last_status(rows)
    check("ABORTED on the geofence", st.get("state") == "ABORTED" and "geofence" in st.get("reason", ""), st)
    check("ends StopMove, StandDown", ids(rows)[-2:] == [1003, 1005], ids(rows)[-4:])

    print("J  telemetry dies mid-walk -> abort: StopMove, StandDown")
    rows, _, out = run_scenario(td, "telemetry")
    td_ = ev_t(rows, "telemetry stops")
    post = [r for r in sub(rows, "robot_req") if td_ and r["t"] > td_]
    non_move = [r for r in post if r["api_id"] != 1008]
    check("after telemetry stops: Moves, then exactly StopMove, StandDown, then nothing",
          [r["api_id"] for r in non_move] == [1003, 1005] and post[-2:] == non_move, [r["api_id"] for r in post])
    if non_move:
        dt = non_move[0]["t"] - td_
        check("StopMove 0.5-0.62 s after the silence (stale_s + one tick)", 0.5 <= dt <= 0.62, dt)
    check("reason names the silent topic", "silent" in last_status(rows).get("reason", ""), last_status(rows))

    print("K  SIGINT mid-walk -> best-effort exit StopMove")
    rows, t_sig, out = run_scenario(td, "sigint")
    post = [r["api_id"] for r in sub(rows, "robot_req") if r["t"] > t_sig]
    R["sigint"] = post
    check("a StopMove arrives after SIGINT", 1003 in post, (post, out[-300:]))
    check("no Move after that StopMove", 1003 in post and 1008 not in post[post.index(1003):], post)
    check("node reported it", "exit StopMove sent" in out, out[-300:])

    print("L  SIGKILL mid-walk -> NOTHING more (this is the dead-bridge case PV4 measures on the robot)")
    rows, t_sig, out = run_scenario(td, "sigkill")
    post = [r["api_id"] for r in sub(rows, "robot_req") if r["t"] > t_sig + 0.02]
    pre = [r["api_id"] for r in sub(rows, "robot_req") if r["t"] <= t_sig]
    check("Moves were flowing before the kill", pre.count(1008) >= 10, pre[-3:])
    check("nothing after SIGKILL -- no StopMove can be sent", post == [], post)

    print("\n%d checks, %d failure(s)" % (checks, len(fails)))
    if "latency" in R:
        print("latency Twist->Move: median %.1f ms, max %.1f ms" % (R["latency"][0] * 1e3, R["latency"][1] * 1e3))
    if "tail" in R:
        print("watchdog tail: %d Move(s) after the last Twist" % R["tail"])
    print("logs: " + td)
    if checks < 35:
        print("FAIL: fewer checks ran than written")
        return 1
    print("ALL CMD BRIDGE INTEGRATION TESTS PASS" if not fails else "FAILURES:\n  - " + "\n  - ".join(fails))
    return 1 if fails else 0


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "--fake":
        fake(sys.argv[2], sys.argv[3], float(sys.argv[4]))
        sys.exit(0)
    sys.exit(main())
