#!/usr/bin/env python3
"""END-TO-END test of go2_action_bridge. Needs ROS 2 + unitree msgs + the built package.
Does NOT need the robot. Plan: Claude_Research friday_dance_demo_plan.md §3.2 stage S0.

test_action_bridge_offline.py proves the core's decisions on synthetic time. It cannot
prove the LIVE path: that the dry run really sends nothing over DDS, that "dance1" on
/go2/action really becomes ONE 1022 on /api/sport/request, that a 10 Hz burst still
gives one, that the node's own /api/sport/request subscription does not satisfy the
"matched a subscriber" check, and that a refusal really refuses.

So this file plays the robot AND the trigger source. One fake process publishes
/sportmodestate, /lowstate (all 12 motors mode 1), /wirelesscontroller and the scripted
/go2/action, subscribes to /api/sport/request (answering on /api/sport/response, and
"dancing" -- error_code 1006 for FAKE_DANCE_S, then 1013, as measured 27 Oct) and to
/go2/action_bridge/{requests,status}. Each scenario runs the REAL node
(`ros2 run go2_action_bridge go2_action_bridge_node`) and asserts on what the FAKE saw.

⛔ ISOLATION, as test_cmd_bridge_integration_rosonly.py: loopback-only Cyclone (written
here), and REFUSES TO RUN if the Go2 cable interface has carrier.
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
if os.environ.get("ROS_DISTRO") == "foxy":
    LOOPBACK_XML = LOOPBACK_XML.replace('<Interfaces><NetworkInterface name="lo"/></Interfaces>',
                                        '<NetworkInterfaceAddress>lo</NetworkInterfaceAddress>')

T0 = 7.0            # s after the fake starts: the trigger (bridge started at ~2 s; discovery)
FAKE_DANCE_S = 3.0  # the fake's "dance": 1006 for this long, then 1013
BRIDGE_PARAMS = ["lockout_s:=5.0"]     # the floor, so a scenario fits in ~18 s

# scenario -> (armed, run s). Behaviour per scenario is in the fake.
SCENARIOS = {
    "dry":       (False, 14),
    "dance":     (True, 20),
    "burst":     (True, 20),
    "standlock": (True, 12),
    "handset":   (True, 12),
    "foreign":   (True, 12),
    "refused":   (True, 13),
    "nostart":   (True, 13),
    "cmdbridge": (True, 12),
    "unknown":   (True, 12),
    "sigint":    (True, 13),
    "lateref":   (True, 14),
}


# ============================================================ the fake robot ===
def fake(scenario, log_path, run_s):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from unitree_go.msg import SportModeState, LowState, WirelessController
    from unitree_api.msg import Request, Response

    log = open(log_path, "a")

    def write(kind, **kw):
        kw.update({"kind": kind, "t": time.time()})
        log.write(json.dumps(kw) + "\n")
        log.flush()

    class Fake(Node):
        def __init__(self):
            super().__init__("fake_go2_and_hexapod")
            self.t_start = time.time()
            self.error_code = 1002 if scenario == "standlock" else 100
            self.dance_until = None
            self.resp_due = None              # (time, code): the end-of-action response
            self.done = set()
            self.p_sport = self.create_publisher(SportModeState, "/sportmodestate", 10)
            self.p_low = self.create_publisher(LowState, "/lowstate", 10)
            self.p_hs = self.create_publisher(WirelessController, "/wirelesscontroller", 10)
            self.p_resp = self.create_publisher(Response, "/api/sport/response", 10)
            self.p_act = self.create_publisher(String, "/go2/action", 10)
            # Plays the handset's on-robot relay (27 Oct): a Move from someone who is not the bridge.
            self.p_foreign = self.create_publisher(Request, "/api/sport/request", 10) if scenario == "foreign" else None
            self.p_cmdb = (self.create_publisher(String, "/go2/cmd_bridge/status", 10)
                           if scenario == "cmdbridge" else None)
            if scenario == "ext":   # count what ros1_bridge delivers (a bidirectional bridge could echo it)
                self.create_subscription(String, "/go2/action", lambda m: write("trigger_seen", data=m.data), 10)
            self.create_subscription(Request, "/api/sport/request", self.on_req, 10)
            self.create_subscription(String, "/go2/action_bridge/requests", self.on_breq, 10)
            self.create_subscription(String, "/go2/action_bridge/status", self.on_status, 10)
            self.create_timer(0.01, self.tick)
            self.create_timer(0.1, self.tick_10hz)

        def rel(self):
            return time.time() - self.t_start

        def on_req(self, m):
            aid = m.header.identity.api_id
            write("robot_req", api_id=aid, parameter=m.parameter)
            if aid != 1022:
                return                      # our own foreign Move, seen on our own subscription
            if scenario in ("refused", "nostart"):
                # No dance: answered at once. ⚠️ Unmeasured (no refusal or no-start has been seen on the robot).
                self.respond(3104 if scenario == "refused" else 0)
                return
            self.error_code = 1006
            self.dance_until = time.time() + FAKE_DANCE_S
            # As measured 27 Oct S3 (n = 1): the response comes at the END of the dance, 12 ms before
            # 1006 -> 1013. "lateref": a non-zero code just AFTER 1013 (the 12 ms race flipped).
            if scenario == "lateref":
                self.resp_due = (self.dance_until + 1.5, 3104)   # > 2 status samples of LOCKOUT
            else:
                self.resp_due = (self.dance_until - 0.012, 0)
            write("event", what="dance starts")

        def respond(self, code):
            r = Response()
            r.header.identity.api_id = 1022
            r.header.status.code = code
            self.p_resp.publish(r)
            write("event", what="response %d" % code)

        def on_breq(self, m):
            write("bridge_req", **json.loads(m.data))

        def on_status(self, m):
            write("status", status=json.loads(m.data))

        def tick(self):
            if self.resp_due is not None and time.time() >= self.resp_due[0]:
                self.respond(self.resp_due[1])
                self.resp_due = None
            if self.dance_until is not None and time.time() > self.dance_until:
                self.dance_until = None
                self.error_code = 1013
                write("event", what="dance ends")
            s = SportModeState()
            s.error_code = self.error_code
            self.p_sport.publish(s)
            lo = LowState()
            for i in range(12):
                lo.motor_state[i].mode = 1
                lo.motor_state[i].temperature = 30
            self.p_low.publish(lo)

        def once(self, key):
            if key in self.done:
                return False
            self.done.add(key)
            return True

        def tick_10hz(self):
            r = self.rel()
            w = WirelessController()
            if scenario == "handset" and r > T0 - 0.5 and self.once("keys"):
                w.keys = 288
                write("event", what="handset keys 288")
            self.p_hs.publish(w)
            if self.p_foreign is not None and r > T0 - 1.0 and self.once("foreign"):
                q = Request()
                q.header.identity.api_id = 1008
                q.parameter = json.dumps({"x": 0.3, "y": 0.0, "z": 0.0})
                self.p_foreign.publish(q)
                write("event", what="foreign move")
            if self.p_cmdb is not None:
                self.p_cmdb.publish(String(data="{}"))
            word = "dance2" if scenario == "unknown" else "dance1"
            if scenario == "ext":
                return              # dance_loopback_test.sh: the trigger comes from ROS 1, through ros1_bridge
            if scenario == "burst":
                if T0 <= r < T0 + 8.0:
                    self.p_act.publish(String(data=word))
                    write("trigger", data=word)
            elif r >= T0 and self.once("trigger"):
                self.p_act.publish(String(data=word))
                write("trigger", data=word)

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
    armed, run_s = SCENARIOS[scenario]
    log = os.path.join(td, scenario + ".jsonl")
    open(log, "w").close()
    fk = subprocess.Popen([sys.executable, __file__, "--fake", scenario, log, str(run_s)],
                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    t_fake = time.time()
    time.sleep(2.0)
    out = open(os.path.join(td, scenario + ".bridge.txt"), "w")
    args = ["ros2", "run", "go2_action_bridge", "go2_action_bridge_node", "--ros-args",
            "-p", "armed:=%s" % ("true" if armed else "false")]
    for p in BRIDGE_PARAMS:
        args += ["-p", p]
    br = subprocess.Popen(args, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
    t_sig = None
    if scenario == "sigint":
        time.sleep(t_fake + T0 + 1.0 - time.time())
        t_sig = time.time()
        os.killpg(br.pid, signal.SIGINT)
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
    base = os.environ.get("ACTION_BRIDGE_IT_DIR")
    if base:
        os.makedirs(base, exist_ok=True)
    td = tempfile.mkdtemp(prefix="action_bridge_it_", dir=base)
    xml = os.path.join(td, "cyclonedds_loopback.xml")
    open(xml, "w").write(LOOPBACK_XML)
    os.environ["CYCLONEDDS_URI"] = "file://" + xml
    os.environ["ROS_DOMAIN_ID"] = "0"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    print("work dir %s; CYCLONEDDS_URI loopback-only; %s no carrier" % (td, CABLE_IFACE))

    def sub(rows, kind):
        return [r for r in rows if r["kind"] == kind]

    def dances(rows):
        return [r for r in sub(rows, "robot_req") if r["api_id"] == 1022]

    def states(rows):
        seq = []
        for r in sub(rows, "status"):
            s = r["status"]["state"]
            if not seq or seq[-1] != s:
                seq.append(s)
        return seq

    def last_status(rows):
        st = sub(rows, "status")
        return st[-1]["status"] if st else {}

    print("A  dry run: trigger in balance stand -> robot receives NOTHING; requests topic shows ONE would-send")
    rows, _, out = run_scenario(td, "dry")
    br = sub(rows, "bridge_req")
    check("robot received NOTHING", sub(rows, "robot_req") == [], sub(rows, "robot_req"))
    check("exactly one bridge_req, DANCE1 1022, routed dry",
          [(r["name"], r["api_id"], r["routed"]) for r in br] == [("DANCE1", 1022, "dry")], br)
    check("status READY -> SENT -> NO_START (no 1006 in a dry run)", states(rows)[-3:] == ["READY", "SENT", "NO_START"],
          states(rows))
    check("startup line says DRY RUN", "DRY RUN" in out, out[-300:])

    print("B  armed dance: ONE 1022, the fake dances 3 s, then LOCKOUT and SPENT")
    rows, _, out = run_scenario(td, "dance")
    tr = sub(rows, "trigger")
    d = dances(rows)
    check("exactly ONE 1022 received", len(d) == 1, d)
    check("nothing but that one request reached the robot", len(sub(rows, "robot_req")) == 1, sub(rows, "robot_req"))
    if d and tr:
        lat = d[0]["t"] - tr[0]["t"]
        print("  trigger -> 1022 latency %.1f ms" % (lat * 1e3))
        check("trigger -> 1022 latency < 50 ms (sent from the callback, not the tick)", lat < 0.05, lat)
    # SENT is optional HERE: status is sampled at 2 Hz and the fake answers 1006 within ~1 ms, so SENT
    # (< 0.1 s) is usually never sampled (27 Oct, first run: ['READY', 'DANCING', ...]). The send itself is
    # proven by the 1022 above; the SENT -> NO_START path by scenarios A and H.
    seq = [x for x in states(rows) if x != "SENT"]
    check("status READY -> (SENT) -> DANCING -> LOCKOUT -> SPENT",
          seq[-4:] == ["READY", "DANCING", "LOCKOUT", "SPENT"], states(rows))
    br = sub(rows, "bridge_req")
    check("bridge_req routed 'robot', subscribers name the fake and NOT the bridge itself",
          br and br[0]["routed"] == "robot" and br[0]["subscribers"] == ["fake_go2_and_hexapod"], br)
    check("last_response (1022, 0)", last_status(rows).get("last_response") == [1022, 0], last_status(rows))
    check("own_echo counted 1, foreign 0", last_status(rows).get("counts", {}).get("own_echo") == 1
          and last_status(rows)["counts"]["foreign_requests"] == 0, last_status(rows).get("counts"))
    check("startup line says ARMED", "ARMED" in out, out[-300:])

    print("C  burst: \"dance1\" at 10 Hz for 8 s (rostopic pub -r 10) -> ONE 1022")
    rows, _, out = run_scenario(td, "burst")
    check("~80 triggers published", len(sub(rows, "trigger")) >= 70, len(sub(rows, "trigger")))
    check("exactly ONE 1022 received", len(dances(rows)) == 1, dances(rows))
    c = last_status(rows).get("counts", {})
    check("bridge counted the rest as ignored_not_ready", c.get("ignored_not_ready", 0) >= 60, c)

    print("D  stand-lock (1002): trigger -> nothing")
    rows, _, out = run_scenario(td, "standlock")
    check("robot received NOTHING", sub(rows, "robot_req") == [])
    check("status WAITING naming 1002", last_status(rows).get("state") == "WAITING"
          and "1002" in last_status(rows).get("reason", ""), last_status(rows))
    check("node logged the ignored trigger", "ignored" in out and "1002" in out, out[-300:])

    print("E  handset keys 0.5 s before the trigger -> refused")
    rows, _, out = run_scenario(td, "handset")
    check("robot received NOTHING", sub(rows, "robot_req") == [])
    check("last_ignored names the handset", "handset" in (last_status(rows).get("last_ignored") or ""), last_status(rows))

    print("F  a foreign Move 1 s before the trigger -> refused")
    rows, _, out = run_scenario(td, "foreign")
    check("no 1022 received", dances(rows) == [], dances(rows))
    check("last_ignored names 1008", "1008" in (last_status(rows).get("last_ignored") or ""), last_status(rows))
    check("bridge counted 1 foreign request", last_status(rows).get("counts", {}).get("foreign_requests") == 1,
          last_status(rows).get("counts"))

    print("G  sport server answers 3104 -> REFUSED (latched)")
    rows, _, out = run_scenario(td, "refused")
    check("one 1022 sent", len(dances(rows)) == 1)
    check("status REFUSED naming 3104", last_status(rows).get("state") == "REFUSED"
          and "3104" in last_status(rows).get("reason", ""), last_status(rows))

    print("H  answered 0 but never dances -> NO_START (latched)")
    rows, _, out = run_scenario(td, "nostart")
    check("one 1022 sent", len(dances(rows)) == 1)
    check("status NO_START", last_status(rows).get("state") == "NO_START", last_status(rows))

    print("I  go2_cmd_bridge on the graph -> refused")
    rows, _, out = run_scenario(td, "cmdbridge")
    check("robot received NOTHING", sub(rows, "robot_req") == [])
    check("status WAITING naming go2_cmd_bridge", "cmd_bridge" in last_status(rows).get("reason", ""), last_status(rows))

    print("J  \"dance2\" -> unknown, nothing")
    rows, _, out = run_scenario(td, "unknown")
    check("robot received NOTHING", sub(rows, "robot_req") == [])
    check("counted ignored_unknown, still READY", last_status(rows).get("counts", {}).get("ignored_unknown") == 1
          and last_status(rows).get("state") == "READY", last_status(rows))

    print("K  SIGINT mid-dance -> nothing more sent, clean exit")
    rows, t_sig, out = run_scenario(td, "sigint")
    check("one 1022 before the signal", len([r for r in dances(rows) if r["t"] < t_sig]) == 1, dances(rows))
    check("NOTHING after the signal", [r for r in sub(rows, "robot_req") if r["t"] > t_sig] == [])
    check("node reported its exit", "go2_action_bridge exiting" in out, out[-300:])

    print("L  late refusal: 3104 arrives 1.5 s AFTER 1013 (the 12 ms race flipped) -> REFUSED, not SPENT")
    rows, _, out = run_scenario(td, "lateref")
    ev = [r["what"] for r in sub(rows, "event")]
    check("fake order: dance ends, THEN response 3104", "dance ends" in ev and "response 3104" in ev
          and ev.index("dance ends") < ev.index("response 3104"), ev)
    check("one 1022 sent", len(dances(rows)) == 1, dances(rows))
    check("status LOCKOUT seen, then REFUSED naming 3104", "LOCKOUT" in states(rows)
          and last_status(rows).get("state") == "REFUSED" and "3104" in last_status(rows).get("reason", ""),
          (states(rows), last_status(rows)))

    print("\n%d checks, %d failure(s)" % (checks, len(fails)))
    print("logs: " + td)
    if checks < 37:
        print("FAIL: fewer checks ran than written")
        return 1
    print("ALL ACTION BRIDGE INTEGRATION TESTS PASS" if not fails else "FAILURES:\n  - " + "\n  - ".join(fails))
    return 1 if fails else 0


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "--fake":
        fake(sys.argv[2], sys.argv[3], float(sys.argv[4]))
        sys.exit(0)
    sys.exit(main())
