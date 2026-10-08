#!/usr/bin/env python3
"""END-TO-END test of go2_sport_probe. Needs ROS 2 + unitree msgs. Does NOT need the robot.

test_sport_probe_offline.py proves the allowlist, clamps, plans and analysis. It
cannot prove the LIVE path: that preflight refuses when it should, that nothing is
sent before the publisher matches, that an abort really sends StopMove + StandDown,
and -- most important -- that after a handset YIELD not one more request leaves.

So this file plays the robot. A fake sport server publishes /sportmodestate,
/lowstate and /wirelesscontroller, subscribes to /api/sport/request, answers on
/api/sport/response, and writes every request it receives to a file. Each scenario
runs the real probe as a subprocess against it, and asserts on what the FAKE saw.

⛔ ISOLATION. The fake publishes robot-named topics on domain 0 -- the probe refuses
any other domain, by design. Isolation is therefore at the transport, twice over:
  1. CYCLONEDDS_URI is forced to a loopback-only config (lo, no multicast, peer
     127.0.0.1), written by this file, so nothing leaves the machine;
  2. the test REFUSES TO RUN if the Go2 cable interface has carrier.
RUN IT WITH THE CABLE UNPLUGGED, inside go2_humble with /ws and /scripts mounted.
"""
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
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


# ============================================================ the fake robot ===
def fake_robot(scenario, log_path, run_s):
    import math
    import rclpy
    from rclpy.node import Node
    from unitree_go.msg import SportModeState, LowState, WirelessController
    from unitree_api.msg import Request, Response

    class Fake(Node):
        def __init__(self):
            super().__init__("fake_go2")
            self.bh = 0.30 if scenario == "starts_standing" else 0.0715
            self.target = self.bh
            self.v = 0.0
            self.v_until = 0.0
            self.persist = scenario == "persist"
            self.hot = False
            self.keys = 0
            self.t_stand = None
            self.px = 0.0                             # odometry: integrates self.v
            self.t_last = time.time()
            self.p_sport = self.create_publisher(SportModeState, "/sportmodestate", 10)
            self.p_low = self.create_publisher(LowState, "/lowstate", 10)
            self.p_hs = self.create_publisher(WirelessController, "/wirelesscontroller", 10)
            self.p_resp = self.create_publisher(Response, "/api/sport/response", 10)
            if scenario != "no_server":
                self.create_subscription(Request, "/api/sport/request", self.on_req, 10)
            self.log = open(log_path, "a")
            self.create_timer(0.01, self.tick)        # 100 Hz -- enough for a 0.5 s stale guard
            self.create_timer(0.1, self.tick_hs)

        def on_req(self, m):
            aid = m.header.identity.api_id
            self.log.write(json.dumps({"t": time.time(), "api_id": aid,
                                       "parameter": m.parameter}) + "\n")
            self.log.flush()
            if aid == 1004:
                self.target = 0.30; self.t_stand = time.time()
            elif aid == 1005:
                self.target = 0.0715; self.v = 0.0
            elif aid == 1003:
                self.v = 0.0; self.v_until = 0.0
            elif aid == 1008:
                self.v = json.loads(m.parameter)["x"]
                self.v_until = float("inf") if self.persist else time.time() + 0.6
            r = Response()
            r.header.identity.api_id = aid
            r.header.status.code = 0
            r.data = '{"fake":true}'
            self.p_resp.publish(r)

        def tick(self):
            now = time.time()
            self.bh += (self.target - self.bh) * 0.1
            if now > self.v_until:
                self.v = 0.0
            # The analyser scores PM3 on POSITION (8 Oct), so the fake must move in
            # position, not only report a velocity -- as the real odometry does.
            self.px += self.v * (now - self.t_last)
            self.t_last = now
            if self.t_stand and now - self.t_stand > 3.0:
                if scenario == "hot":
                    self.hot = True
                if scenario == "handset":
                    self.keys = 1
            s = SportModeState()
            s.body_height = float(self.bh)
            s.velocity = [float(self.v), 0.0, 0.0]
            s.position = [float(self.px), 0.0, 0.0]
            s.error_code = 1001
            self.p_sport.publish(s)
            lo = LowState()
            for i in range(12):
                lo.motor_state[i].temperature = 26
                lo.motor_state[i].q = 0.5
            if self.hot:
                lo.motor_state[6].temperature = 51
            self.p_low.publish(lo)

        def tick_hs(self):
            w = WirelessController()
            w.keys = self.keys
            self.p_hs.publish(w)

    rclpy.init()
    n = Fake()
    t_end = time.time() + run_s
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.01)
    n.destroy_node()
    rclpy.shutdown()


# ============================================================ the harness ===
fails = []
checks = 0


def check(label, ok, detail=""):
    global checks
    checks += 1
    print(f"  {'pass' if ok else 'FAIL'} {label}" + (f" -- {detail}" if detail and not ok else ""))
    if not ok:
        fails.append(label)


def cable_has_carrier():
    try:
        return open(f"/sys/class/net/{CABLE_IFACE}/carrier").read().strip() == "1"
    except OSError:
        return False          # interface absent or down: no carrier


def run_scenario(td, scenario, stage, run_s):
    log = os.path.join(td, f"{scenario}.requests.jsonl")
    open(log, "w").close()
    out = os.path.join(td, scenario)
    os.makedirs(out)
    env = dict(os.environ)
    fake = subprocess.Popen([sys.executable, __file__, "--fake", scenario, log, str(run_s + 6)],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(3.0)                                    # discovery
    p = subprocess.run([sys.executable, os.path.join(HERE, "go2_sport_probe.py"), stage,
                        "--armed", "--out", out], env=env, capture_output=True, text=True,
                       timeout=run_s + 30)
    fake.terminate()
    try:
        fake.wait(timeout=5)
    except subprocess.TimeoutExpired:
        fake.kill()
    reqs = [json.loads(l) for l in open(log) if l.strip()]
    runs = [os.path.join(out, d) for d in os.listdir(out)]
    events = []
    if runs:
        import csv
        with open(os.path.join(runs[0], "events.csv")) as fh:
            events = list(csv.DictReader(fh))
    return p, reqs, events, (runs[0] if runs else None)


def main():
    if cable_has_carrier():
        print(f"REFUSING: {CABLE_IFACE} has carrier. Unplug the Go2 cable -- this test publishes "
              f"robot-named topics on domain 0.")
        return 2
    td = tempfile.mkdtemp(prefix="sport_probe_it_")
    xml = os.path.join(td, "cyclonedds_loopback.xml")
    open(xml, "w").write(LOOPBACK_XML)
    os.environ["CYCLONEDDS_URI"] = "file://" + xml
    os.environ["ROS_DOMAIN_ID"] = "0"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    print(f"work dir {td}; CYCLONEDDS_URI loopback-only; {CABLE_IFACE} no carrier")

    ids = lambda reqs: [r["api_id"] for r in reqs]

    print("A  query, lying -> one 2055, answered, joints still")
    p, reqs, ev, rd = run_scenario(td, "query", "query", 12)
    check("rc 0", p.returncode == 0, p.stdout[-600:] + p.stderr[-600:])
    check("fake received exactly [2055]", ids(reqs) == [2055], str(ids(reqs)))
    rep = open(os.path.join(rd, "report.txt")).read() if rd else ""
    check("report matched a response, code 0", "AUTORECOVERY_GET  code=    0" in rep, rep)
    check("report PM1 PASS", "PM1" in rep and "PASS" in rep.split("PM1")[1].split("\n")[0], rep)

    print("B  refuse: query while standing -> nothing sent")
    p, reqs, ev, rd = run_scenario(td, "starts_standing", "query", 8)
    check("rc 2", p.returncode == 2, p.stdout[-600:])
    check("fake received NOTHING", reqs == [], str(ids(reqs)))

    print("C  refuse: no sport server subscribed -> publisher unmatched -> nothing sent")
    p, reqs, ev, rd = run_scenario(td, "no_server", "query", 8)
    check("rc 2", p.returncode == 2, p.stdout[-600:])
    check("events record the unmatched refusal",
          any("matched no subscriber" in e["detail"] for e in ev), str([e["detail"] for e in ev]))

    print("D  move_once, non-persistent Move")
    p, reqs, ev, rd = run_scenario(td, "move", "move_once", 30)
    check("rc 0", p.returncode == 0, p.stdout[-800:] + p.stderr[-800:])
    check("request sequence", ids(reqs) == [1004, 1002, 1008, 1003, 1003, 1005], str(ids(reqs)))
    check("the one Move carried the baked parameter",
          [json.loads(r["parameter"]) for r in reqs if r["api_id"] == 1008] == [{"x": 0.1, "y": 0.0, "z": 0.0}])
    rep = open(os.path.join(rd, "report.txt")).read() if rd else ""
    check("report: does NOT persist", "does NOT persist" in rep, rep)

    print("E  move_once, PERSISTENT Move -> reported as persisting, still stopped by the script")
    p, reqs, ev, rd = run_scenario(td, "persist", "move_once", 30)
    check("rc 0", p.returncode == 0, p.stdout[-800:])
    rep = open(os.path.join(rd, "report.txt")).read() if rd else ""
    check("report: PERSISTS", "PERSISTS" in rep, rep)
    check("StopMove was sent after the Move", ids(reqs)[ids(reqs).index(1008) + 1] == 1003
          if 1008 in ids(reqs) else False, str(ids(reqs)))

    print("F  thermal: rear hip 51 C while standing -> abort, StopMove + StandDown")
    p, reqs, ev, rd = run_scenario(td, "hot", "stand", 25)
    check("rc 1 (abort)", p.returncode == 1, p.stdout[-800:])
    check("abort names the rear hip", any(e["kind"] == "abort" and "rear hip" in e["detail"] for e in ev))
    check("last two requests are StopMove, StandDown", ids(reqs)[-2:] == [1003, 1005], str(ids(reqs)))

    print("G  handset activity while standing -> YIELD, nothing more sent")
    p, reqs, ev, rd = run_scenario(td, "handset", "stand", 25)
    check("rc 3 (yield)", p.returncode == 3, p.stdout[-800:])
    t_yield = [float(e["t"]) for e in ev if e["kind"] == "yield"]
    check("a yield event exists", bool(t_yield))
    if t_yield:
        after = [r for r in reqs if r["t"] > t_yield[0]]
        check("ZERO requests received after the yield", after == [], str(after))
    check("only StandUp was ever sent", ids(reqs) == [1004], str(ids(reqs)))

    print(f"\n{checks} checks, {len(fails)} failure(s)")
    if checks < 20:
        print("FAIL: fewer checks ran than written")
        return 1
    print("ALL SPORT PROBE INTEGRATION TESTS PASS" if not fails else "FAILURES:\n  - " + "\n  - ".join(fails))
    return 1 if fails else 0


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "--fake":
        fake_robot(sys.argv[2], sys.argv[3], float(sys.argv[4]))
        sys.exit(0)
    sys.exit(main())
