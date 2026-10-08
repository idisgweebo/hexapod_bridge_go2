#!/usr/bin/env python3
"""END-TO-END: cmd_vel_profile.py -> the REAL go2_cmd_bridge_node -> a fake Go2.
Needs ROS 2 + unitree msgs + go2_cmd_bridge built and sourced. NOT the robot.

This is the robot-stage chain exactly as it will run in session 13, with the robot
replaced by the fake from test_cmd_bridge_integration_rosonly.py ("robot_only").

  P1  walk_03, armed: profile streams, bridge Moves, watchdog StopMove; report has PV3
  P2  preflight: profile says --dry, bridge is armed -> refuses, robot gets NOTHING
  P3  stage-0 rehearsal: robot LYING (1001), bridge dry -> profile accepts WAITING,
      streams, bridge would send nothing
  P4  walk_03_kill: the profile SIGKILLs the bridge -> nothing after; report has PV4
  P5  handset mid-walk: bridge YIELDS -> profile stops streaming
  P6  Ctrl+C on the profile mid-stream -> zero Twists -> bridge sends ONE StopMove

⛔ Same isolation as the bridge test: loopback-only Cyclone, refuses on carrier.
"""
import csv
import glob
import json
import os
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_cmd_bridge_integration_rosonly as bt     # LOOPBACK_XML, cable check, the fake

fails = []
checks = 0


def check(label, ok, detail=""):
    global checks
    checks += 1
    print("  %s %s%s" % ("pass" if ok else "FAIL", label, (" -- " + str(detail)[:500]) if detail and not ok else ""))
    if not ok:
        fails.append(label)


def run(td, name, profile, prof_args, armed=True, env_extra=None, sigint_at=None, fake_s=25):
    env = dict(os.environ)
    env.update(env_extra or {})
    log = os.path.join(td, name + ".fake.jsonl")
    open(log, "w").close()
    fk = subprocess.Popen([sys.executable, os.path.join(HERE, "test_cmd_bridge_integration_rosonly.py"),
                           "--fake", "robot_only", log, str(fake_s)], env=env,
                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    time.sleep(2.0)
    bout = open(os.path.join(td, name + ".bridge.txt"), "w")
    br = subprocess.Popen(["ros2", "run", "go2_cmd_bridge", "go2_cmd_bridge_node", "--ros-args",
                           "-p", "armed:=%s" % ("true" if armed else "false")], env=env,
                          stdout=bout, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(2.5)
    out = os.path.join(td, name)
    os.makedirs(out)
    pout = open(os.path.join(td, name + ".profile.txt"), "w")
    t_prof = time.time()
    pr = subprocess.Popen([sys.executable, os.path.join(HERE, "cmd_vel_profile.py"), profile, "--out", out]
                          + prof_args, env=env, stdout=pout, stderr=subprocess.STDOUT)
    t_sig = None
    if sigint_at is not None:
        # wait for the first Twist, then interrupt sigint_at s later
        t_end = time.time() + 15
        while time.time() < t_end:
            # Parse the CSV and look at `kind`. A substring search matched the bridge's
            # status JSON ("counts": {"twist": 0 ...) in the preflight row and fired
            # the signal before anything streamed (first run, 8 Oct).
            ev = glob.glob(os.path.join(out, "*", "events.csv"))
            if ev:
                with open(ev[0]) as fh:
                    if any(r.get("kind") == "twist" for r in csv.DictReader(fh)):
                        break
            time.sleep(0.05)
        time.sleep(sigint_at)
        t_sig = time.time()
        pr.send_signal(signal.SIGINT)
    rc = pr.wait(timeout=40)
    pout.close()
    time.sleep(1.5)                       # let the bridge's reaction land
    if br.poll() is None:
        os.killpg(br.pid, signal.SIGINT)
        try:
            br.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(br.pid, signal.SIGKILL)
    bout.close()
    fk.terminate()
    try:
        fk.wait(timeout=5)
    except subprocess.TimeoutExpired:
        fk.kill()
    rows = [json.loads(l) for l in open(log) if l.strip()]
    reqs = [r for r in rows if r["kind"] == "robot_req"]
    runs = glob.glob(os.path.join(out, "*"))
    rd = runs[0] if runs else None
    events = []
    report = ""
    if rd:
        with open(os.path.join(rd, "events.csv")) as fh:
            events = list(csv.DictReader(fh))
        rp = os.path.join(rd, "report.txt")
        report = open(rp).read() if os.path.exists(rp) else ""
    ptxt = open(os.path.join(td, name + ".profile.txt")).read()
    return rc, reqs, events, report, ptxt, t_sig, br.returncode


def main():
    if bt.cable_has_carrier():
        print("REFUSING: %s has carrier. Unplug the Go2 cable." % bt.CABLE_IFACE)
        return 2
    base = os.environ.get("CMD_BRIDGE_IT_DIR")
    if base:
        os.makedirs(base, exist_ok=True)
    td = tempfile.mkdtemp(prefix="cmd_vel_profile_it_", dir=base)
    xml = os.path.join(td, "cyclonedds_loopback.xml")
    open(xml, "w").write(bt.LOOPBACK_XML)
    os.environ["CYCLONEDDS_URI"] = "file://" + xml
    os.environ["ROS_DOMAIN_ID"] = "0"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    print("work dir %s; loopback-only; %s no carrier" % (td, bt.CABLE_IFACE))
    ids = lambda reqs: [r["api_id"] for r in reqs]

    print("P1  walk_03, armed")
    rc, reqs, ev, rep, ptxt, _, _ = run(td, "walk", "walk_03", ["--armed"])
    check("rc 0", rc == 0, ptxt[-600:])
    moves = ids(reqs).count(1008)
    check("robot got 25-30 Moves then ONE StopMove, nothing after",
          25 <= moves <= 30 and ids(reqs).count(1003) == 1 and ids(reqs)[-1] == 1003, ids(reqs)[-6:])
    check("profile recorded 25 Twists", sum(e["kind"] == "twist" for e in ev) == 25)
    check("profile recorded the bridge's robot requests as 'sent'",
          sum(e["kind"] == "sent" for e in ev) == len(reqs), (sum(e["kind"] == "sent" for e in ev), len(reqs)))
    check("report has the PV3 watchdog line and Move rate", "PV3 -- watchdog StopMove" in rep and "Move rate" in rep, rep[-800:])

    print("P2  preflight: --dry against an ARMED bridge -> refuse")
    rc, reqs, ev, rep, ptxt, _, _ = run(td, "mismatch", "walk_03", ["--dry"])
    check("rc 2", rc == 2, ptxt[-400:])
    check("robot received NOTHING", reqs == [], ids(reqs))
    check("refusal names the mismatch", any(e["kind"] == "refuse" and "armed" in e["detail"] for e in ev))

    print("P3  stage-0 rehearsal: robot lying (1001), bridge dry, --require-state WAITING")
    rc, reqs, ev, rep, ptxt, _, _ = run(td, "stage0", "walk_03", ["--dry", "--require-state", "WAITING"],
                                        armed=False, env_extra={"FAKE_ERROR_CODE": "1001"})
    check("rc 0", rc == 0, ptxt[-400:])
    check("robot received NOTHING", reqs == [], ids(reqs))
    check("bridge would have sent nothing either (no dry_ events)",
          not any(e["kind"].startswith("dry") for e in ev), [e["kind"] for e in ev if e["kind"].startswith("dry")])
    check("profile still streamed its 25 Twists", sum(e["kind"] == "twist" for e in ev) == 25)
    rc2, _, ev2, _, ptxt2, _, _ = run(td, "stage0_idle", "walk_03", ["--dry"], armed=False,
                                      env_extra={"FAKE_ERROR_CODE": "1001"})
    check("...and the default --require-state IDLE REFUSES a lying robot (rc 2)", rc2 == 2, ptxt2[-300:])

    print("P4  walk_03_kill: the profile SIGKILLs the bridge")
    rc, reqs, ev, rep, ptxt, _, brc = run(td, "kill", "walk_03_kill", ["--armed"])
    tk = [float(e["t"]) for e in ev if e["kind"] == "kill_bridge"]
    check("kill event recorded", bool(tk))
    if tk:
        after = [r for r in reqs if r["t"] > tk[0] + 0.02]
        check("robot received NOTHING after the kill", after == [], ids(after))
        check("Moves before it (>= 10)", sum(1 for r in reqs if r["api_id"] == 1008 and r["t"] <= tk[0]) >= 10)
    check("bridge process died by signal (rc != 0)", brc not in (0, None), brc)
    check("report has PV4 and 'requests after the kill: 0'",
          "PV4 --" in rep and "requests after the kill: 0" in rep, rep[-800:])

    print("P5  handset mid-walk -> bridge YIELDS -> profile stops streaming")
    rc, reqs, ev, rep, ptxt, _, _ = run(td, "yield", "walk_03", ["--armed"],
                                        env_extra={"FAKE_HANDSET_AT": "6.0"})
    stopped = [e for e in ev if e["kind"] == "stream_stopped"]
    check("profile logged stream_stopped (bridge YIELDED)", stopped and "YIELDED" in stopped[0]["detail"],
          [e["detail"] for e in ev if e["kind"] == "bridge_state"])
    check("fewer than 25 Twists sent", sum(e["kind"] == "twist" for e in ev) < 25)
    if stopped:
        t_s = float(stopped[0]["t"])
        check("no Twist after stream_stopped", not any(e["kind"] == "twist" and float(e["t"]) > t_s for e in ev))

    print("P6  Ctrl+C on the profile mid-stream -> zero Twists -> ONE StopMove")
    rc, reqs, ev, rep, ptxt, t_sig, _ = run(td, "sigint", "walk_03", ["--armed"], sigint_at=0.8)
    check("profile rc 1 (aborted)", rc == 1, ptxt[-400:])
    zeros = [e for e in ev if e["kind"] == "twist" and e["detail"] == "0.000 0.000"]
    check("profile recorded 3 zero Twists", len(zeros) == 3, len(zeros))
    post = [r["api_id"] for r in reqs if t_sig and r["t"] > t_sig]
    check("robot got a StopMove after the Ctrl+C", 1003 in post, post)
    check("...and no Move after that StopMove", 1003 in post and 1008 not in post[post.index(1003):], post)
    check("the StopMove came well before the 0.4 s watchdog would have sent it",
          1003 in post and [r["t"] for r in reqs if r["api_id"] == 1003][0] - t_sig < 0.35,
          [round(r["t"] - t_sig, 3) for r in reqs if r["api_id"] == 1003])

    print("\n%d checks, %d failure(s)\nlogs: %s" % (checks, len(fails), td))
    if checks < 25:
        print("FAIL: fewer checks ran than written")
        return 1
    print("ALL CMD_VEL PROFILE INTEGRATION TESTS PASS" if not fails else "FAILURES:\n  - " + "\n  - ".join(fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
