#!/usr/bin/env python3
"""END-TO-END: real X key events -> go2_teleop.py (Tk) -> the REAL go2_cmd_bridge_node -> a fake Go2.
Needs ROS 2 + unitree msgs + go2_cmd_bridge built and sourced, Xvfb and xdotool. NOT the robot.

The fake robot is test_cmd_bridge_integration_rosonly.py's "robot_only". Keys are pressed by
xdotool (XTEST) into a headless Xvfb, so the X server -- not this test -- generates any
auto-repeat. The window, its 10 ms timer, rclpy and the bridge all run for real.

  K1  hold W 2.0 s: Moves at 10 Hz all through the hold, NO StopMove inside it, exactly ONE
      after the release; key-up -> robot StopMove latency measured
  K2  preflight: teleop --dry against an ARMED bridge -> rc 2, the robot gets NOTHING
  K3  W, then add A, let A go, let W go: Moves carry z = 1.0 only while A is down
  K4  focus moves to another window mid-hold -> zero -> StopMove, though W stays held
  K5  SIGKILL the teleop mid-hold -> the bridge's watchdog StopMove (<= 0.5 s)
  K6  SIGINT the teleop mid-hold -> zeros -> StopMove, promptly
  K7  handset press mid-hold -> bridge YIELDED -> teleop latches, publishes no motion after

⛔ Same isolation as the bridge test: loopback-only Cyclone, REFUSES TO RUN if the Go2 cable
interface has carrier. RUN IT WITH THE CABLE UNPLUGGED.
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

DISPLAY = ":97"         # NOT :0 -- with --network host the abstract X socket is the host's
fails = []
checks = 0


def check(label, ok, detail=""):
    global checks
    checks += 1
    print("  %s %s%s" % ("pass" if ok else "FAIL", label, (" -- " + str(detail)[:500]) if not ok else ""))
    if not ok:
        fails.append(label)


def xdo(*a):
    return subprocess.run(["xdotool"] + [str(x) for x in a], capture_output=True, text=True,
                          env=dict(os.environ, DISPLAY=DISPLAY)).stdout.strip()


def run(td, name, script, teleop_args=("--armed",), armed=True, env_extra=None, fake_s=20):
    """script(ctx) drives keys; ctx has win, t0 and record(label) for key-time stamps."""
    env = dict(os.environ, DISPLAY=DISPLAY)
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
    time.sleep(3.0)
    out = os.path.join(td, name)
    os.makedirs(out)
    tout = open(os.path.join(td, name + ".teleop.txt"), "w")
    tp = subprocess.Popen([sys.executable, os.path.join(HERE, "go2_teleop.py"), *teleop_args, "--out", out],
                          env=env, stdout=tout, stderr=subprocess.STDOUT)
    ctx = {"marks": {}, "tp": tp}
    ctx["record"] = lambda label: ctx["marks"].__setitem__(label, time.time())
    win = ""
    t_end = time.time() + 10
    while not win and time.time() < t_end and tp.poll() is None:
        win = xdo("search", "--name", "go2_teleop").split("\n")[0]
        time.sleep(0.2)
    ctx["win"] = win
    if win:
        xdo("windowfocus", "--sync", win)
        time.sleep(1.0)
        script(ctx)
    if tp.poll() is None:
        xdo("key", "q")
        try:
            tp.wait(timeout=5)
        except subprocess.TimeoutExpired:
            tp.kill()
    ctx["rc"] = tp.wait()
    fk.wait(timeout=fake_s + 10)
    if br.poll() is None:
        os.killpg(br.pid, signal.SIGINT)
        try:
            br.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(br.pid, signal.SIGKILL)
    tout.close()
    rows = [json.loads(l) for l in open(log) if l.strip()]
    evf = glob.glob(os.path.join(out, "teleop_*", "events.csv"))
    ctx["events"] = list(csv.DictReader(open(evf[0]))) if evf else []
    ctx["teleop_out"] = open(os.path.join(td, name + ".teleop.txt")).read()
    return rows, ctx


def reqs(rows, aid=None):
    return [r for r in rows if r["kind"] == "robot_req" and (aid is None or r["api_id"] == aid)]


def twists(ctx):
    return [e for e in ctx["events"] if e["kind"] == "twist"]


def quit_counts(ctx):
    q = [e for e in ctx["events"] if e["kind"] == "quit"]
    if not q:
        return {}
    return json.loads(q[0]["detail"].split("counts ", 1)[1])


def hold_w(ctx, secs, mark="up"):
    ctx["record"]("down")
    xdo("keydown", "w")
    time.sleep(secs)
    ctx["record"](mark)
    xdo("keyup", "w")


def main():
    if bt.cable_has_carrier():
        print("REFUSING: %s has carrier. Unplug the Go2 cable -- this test runs an ARMED bridge "
              "against robot-named topics on domain 0." % bt.CABLE_IFACE)
        return 2
    td = tempfile.mkdtemp(prefix="teleop_it_", dir=os.environ.get("TELEOP_IT_DIR"))
    xml = os.path.join(td, "cyclonedds_loopback.xml")
    open(xml, "w").write(bt.LOOPBACK_XML)
    os.environ.update({"CYCLONEDDS_URI": "file://" + xml, "ROS_DOMAIN_ID": "0",
                       "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"})
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    xv = subprocess.Popen(["Xvfb", DISPLAY, "-nolisten", "tcp", "-screen", "0", "800x600x24"],
                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    time.sleep(1.5)
    print("work dir %s; loopback-only Cyclone; %s no carrier; Xvfb %s" % (td, bt.CABLE_IFACE, DISPLAY))
    xset = subprocess.run(["sh", "-c", "DISPLAY=%s xset q 2>&1 | grep -i 'auto repeat'" % DISPLAY],
                          capture_output=True, text=True).stdout.strip()
    print("  Xvfb auto-repeat: %s" % (xset or "(xset not installed -- see K1's autorepeat count)"))
    R = {}
    try:
        print("K1  hold W 2.0 s")
        rows, ctx = run(td, "k1", lambda c: (hold_w(c, 2.0), time.sleep(1.5)))
        moves, stops = reqs(rows, 1008), reqs(rows, 1003)
        d, u = ctx["marks"].get("down"), ctx["marks"].get("up")
        check("window found and focused", bool(ctx["win"]), ctx["teleop_out"][-400:])
        inside = [m for m in moves if d and u and d <= m["t"] <= u]
        check("18-22 Moves during the 2.0 s hold", 18 <= len(inside) <= 22, len(inside))
        if len(inside) > 2:
            g = [b["t"] - a["t"] for a, b in zip(inside, inside[1:])]
            R["k1_maxgap"] = max(g)
            check("no Move gap over 0.2 s in the hold (the 500 ms repeat delay is invisible)",
                  max(g) <= 0.2, max(g))
        check("NO StopMove during the hold", not [s for s in stops if d <= s["t"] <= u], stops)
        after = [s for s in stops if u and s["t"] > u]
        check("exactly ONE StopMove after the release", len(after) == 1, len(after))
        if after:
            R["k1_lat"] = after[0]["t"] - u
            check("key-up -> robot StopMove <= 0.20 s (debounce 20 ms + 10 ms poll + <=100 ms tick)",
                  R["k1_lat"] <= 0.20, R["k1_lat"])
        check("no Move after that StopMove", after and not [m for m in moves if m["t"] > after[0]["t"]])
        qc = quit_counts(ctx)
        R["k1_autorepeat"] = qc.get("autorepeat")
        print("     teleop counts: %s" % qc)
        check("teleop exited 0 on Q", ctx["rc"] == 0, ctx["rc"])

        print("K2  preflight: --dry teleop vs ARMED bridge")
        rows, ctx = run(td, "k2", lambda c: hold_w(c, 1.0), teleop_args=("--dry",), fake_s=15)
        check("teleop refused (rc 2)", ctx["rc"] == 2, (ctx["rc"], ctx["teleop_out"][-300:]))
        check("no window was ever shown", not ctx["win"])
        check("robot received NOTHING", reqs(rows) == [], reqs(rows)[:3])
        check("teleop published no Twist", twists(ctx) == [])

        print("K3  W, add A, release A, release W")

        def k3(c):
            c["record"]("w_down"); xdo("keydown", "w"); time.sleep(1.0)
            c["record"]("a_down"); xdo("keydown", "a"); time.sleep(1.0)
            c["record"]("a_up"); xdo("keyup", "a"); time.sleep(1.0)
            c["record"]("w_up"); xdo("keyup", "w"); time.sleep(1.5)
        rows, ctx = run(td, "k3", k3)
        m = ctx["marks"]
        moves = reqs(rows, 1008)
        z = lambda r: json.loads(r["parameter"])["z"]
        seg = lambda a, b: [r for r in moves if m[a] + 0.15 <= r["t"] <= m[b]]
        check("W alone: z = 0", seg("w_down", "a_down") and all(z(r) == 0.0 for r in seg("w_down", "a_down")))
        check("W + A: z = 1.0", seg("a_down", "a_up") and all(z(r) == 1.0 for r in seg("a_down", "a_up")),
              [z(r) for r in seg("a_down", "a_up")])
        check("after A's release, W alone again: z = 0, still moving (no W event in between)",
              seg("a_up", "w_up") and all(z(r) == 0.0 for r in seg("a_up", "w_up")),
              [z(r) for r in seg("a_up", "w_up")])
        check("one StopMove, after W's release", len(reqs(rows, 1003)) == 1
              and reqs(rows, 1003)[0]["t"] > m["w_up"], [(s["t"] - m["w_up"]) for s in reqs(rows, 1003)])

        print("K4  focus moves to another window mid-hold")

        def k4(c):
            decoy = subprocess.Popen([sys.executable, "-c",
                                      "import tkinter as t; r=t.Tk(); r.title('decoy'); r.after(9000, r.destroy); r.mainloop()"],
                                     env=dict(os.environ, DISPLAY=DISPLAY))
            dw = ""
            t_end = time.time() + 5
            while not dw and time.time() < t_end:
                dw = xdo("search", "--name", "decoy").split("\n")[0]
                time.sleep(0.2)
            xdo("windowfocus", "--sync", c["win"])
            time.sleep(0.5)
            c["record"]("down"); xdo("keydown", "w"); time.sleep(1.0)
            c["record"]("focus"); xdo("windowfocus", dw); time.sleep(1.5)
            c["record"]("up"); xdo("keyup", "w"); time.sleep(0.5)
            xdo("windowfocus", "--sync", c["win"]); time.sleep(0.3)
            decoy.kill()
        rows, ctx = run(td, "k4", k4)
        m = ctx["marks"]
        stops = reqs(rows, 1003)
        check("a StopMove within 0.20 s of the focus change, W still held",
              stops and 0 < stops[0]["t"] - m["focus"] <= 0.20, [s["t"] - m["focus"] for s in stops])
        check("no Move between 0.2 s after the focus change and the end",
              not [r for r in reqs(rows, 1008) if r["t"] > m["focus"] + 0.2])
        check("teleop logged focus_out", any(e["kind"] == "focus_out" for e in ctx["events"]))

        print("K5  SIGKILL the teleop mid-hold")

        def k5(c):
            c["record"]("down"); xdo("keydown", "w"); time.sleep(1.5)
            c["record"]("kill"); c["tp"].kill(); time.sleep(1.5)
            xdo("keyup", "w")
        rows, ctx = run(td, "k5", k5)
        k = ctx["marks"]["kill"]
        stops = reqs(rows, 1003)
        check("one StopMove 0.3-0.6 s after the kill (the bridge's 0.4 s watchdog)",
              len(stops) == 1 and 0.3 <= stops[0]["t"] - k <= 0.6, [s["t"] - k for s in stops])
        R["k5_lat"] = stops[0]["t"] - k if stops else None

        print("K6  SIGINT the teleop mid-hold")

        def k6(c):
            c["record"]("down"); xdo("keydown", "w"); time.sleep(1.5)
            c["record"]("int"); c["tp"].send_signal(signal.SIGINT); time.sleep(1.5)
            xdo("keyup", "w")
        rows, ctx = run(td, "k6", k6)
        i = ctx["marks"]["int"]
        stops = reqs(rows, 1003)
        check("one StopMove within 0.20 s of SIGINT", len(stops) == 1 and 0 < stops[0]["t"] - i <= 0.20,
              [s["t"] - i for s in stops])
        check("teleop exited 0 and logged its quit", ctx["rc"] == 0 and any(e["kind"] == "quit" for e in ctx["events"]),
              ctx["rc"])

        print("K7  handset press mid-hold")
        # The fake's clock starts ~2 s before the bridge, ~5+ s before the window: press at 10 s.
        rows, ctx = run(td, "k7", lambda c: (c["record"]("down"), xdo("keydown", "w"), time.sleep(6.0),
                                            xdo("keyup", "w"), time.sleep(1.0)),
                        env_extra={"FAKE_HANDSET_AT": "10.0"})
        hs = [r["t"] for r in rows if r["kind"] == "event" and r["what"] == "handset keys 288"]
        check("handset press happened during the hold", bool(hs))
        latch = [e for e in ctx["events"] if e["kind"] == "latch"]
        check("teleop latched on the bridge's YIELD", bool(latch), [e["kind"] for e in ctx["events"]][-8:])
        if hs:
            check("robot received NOTHING after the handset press", not [r for r in reqs(rows) if r["t"] > hs[0] + 0.05],
                  [(r["api_id"], r["t"] - hs[0]) for r in reqs(rows) if r["t"] > hs[0]])
        if latch:
            lt = float(latch[0]["t"])
            check("teleop published no motion after its latch",
                  not [e for e in twists(ctx) if float(e["t"]) > lt and e["detail"] != "0.000 0.000"])
    finally:
        xv.terminate()

    print()
    print("measured: %s" % json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in R.items()}))
    print("%d checks, %d failure(s)" % (checks, len(fails)))
    if checks < 25:
        print("FAIL: fewer checks ran than written -- refusing to report success")
        return 1
    print("ALL TELEOP INTEGRATION TESTS PASS" if not fails else "FAILURES:\n  - " + "\n  - ".join(fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
