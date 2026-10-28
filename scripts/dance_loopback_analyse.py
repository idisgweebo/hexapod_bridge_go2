#!/usr/bin/env python3
"""Friday dance demo -- score a dance_loopback_test.sh run against PDL-0..PDL-7. Pure: no ROS, re-runnable on a saved OUT dir.

    dance_loopback_analyse.py OUT_DIR

Predictions (registered 28 Oct 2026, before the first run):
  PDL-0  the bridge loads dance_bridge.yaml: its log creates a bridge for /go2/action and NONE for /go2/cmd_vel
  PDL-1  C (bridge on domain 42, control): the fake sees 0 triggers, while the node's status DOES reach it (instrument alive)
  PDL-2  D (dry, 10 Hz x 5 s from ROS 1): 30..51 triggers arrive (none duplicated by the bidirectional bridge), the node
         counts the same number (+-2), exactly ONE would-send routed "dry", nothing on /api/sport/request
  PDL-3  O (dry, `rostopic pub --once`): the single latched message crosses -> 1 trigger, ONE would-send
  PDL-4  A (armed vs fake, 10 Hz x 5 s): exactly ONE 1022 at the fake and nothing else; DANCING -> LOCKOUT -> SPENT;
         last_response (1022, 0); routed "robot", matched subscribers = the fake only
  PDL-5  R (ROS 2 pub x5): all 5 reach the ROS 1 master (bidirectional, gate 8 D2) and the fake sees 5, not 10 (no loop)
  PDL-6  the node started from the RUN SHEET's --params-file path in D, O, A (startup line, lockout_s 5.0, DRY/ARMED right)
  PDL-7  hygiene: ip_forward 0 at every logged point; no /gate8_bridge parameter left on the master after any phase
Clocks: one host wall clock (all containers share it).
"""
import glob
import json
import os
import sys

OUT = sys.argv[1]
results = []


def score(pid, ok, detail):
    tag = "PASS" if ok is True else ("FAIL" if ok is False else "n/a ")
    results.append((pid, tag))
    print("  %s %-6s %s" % (tag, pid, detail))


def rd(path, default=""):
    try:
        return open(path).read()
    except OSError:
        return default


def jsonl(path):
    rows = []
    for line in rd(path).splitlines():
        if line.strip():
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass
    return rows


def phase(n):
    p = os.path.join(OUT, n)
    if not os.path.isdir(p):
        return None
    rows = jsonl(os.path.join(p, "fake.jsonl"))
    st = [r["status"] for r in rows if r["kind"] == "status"]
    seq = []
    for s in st:
        if not seq or seq[-1] != s["state"]:
            seq.append(s["state"])
    blog = "".join(rd(f) for f in glob.glob(os.path.join(p, "bridge_logs", "*", "bridge.txt")))
    cfg = "".join(rd(f) for f in glob.glob(os.path.join(p, "bridge_logs", "*", "config.txt")))
    echo = [l for l in rd(os.path.join(p, "ros1_echo.csv")).splitlines() if l and not l.startswith("%")]
    return {"rows": rows, "seen": [r for r in rows if r["kind"] == "trigger_seen"],
            "breq": [r for r in rows if r["kind"] == "bridge_req"],
            "robot": [r for r in rows if r["kind"] == "robot_req"],
            "status": st, "last": st[-1] if st else {}, "seq": seq, "bridge_log": blog, "cfg": cfg,
            "node": rd(os.path.join(p, "node.txt")), "echo": echo, "master": rd(os.path.join(p, "master_after.txt")),
            "t0": float(rd(os.path.join(p, "t_source_start.txt"), "0").strip() or 0)}


P = {n: phase(n) for n in "CDOAR"}
print("dance loopback analysis: %s  (phases present: %s)" % (OUT, "".join(n for n in P if P[n])))

# PDL-0
logs = [(n, P[n]["bridge_log"], P[n]["cfg"]) for n in P if P[n]]
ok0 = bool(logs) and all("/go2/action" in b and "create bidirectional bridge" in b and "/go2/cmd_vel" not in b
                         and "dance_bridge.yaml" in c for _, b, c in logs)
score("PDL-0", ok0, "per phase: %s" % [(n, "/go2/action" in b, "/go2/cmd_vel" in b, "dance_bridge.yaml" in c)
                                         for n, b, c in logs])

# PDL-1
c = P["C"]
if c:
    score("PDL-1", len(c["seen"]) == 0 and c["last"].get("counts", {}).get("triggers") == 0 and len(c["status"]) > 5,
          "C: fake saw %d triggers; node triggers %s; %d status msgs reached the fake (instrument alive); ros1 echo %d"
          % (len(c["seen"]), c["last"].get("counts", {}).get("triggers"), len(c["status"]), len(c["echo"])))
else:
    score("PDL-1", None, "phase C not run")

# PDL-2
d = P["D"]
if d:
    nt = d["last"].get("counts", {}).get("triggers")
    lat = (d["seen"][0]["t"] - d["t0"]) if d["seen"] and d["t0"] else None
    score("PDL-2", 30 <= len(d["seen"]) <= 51 and nt is not None and abs(nt - len(d["seen"])) <= 2
          and [(b["api_id"], b["routed"]) for b in d["breq"]] == [(1022, "dry")] and d["robot"] == [],
          "D: fake saw %d, node counted %s, ros1 echo %d; would-sends %s; robot %d; first arrival %s s after the "
          "docker-exec of rostopic pub; states %s" % (len(d["seen"]), nt, len(d["echo"]),
                                                      [(b["api_id"], b["routed"]) for b in d["breq"]], len(d["robot"]),
                                                      None if lat is None else round(lat, 2), d["seq"]))
else:
    score("PDL-2", None, "phase D not run")

# PDL-3
o = P["O"]
if o:
    score("PDL-3", len(o["seen"]) == 1 and [(b["api_id"], b["routed"]) for b in o["breq"]] == [(1022, "dry")]
          and o["robot"] == [],
          "O: fake saw %d, would-sends %s, robot %d, states %s"
          % (len(o["seen"]), [(b["api_id"], b["routed"]) for b in o["breq"]], len(o["robot"]), o["seq"]))
else:
    score("PDL-3", None, "phase O not run")

# PDL-4
a = P["A"]
if a:
    br = a["breq"]
    score("PDL-4", [r["api_id"] for r in a["robot"]] == [1022] and a["seq"][-3:] == ["DANCING", "LOCKOUT", "SPENT"]
          and a["last"].get("last_response") == [1022, 0] and len(br) == 1 and br[0]["routed"] == "robot"
          and br[0]["subscribers"] == ["fake_go2_and_hexapod"],
          "A: robot got %s; states %s; last_response %s; routed %s subs %s; fake saw %d triggers, node counted %s"
          % ([r["api_id"] for r in a["robot"]], a["seq"], a["last"].get("last_response"),
             [b["routed"] for b in br], [b["subscribers"] for b in br], len(a["seen"]),
             a["last"].get("counts", {}).get("triggers")))
else:
    score("PDL-4", None, "phase A not run")

# PDL-5
r = P["R"]
if r:
    score("PDL-5", len(r["echo"]) == 5 and len(r["seen"]) == 5,
          "R: ROS 1 echo %d (expect 5), fake saw %d (expect 5; 10 = a loop)" % (len(r["echo"]), len(r["seen"])))
else:
    score("PDL-5", None, "phase R not run")

# PDL-6
nodes = [(n, P[n]["node"]) for n in "DOA" if P[n]]
# ... and the node log is COMPLETE: the send line and the exit line are there (28 Oct run 1: `setsid` under docker
# exec cut node.txt after 2 lines while the node ran on -- a harness defect the fake's record did not show)
ok6 = bool(nodes) and all("go2_action_bridge up" in t and '"lockout_s": 5.0' in t
                          and (("ARMED" in t) == (n == "A")) and "go2_action_bridge exiting" in t
                          and (("robot DANCE1 api_id 1022" in t) if n == "A" else ("dry DANCE1 api_id 1022" in t))
                          for n, t in nodes)
score("PDL-6", ok6, "(phase, startup, armed-as-expected, send line, exit line): %s"
      % [(n, "go2_action_bridge up" in t, "ARMED" in t and n == "A" or "DRY RUN" in t and n != "A",
          "DANCE1 api_id 1022" in t, "go2_action_bridge exiting" in t) for n, t in nodes])

# PDL-7
summ = rd(os.path.join(OUT, "summary.txt"))
fw = [l for l in summ.splitlines() if "ip_forward=" in l]
bad_fw = [l for l in fw if "ip_forward=1" in l]
left = [n for n in P if P[n] and "gate8" in P[n]["master"]]
score("PDL-7", bool(fw) and not bad_fw and not left,
      "%d ip_forward readings, %d were 1; phases leaving /gate8_bridge: %s" % (len(fw), len(bad_fw), left))

fails = [p for p, t in results if t == "FAIL"]
nas = [p for p, t in results if t == "n/a "]
print("\n%d scored: %d PASS, %d FAIL, %d n/a" % (len(results), len(results) - len(fails) - len(nas), len(fails), len(nas)))
print("ALL DANCE LOOPBACK PREDICTIONS PASS" if not fails and not nas else
      "NOT ALL PASS: %s" % (fails + nas))
sys.exit(1 if fails else 0)
