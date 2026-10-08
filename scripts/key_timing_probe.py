#!/usr/bin/env python3
"""key_timing_probe.py -- measure keyboard auto-repeat timing, the input to teleop (Gate 7 §10).

No ROS, no robot. Two instruments:

  term  -- raw terminal (cbreak). Sees one byte per press/repeat; CANNOT see a release (PK2).
  x11   -- a Tk window. Sees KeyPress / KeyRelease / FocusIn / FocusOut, with X server time.

Protocol per run: hold W ~3 s, release, wait ~2 s, x3; then tap W 5 times ~1 s apart; then
Ctrl+C (term) or Esc (x11). Bounded by --duration either way.

Writes <out>/<tag>_events.csv and <out>/<tag>_report.txt. `--replay CSV` re-analyses a saved file.
"""
import argparse
import csv
import os
import statistics
import sys
import time

CMD_TIMEOUT = 0.4   # go2_cmd_bridge cmd_timeout, s
PAIR_S = 0.005      # a release followed by a press of the same key within this = auto-repeat pair (PK3)
REAL_RELEASE_S = 0.050  # PK4: a release with no press within this is a real release
BURST_S = 0.7       # term mode: same-key presses closer than this belong to one hold.
                    # 1.0 merged Doug's ~0.85-0.98 s taps into a fake "hold" (8 Oct, term_host/term_docker)
REPEAT_MAX_S = 0.1  # a hold counts as auto-repeat only if its median interval is below this


# ---------------------------------------------------------------- analysis (pure, offline-tested)

def classify(events):
    """events: list of dicts t (float s), kind ('press'|'release'|'focus_in'|'focus_out'), key.
    Returns a copy with 'role' added: 'down', 'repeat', 'synthetic_release', 'release', 'focus'.
    A release followed by a press of the same key within PAIR_S makes both halves auto-repeat."""
    ev = [dict(e) for e in sorted(events, key=lambda e: e["t"])]
    for e in ev:
        e["role"] = None
    for i, e in enumerate(ev):
        if e["kind"].startswith("focus"):
            e["role"] = "focus"
        elif e["kind"] == "release":
            nxt = next((x for x in ev[i + 1:] if x["key"] == e["key"] and not x["kind"].startswith("focus")), None)
            if nxt is not None and nxt["kind"] == "press" and nxt["t"] - e["t"] < PAIR_S:
                e["role"] = "synthetic_release"
                nxt["role"] = "repeat"
            else:
                e["role"] = "release"
    for e in ev:
        if e["role"] is None:   # an unpaired press
            e["role"] = "down"
    return ev


def holds(ev, mode):
    """Group classified events into holds: [{key, t0, t1, presses:[t...], real_release:bool}]."""
    out = []
    if mode == "term":
        cur = None
        for e in ev:
            if e["kind"] != "press":
                continue
            if cur and e["key"] == cur["key"] and e["t"] - cur["presses"][-1] < BURST_S:
                cur["presses"].append(e["t"])
            else:
                cur = {"key": e["key"], "presses": [e["t"]], "real_release": False}
                out.append(cur)
        for h in out:
            h["t0"], h["t1"] = h["presses"][0], h["presses"][-1]
        return out
    open_ = {}
    for e in ev:
        if e["role"] == "down":
            open_[e["key"]] = {"key": e["key"], "t0": e["t"], "presses": [e["t"]], "real_release": False}
        elif e["role"] == "repeat" and e["key"] in open_:
            open_[e["key"]]["presses"].append(e["t"])
        elif e["role"] == "release" and e["key"] in open_:
            h = open_.pop(e["key"])
            h["t1"], h["real_release"] = e["t"], True
            out.append(h)
    for h in open_.values():
        h["t1"] = h["presses"][-1]
        out.append(h)
    return sorted(out, key=lambda h: h["t0"])


def hold_stats(h):
    p = h["presses"]
    gaps = [b - a for a, b in zip(p, p[1:])]
    return {
        "dur": h["t1"] - h["t0"],
        "n": len(p),
        "first_gap": gaps[0] if gaps else None,
        "median_interval": statistics.median(gaps[1:]) if len(gaps) > 1 else None,
        "max_interval_after_first": max(gaps[1:]) if len(gaps) > 1 else None,
    }


def report(events, mode, meta):
    ev = classify(events)
    hs = holds(ev, mode)
    roles = {}
    for e in ev:
        roles[e["role"]] = roles.get(e["role"], 0) + 1
    pair_gaps = []
    for i, e in enumerate(ev):
        if e["role"] == "synthetic_release":
            nxt = next(x for x in ev[i + 1:] if x["key"] == e["key"] and x["role"] == "repeat")
            pair_gaps.append(nxt["t"] - e["t"])
    L = [f"key_timing_probe report -- mode {mode}"]
    L += [f"  {k}: {v}" for k, v in meta.items()]
    L.append(f"events: {len(ev)}  roles: {roles}")
    if pair_gaps:
        L.append(f"auto-repeat release->press pairs: n={len(pair_gaps)}  max gap {max(pair_gaps)*1e3:.3f} ms  "
                 f"median {statistics.median(pair_gaps)*1e3:.3f} ms")
    else:
        L.append("auto-repeat release->press pairs: none")
    L.append("")
    L.append(f"{'#':>2} {'key':>8} {'dur s':>7} {'n':>4} {'first gap ms':>13} {'median ms':>10} {'max ms':>8} "
             f"{'real rel':>8}")
    firsts, meds = [], []
    for i, h in enumerate(hs):
        s = hold_stats(h)
        f = "-" if s["first_gap"] is None else f"{s['first_gap']*1e3:.1f}"
        m = "-" if s["median_interval"] is None else f"{s['median_interval']*1e3:.2f}"
        x = "-" if s["max_interval_after_first"] is None else f"{s['max_interval_after_first']*1e3:.1f}"
        L.append(f"{i:>2} {h['key']:>8} {s['dur']:>7.3f} {s['n']:>4} {f:>13} {m:>10} {x:>8} {str(h['real_release']):>8}")
        if s["n"] >= 3 and s["median_interval"] < REPEAT_MAX_S:
            firsts.append(s["first_gap"])
            meds.append(s["median_interval"])
    L.append("")
    if firsts:
        L.append(f"auto-repeat holds (>=3 events, median interval < {REPEAT_MAX_S*1e3:.0f} ms): {len(firsts)}")
        L.append(f"  first gap  : min {min(firsts)*1e3:.1f}  max {max(firsts)*1e3:.1f} ms")
        L.append(f"  median int : min {min(meds)*1e3:.2f}  max {max(meds)*1e3:.2f} ms")
        verdict = "EXCEEDS" if max(firsts) > CMD_TIMEOUT else "within"
        L.append(f"  first gap vs cmd_timeout {CMD_TIMEOUT*1e3:.0f} ms: {verdict}")
        if verdict == "EXCEEDS":
            L.append("  -> a stream riding auto-repeat lets the watchdog fire on every key-down")
    else:
        L.append("no auto-repeat hold -- nothing to say about repeat timing")
    return "\n".join(L) + "\n"


# ---------------------------------------------------------------- instruments

def run_term(duration):
    import select
    import termios
    import tty
    if not sys.stdin.isatty():
        sys.exit("term mode needs a real terminal (stdin is not a tty)")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    events = []
    print(f"term mode: hold W ~3 s, release, wait ~2 s, x3; tap W x5; Ctrl+C to end (max {duration:.0f} s)",
          flush=True)
    t_end = time.monotonic() + duration
    try:
        tty.setcbreak(fd)
        while time.monotonic() < t_end:
            r, _, _ = select.select([fd], [], [], 0.1)
            if not r:
                continue
            t = time.monotonic()
            for b in os.read(fd, 64):   # one read may carry several bytes; same t for all
                if b == 3:
                    raise KeyboardInterrupt
                events.append({"t": t, "kind": "press", "key": f"0x{b:02x}", "xtime": ""})
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return events


def run_x11(duration):
    import tkinter as tk
    events = []
    root = tk.Tk()
    root.title("key_timing_probe -- focus me")
    tk.Label(root, font=("TkDefaultFont", 14), padx=30, pady=30, justify="left",
             text="Hold W ~3 s, release, wait ~2 s, x3\nthen tap W x5\nEsc to end").pack()

    def rec(kind):
        def h(e):
            ks = getattr(e, "keysym", "") if kind in ("press", "release") else ""
            events.append({"t": time.monotonic(), "kind": kind, "key": ks, "xtime": getattr(e, "time", "")})
            if kind == "press" and ks == "Escape":
                root.destroy()
        return h

    root.bind("<KeyPress>", rec("press"))
    root.bind("<KeyRelease>", rec("release"))
    root.bind("<FocusIn>", rec("focus_in"))
    root.bind("<FocusOut>", rec("focus_out"))
    root.after(int(duration * 1000), root.destroy)
    root.mainloop()
    return [e for e in events if e["key"] != "Escape"]


def load_csv(path):
    with open(path) as f:
        return [{"t": float(r["t"]), "kind": r["kind"], "key": r["key"], "xtime": r["xtime"]}
                for r in csv.DictReader(f)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["term", "x11"])
    ap.add_argument("--tag", default=None)
    ap.add_argument("--out", default=os.path.expanduser("~/hexapod_bridge_go2/logs/teleop/keytiming"))
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--replay", default=None, help="re-analyse a saved events CSV; no capture")
    a = ap.parse_args()

    if a.replay:
        events = load_csv(a.replay)
        sys.stdout.write(report(events, a.mode, {"replay": a.replay}))
        return

    tag = a.tag or f"{a.mode}_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}"
    meta = {"tag": tag, "utc_start": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "in_container": os.path.exists("/.dockerenv"), "pair_s": PAIR_S, "burst_s": BURST_S}
    events = run_term(a.duration) if a.mode == "term" else run_x11(a.duration)
    os.makedirs(a.out, exist_ok=True)
    if os.path.exists(os.path.join(a.out, f"{tag}_events.csv")):
        # a redo under the same tag silently destroyed run 1 on 8 Oct; refuse rather than overwrite
        tag = f"{tag}_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}"
        print(f"tag already used in {a.out}; saving as {tag}")
    t0 = events[0]["t"] if events else 0.0
    with open(os.path.join(a.out, f"{tag}_events.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["t", "kind", "key", "xtime"])
        w.writeheader()
        for e in events:
            w.writerow({**e, "t": f"{e['t'] - t0:.6f}"})
    rel = [{**e, "t": e["t"] - t0} for e in events]
    txt = report(rel, a.mode, meta)
    with open(os.path.join(a.out, f"{tag}_report.txt"), "w") as f:
        f.write(txt)
    sys.stdout.write(txt)
    print(f"wrote {a.out}/{tag}_events.csv and _report.txt")


if __name__ == "__main__":
    main()
