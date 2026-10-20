#!/usr/bin/env python3
"""Go2 power profile -- READ-ONLY: subscribes to /lowstate and /sportmodestate, never publishes.

WHY THIS EXISTS
---------------
go2_wireless_options.md registered PWR0 / PWR1 (20 Oct 2026):
  PWR0  bms_state.current x power_v, lying, exceeds power_v x power_a by >= 10 W
        (tests the inference that power_a is the motor bus only).
  PWR1  pack draw lying idle 20-60 W; standing 110-200 W.
power_v x power_a reads 8-13 W lying -- implausibly low for a robot with an on-board
computer and a LiDAR -- so it is not trusted as pack draw. bms_state.current is the
BMS's own number. ⚠ Its UNITS AND SIGN ARE UNKNOWN: this script records it RAW and
only prints a wattage under the label "if mA, discharge-negative", an assumption.

WHAT IT RECORDS
---------------
/lowstate arrives at 500 Hz. One CSV row per DECIMATE messages (default 10 -> 50 Hz),
carrying the newest values PLUS the window's max power_a and min power_v, so a
stand-up transient is not lost to decimation. Posture comes from the newest
SportModeState (error_code: 1001 lying, 1002 stand-lock, 100 balance stand, 2006
standing down -- s12; body_height).

  go2_power_profile.py --out /logs/power.csv          record until Ctrl+C (or --duration)
  go2_power_profile.py --replay power.csv             per-posture-segment summary, NO ROS needed
  go2_power_profile.py --selftest                     pure-logic checks, NO ROS needed
"""
import argparse, csv, math, os, statistics, subprocess, sys, tempfile, time

RR_HIP, RL_HIP = 6, 9
DECIMATE = 10
STALE_S = 1.0
EDGE_S = 3.0          # seconds trimmed from each end of a segment before the means (transients)
FIELDS = ["t_wall", "tick", "n_rx", "power_v", "power_a", "power_a_max", "power_v_min",
          "bms_current", "bms_soc", "bms_status", "cell_sum_mv", "n_cells", "bq_ntc0", "bq_ntc1",
          "mcu_ntc0", "mcu_ntc1", "ntc1", "ntc2", "rr_hip_c", "rl_hip_c", "max_other_c",
          "error_code", "body_height"]
POSTURE = {1001: "lying", 1002: "stand-lock", 100: "balance-stand", 2006: "standing-down"}


def p_pack_if_ma(v, cur):
    """Pack watts IF bms_current is mA and discharge is negative. An assumption, labelled as one."""
    return v * (-cur) / 1000.0


def segments(rows):
    """Split rows into contiguous runs of the same error_code. Returns [(code, [rows])]."""
    out = []
    for r in rows:
        code = r["error_code"]
        if out and out[-1][0] == code:
            out[-1][1].append(r)
        else:
            out.append((code, [r]))
    return out


def seg_summary(code, rs, edge_s=EDGE_S):
    t0, t1 = rs[0]["t_wall"], rs[-1]["t_wall"]
    core = [r for r in rs if t0 + edge_s <= r["t_wall"] <= t1 - edge_s]
    s = {"code": code, "posture": POSTURE.get(code, f"code {code}"), "t0": t0, "dur_s": t1 - t0,
         "n": len(rs), "n_core": len(core)}
    s["peak_bus_w"] = max(r["power_v_min"] * r["power_a_max"] for r in rs)
    s["peak_pack_w"] = max(p_pack_if_ma(r["power_v"], r["bms_current"]) for r in rs)
    if core:
        bus = [r["power_v"] * r["power_a"] for r in core]
        pack = [p_pack_if_ma(r["power_v"], r["bms_current"]) for r in core]
        cur = [r["bms_current"] for r in core]
        s.update(bus_w=statistics.fmean(bus), pack_w=statistics.fmean(pack),
                 bms_cur_min=min(cur), bms_cur_max=max(cur),
                 bms_cur_distinct=len(set(cur)),
                 # how often the BMS number actually changes -- a slow BMS is not a 500 Hz sensor
                 bms_changes=sum(1 for a, b in zip(cur, cur[1:]) if a != b),
                 v_mean=statistics.fmean(r["power_v"] for r in core),
                 cell_sum_v=statistics.fmean(r["cell_sum_mv"] for r in core) / 1000.0,
                 soc=(core[0]["bms_soc"], core[-1]["bms_soc"]),
                 rr=(core[0]["rr_hip_c"], core[-1]["rr_hip_c"]),
                 rl=(core[0]["rl_hip_c"], core[-1]["rl_hip_c"]),
                 height=statistics.fmean(r["body_height"] for r in core))
    return s


def report(rows, out=sys.stdout):
    p = lambda *a: print(*a, file=out)
    if not rows:
        p("NO ROWS. Not a power finding -- check cable, domain, robot power.")
        return
    gaps = [b["t_wall"] - a["t_wall"] for a, b in zip(rows, rows[1:])]
    p(f"{len(rows)} rows over {rows[-1]['t_wall'] - rows[0]['t_wall']:.1f} s; "
      f"max row gap {max(gaps, default=0):.3f} s; lowstate msgs {rows[-1]['n_rx']}")
    p(f"⚠ pack_W assumes bms_current is mA, discharge-negative -- NOT established. Raw counts shown too.")
    p(f"Means exclude {EDGE_S:.0f} s at each segment edge; peaks use every row (window max A x min V).")
    p("-" * 118)
    p(f"{'start(UTC)':>12s} {'posture':>14s} {'dur s':>7s} {'bus W':>7s} {'pack W':>7s} {'pk bus':>7s} "
      f"{'pk pack':>7s} {'bms raw min..max':>17s} {'chg':>5s} {'V':>6s} {'cells V':>7s} {'soc':>7s} "
      f"{'RR C':>9s} {'RL C':>9s} {'h m':>5s}")
    for code, rs in segments(rows):
        s = seg_summary(code, rs)
        ts = time.strftime("%H:%M:%S", time.gmtime(s["t0"]))
        if s["n_core"] == 0:
            p(f"{ts:>12s} {s['posture']:>14s} {s['dur_s']:7.1f}   (shorter than {2*EDGE_S:.0f} s -- transient; "
              f"peak bus {s['peak_bus_w']:.1f} W, peak pack {s['peak_pack_w']:.1f} W)")
            continue
        p(f"{ts:>12s} {s['posture']:>14s} {s['dur_s']:7.1f} {s['bus_w']:7.1f} {s['pack_w']:7.1f} "
          f"{s['peak_bus_w']:7.1f} {s['peak_pack_w']:7.1f} {s['bms_cur_min']:>8d}..{s['bms_cur_max']:<8d} "
          f"{s['bms_changes']:5d} {s['v_mean']:6.2f} {s['cell_sum_v']:7.2f} {s['soc'][0]:>3d}>{s['soc'][1]:<3d} "
          f"{s['rr'][0]:>4.0f}>{s['rr'][1]:<4.0f} {s['rl'][0]:>4.0f}>{s['rl'][1]:<4.0f} {s['height']:5.3f}")


def load_csv(path):
    ints = {"tick", "n_rx", "bms_current", "bms_soc", "bms_status", "cell_sum_mv", "n_cells", "bq_ntc0",
            "bq_ntc1", "mcu_ntc0", "mcu_ntc1", "ntc1", "ntc2", "error_code"}
    with open(path) as f:
        return [{k: (int(v) if k in ints else float(v)) for k, v in r.items()} for r in csv.DictReader(f)]


def run_selftest():
    fails = []
    def chk(name, got, want):
        ok = got == want if not isinstance(want, float) else math.isclose(got, want, rel_tol=1e-9)
        print(f"  {'ok  ' if ok else 'FAIL'} {name}: got {got!r}, want {want!r}")
        if not ok: fails.append(name)

    def row(t, code, cur=-1065, v=29.4, a=0.34, amax=None, vmin=None, n=0):
        return dict(t_wall=t, tick=int(t*1000), n_rx=n, power_v=v, power_a=a,
                    power_a_max=a if amax is None else amax, power_v_min=v if vmin is None else vmin,
                    bms_current=cur, bms_soc=65, bms_status=8, cell_sum_mv=30331, n_cells=8,
                    bq_ntc0=31, bq_ntc1=29, mcu_ntc0=33, mcu_ntc1=32, ntc1=30, ntc2=30,
                    rr_hip_c=25.0, rl_hip_c=25.0, max_other_c=25.0, error_code=code, body_height=0.07)

    chk("s4 sample, if mA: 29.41 V x 1.065 A", p_pack_if_ma(29.411165, -1065), 29.411165 * 1.065)
    rows = [row(i * 0.02, 1001) for i in range(500)] + \
           [row(10 + i * 0.02, 1002, cur=-4000, a=3.2) for i in range(500)]
    seg = segments(rows)
    chk("two segments", [c for c, _ in seg], [1001, 1002])
    chk("segment sizes", [len(r) for _, r in seg], [500, 500])
    s = seg_summary(*seg[1])
    chk("core excludes 3 s each edge", s["n_core"], sum(1 for r in seg[1][1] if 13 <= r["t_wall"] <= 19.98 - 3))
    chk("standing pack W", s["pack_w"], 29.4 * 4.0)
    chk("standing bus W", s["bus_w"], 29.4 * 3.2)
    # a 1-row spike at the edge must reach the PEAK but not the MEAN
    rows[500]["power_a_max"] = 20.0; rows[500]["power_v_min"] = 28.0
    s = seg_summary(*segments(rows)[1])
    chk("edge spike reaches peak", s["peak_bus_w"], 28.0 * 20.0)
    chk("edge spike not in mean", s["bus_w"], 29.4 * 3.2)
    chk("short segment has no core", seg_summary(1001, rows[:100])["n_core"], 0)
    chk("bms change count", seg_summary(1001, [row(i, 1001, cur=-1000 - (i // 2)) for i in range(20)])["bms_changes"],
        sum(1 for i in range(1, 20) if (i // 2) != ((i - 1) // 2) and 3 <= i <= 16))
    # --replay must run with NO ROS: a clean subprocess, rclpy deliberately unimportable (s7 lesson)
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "p.csv")
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=FIELDS); w.writeheader(); w.writerows(rows)
        os.makedirs(os.path.join(d, "rclpy")); open(os.path.join(d, "rclpy", "__init__.py"), "w").write("raise ImportError('no ROS')\n")
        env = dict(os.environ, PYTHONPATH=d)
        r = subprocess.run([sys.executable, os.path.abspath(__file__), "--replay", path],
                           capture_output=True, text=True, env=env)
        chk("--replay exits 0 with rclpy unimportable", r.returncode, 0)
        chk("--replay reports both postures", ("lying" in r.stdout, "stand-lock" in r.stdout), (True, True))
    print(f"{'ALL PASS' if not fails else 'FAILED: ' + ', '.join(fails)}")
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--replay", metavar="CSV")
    ap.add_argument("--out", metavar="CSV")
    ap.add_argument("--duration", type=float, default=3600.0)
    ap.add_argument("--interval", type=float, default=2.0, help="seconds between printed lines")
    ap.add_argument("--decimate", type=int, default=DECIMATE)
    args = ap.parse_args()
    if args.selftest:
        return run_selftest()
    if args.replay:
        report(load_csv(args.replay))
        return 0
    if not args.out:
        ap.error("--out is required to record")
    if os.path.exists(args.out):
        print(f"REFUSING: {args.out} exists -- never overwrite a capture.", file=sys.stderr)
        return 2
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from unitree_go.msg import LowState, SportModeState
    except Exception as e:
        print(f"ROS not available: {e}", file=sys.stderr)
        return 2

    f = open(args.out, "w", newline="")
    w = csv.DictWriter(f, fieldnames=FIELDS); w.writeheader()

    class Rec(Node):
        def __init__(self):
            super().__init__("go2_power_profile")
            q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=20)
            self.n = 0; self.code = -1; self.height = float("nan")
            self.amax = -math.inf; self.vmin = math.inf
            self.last = None; self.t_rx = None
            self.create_subscription(SportModeState, "/sportmodestate", self.sport_cb, q)
            self.create_subscription(LowState, "/lowstate", self.low_cb, q)
        def sport_cb(self, m):
            self.code = int(m.error_code); self.height = float(m.body_height)
        def low_cb(self, m):
            self.n += 1; self.t_rx = time.time()
            self.amax = max(self.amax, m.power_a); self.vmin = min(self.vmin, m.power_v)
            if self.n % args.decimate:
                return
            b = m.bms_state
            cells = [int(c) for c in b.cell_vol if c]
            temps = [m.motor_state[i].temperature for i in range(12)]
            r = dict(t_wall=self.t_rx, tick=int(m.tick), n_rx=self.n, power_v=float(m.power_v),
                     power_a=float(m.power_a), power_a_max=float(self.amax), power_v_min=float(self.vmin),
                     bms_current=int(b.current), bms_soc=int(b.soc), bms_status=int(b.status),
                     cell_sum_mv=sum(cells), n_cells=len(cells), bq_ntc0=int(b.bq_ntc[0]), bq_ntc1=int(b.bq_ntc[1]),
                     mcu_ntc0=int(b.mcu_ntc[0]), mcu_ntc1=int(b.mcu_ntc[1]),
                     ntc1=int(m.temperature_ntc1), ntc2=int(m.temperature_ntc2),
                     rr_hip_c=float(temps[RR_HIP]), rl_hip_c=float(temps[RL_HIP]),
                     max_other_c=float(max(t for i, t in enumerate(temps) if i not in (RR_HIP, RL_HIP))),
                     error_code=self.code, body_height=self.height)
            w.writerow(r); self.last = r
            self.amax = -math.inf; self.vmin = math.inf

    rclpy.init()
    node = Rec()
    t0 = time.time(); last_print = 0.0
    print(f"POWER PROFILE -- read-only. -> {args.out}   (pack W assumes mA, discharge-negative)")
    print(f"{'UTC':>8s} {'posture':>14s} {'V':>6s} {'bus W':>6s} {'bms raw':>8s} {'pack W?':>7s} "
          f"{'soc':>4s} {'cells V':>7s} {'RR C':>5s} {'RL C':>5s} {'h m':>6s}")
    try:
        while rclpy.ok() and time.time() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.time()
            if now - last_print < args.interval:
                continue
            last_print = now
            ts = time.strftime("%H:%M:%S", time.gmtime(now))
            if node.t_rx is None or now - node.t_rx > STALE_S or node.last is None:
                print(f"{ts:>8s} STALE -- no /lowstate for "
                      f"{'ever' if node.t_rx is None else f'{now - node.t_rx:.1f} s'}. Values UNKNOWN.")
                sys.stdout.flush(); continue
            r = node.last
            print(f"{ts:>8s} {POSTURE.get(r['error_code'], str(r['error_code'])):>14s} {r['power_v']:6.2f} "
                  f"{r['power_v'] * r['power_a']:6.1f} {r['bms_current']:8d} "
                  f"{p_pack_if_ma(r['power_v'], r['bms_current']):7.1f} {r['bms_soc']:4d} "
                  f"{r['cell_sum_mv'] / 1000:7.2f} {r['rr_hip_c']:5.0f} {r['rl_hip_c']:5.0f} {r['body_height']:6.3f}")
            sys.stdout.flush(); f.flush()
    except KeyboardInterrupt:
        pass
    f.close()
    print("-" * 78)
    report(load_csv(args.out))
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
