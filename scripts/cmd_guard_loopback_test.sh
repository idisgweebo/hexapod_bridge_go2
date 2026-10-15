#!/usr/bin/env bash
# cmd_guard loopback test (15 Oct 2026). NO ROBOT. Everything on this machine's loopback:
#   Humble peer --(DDS, domain 42, Cyclone on lo)--> parameter_bridge (config/bridge.yaml) --(ROS 1)--> cmd_guard
#   --> /cmd_vel_ros2_safe, recorded with BOTH /cmd_vel_ros2_raw on a Melodic roscore container (stand-in for the hexapod)
#   by `rostopic echo -p` -- receipt times from one container, one clock.
# Usage: scripts/cmd_guard_loopback_test.sh [image] [peer_image]
#   image      default bridge_humble:cmd_guard (runs the bridge AND, via /ros1_env.sh, cmd_guard)
#   peer_image default go2_humble:cyclone (on the Orin: pass the bridge image itself)
# Predictions PCG-3..PCG-8: Claude_Research cmd_guard_humble_port.md. Pairs reference: PAIRS_REF=<print_pairs.txt>.
set -uo pipefail
IMG="${1:-bridge_humble:cmd_guard}"; PEER="${2:-go2_humble:cyclone}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
IFACE="${GO2_CABLE_IFACE:-enp46s0}"
PAIRS_REF="${PAIRS_REF:-$REPO/logs/bridge_humble/20261013T201920Z_armC/print_pairs.txt}"

if [ "$IFACE" != none ] && [ "$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo 0)" = "1" ]; then
  echo "REFUSING: $IFACE has carrier. Unplug the Go2 cable." >&2; exit 2; fi
if ss -ltn 'sport = :11311' | grep -q LISTEN; then
  echo "REFUSING: something already listens on :11311 -- not starting a second ROS master." >&2; exit 2; fi

STAMP=$(date -u +%Y%m%dT%H%M%SZ); OUT="$REPO/logs/bridge_humble/cmd_guard/${STAMP}_loopback"; mkdir -p "$OUT"
S="$OUT/summary.txt"
log() { echo "$*" | tee -a "$S"; }
log "$(date -u +%FT%TZ) image $IMG; peer $PEER; ip_forward=$(cat /proc/sys/net/ipv4/ip_forward); $IFACE no carrier"

cat > "$OUT/lo.xml" <<'X'
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General>
<Discovery><Peers><Peer address="127.0.0.1"/></Peers><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>20</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>
X
R1ENV=(-e ROS_MASTER_URI=http://localhost:11311 -e ROS_HOSTNAME=localhost -e PYTHONUNBUFFERED=1)
R2ENV=(-e PYTHONUNBUFFERED=1 -e ROS_DOMAIN_ID=42 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///out/lo.xml)
SRC2='source /opt/ros/humble/setup.bash && source /bridge_ws/install/setup.bash'
cleanup() { docker rm -f cg_roscore cg_bridge cg_guard cg_peer >/dev/null 2>&1; }
trap cleanup EXIT; cleanup

# PCG-3a: under the image's default (Humble-sourced) entrypoint, ROS 1's rospy must NOT import.
docker run --rm --network none "$IMG" python3 -c 'import rospy' > "$OUT/pcg3a_default_env_rospy.txt" 2>&1; rc=$?
log "PCG-3a default-env 'import rospy': rc=$rc ($([ $rc != 0 ] && echo 'fails, as predicted' || echo 'IMPORTS')) -- $(tail -1 "$OUT/pcg3a_default_env_rospy.txt")"
docker run --rm --network none "$IMG" /ros1_env.sh python3 -c 'import rospy, geometry_msgs.msg as g; print(rospy.__file__, g.__file__)' \
  > "$OUT/ros1_env_imports.txt" 2>&1; log "ros1_env.sh imports: rc=$? -- $(tail -1 "$OUT/ros1_env_imports.txt")"

# PCG-8: print-pairs vs laptop build 2.
docker run --rm --network none "$IMG" bash -c "$SRC2 && ros2 run ros1_bridge dynamic_bridge --print-pairs" > "$OUT/print_pairs.txt" 2>&1
pairs() { grep "^  - '" "$1" | sort; }
log "PCG-8 print-pairs: $(pairs "$OUT/print_pairs.txt" | wc -l) pairs; diff vs $(basename "$(dirname "$PAIRS_REF")"): $(diff <(pairs "$PAIRS_REF") <(pairs "$OUT/print_pairs.txt") | wc -l) lines"

# 1. ROS 1 master (Melodic) with the REAL bridge config.
docker run -d --name cg_roscore --network host "${R1ENV[@]}" -v "$OUT":/out -v "$REPO/config":/config:ro ros:melodic-ros-base roscore >/dev/null
for i in $(seq 30); do docker exec cg_roscore bash -c 'source /opt/ros/melodic/setup.bash && rostopic list' >/dev/null 2>&1 && break; sleep 1; done
docker exec cg_roscore bash -c 'source /opt/ros/melodic/setup.bash && rosparam load /config/bridge.yaml && rosparam get /topics' > "$OUT/rosparam.txt" 2>&1
log "roscore up after ${i}s; bridge.yaml: $(grep -c topic: "$OUT/rosparam.txt") topic(s)"

# 2. The bridge (Humble env) and 3. cmd_guard (ROS 1 env, same image).
docker run -d --name cg_bridge --network host --ipc=host "${R1ENV[@]}" "${R2ENV[@]}" -v "$OUT":/out "$IMG" \
  bash -c "$SRC2 && exec ros2 run ros1_bridge parameter_bridge" >/dev/null
docker run -d --name cg_guard --network host "${R1ENV[@]}" "$IMG" /ros1_env.sh roslaunch cmd_guard cmd_guard.launch >/dev/null
sleep 6

# 4. Recorders on the Melodic side.
rec() { docker exec "${R1ENV[@]}" cg_roscore bash -c "source /opt/ros/melodic/setup.bash && exec rostopic echo -p $1" > "$OUT/$2" 2>&1; }
rec /cmd_vel_ros2_safe safe.csv & RS=$!
rec /cmd_vel_ros2_raw raw.csv & RR=$!
sleep 3   # idle: cmd_guard should publish zeros before any input

# 5. Phases. Each: peer publishes 4 s at 10 Hz, is killed, then 3 s of silence (the watchdog window).
CLAMP='{linear: {x: 0.5, y: -0.5, z: 0.7}, angular: {x: 0.9, y: -0.9, z: 0.5}}'
PASS='{linear: {x: 0.0123, y: -0.0234, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0456}}'
for ph in CLAMP PASS; do
  docker run -d --name cg_peer --network host --ipc=host "${R2ENV[@]}" -v "$OUT":/out "$PEER" \
    bash -c "source /opt/ros/humble/setup.bash && exec ros2 topic pub -r 10 /cmd_vel_ros2_raw geometry_msgs/msg/Twist '${!ph}'" >/dev/null
  sleep 4; docker logs cg_peer > "$OUT/peer_$ph.txt" 2>&1; docker rm -f cg_peer >/dev/null; log "$(date -u +%FT%T.%3NZ) phase $ph peer killed"
  sleep 3
done
docker exec cg_roscore pkill -INT -f 'rostopic echo'; wait $RS $RR 2>/dev/null
docker logs cg_guard > "$OUT/guard.txt" 2>&1; docker logs cg_bridge > "$OUT/bridge.txt" 2>&1

# PCG-3b/c
log "PCG-3b guard: up line: $(grep -c 'cmd_guard up\.' "$OUT/guard.txt"); limits text: $(grep -o 'limits=([^)]*)' "$OUT/guard.txt" | head -1); in/out: $(grep -o 'in=[^ ]* out=[^ ]*' "$OUT/guard.txt" | head -1)"
log "PCG-3c bridge: ROS_DISTRO warnings: $(grep -c 'ROS_DISTRO was set' "$OUT/bridge.txt"); md5/checksum lines: $(grep -ciE 'md5|checksum' "$OUT/bridge.txt"); guard tracebacks: $(grep -c Traceback "$OUT/guard.txt")"

# PCG-4..7 from the two CSVs.
python3 - "$OUT" <<'PY' | tee -a "$S"
import csv, sys, statistics
out = sys.argv[1]
def load(name):
    rows = []
    with open(f"{out}/{name}") as f:
        for r in csv.reader(f):
            if not r or not r[0].isdigit(): continue
            rows.append((int(r[0]) / 1e9, tuple(r[1:7])))
    return rows
safe, raw = load("safe.csv"), load("raw.csv")
print(f"rows: safe {len(safe)}, raw {len(raw)}")
CL_IN, PA_IN = "0.5", "0.0123"
CL_EXP = ("0.03", "-0.03", "0.0", "0.0", "0.0", "0.1")
PA_EXP = ("0.0123", "-0.0234", "0.0", "0.0", "0.0", "0.0456")
ZERO = ("0.0",) * 6
print(f"safe rows by value: clamp {sum(v == CL_EXP for _, v in safe)}, pass {sum(v == PA_EXP for _, v in safe)}, "
      f"zero {sum(v == ZERO for _, v in safe)}, OTHER {sum(v not in (CL_EXP, PA_EXP, ZERO) for _, v in safe)}")
t_first_raw = raw[0][0] if raw else None
idle = [v for t, v in safe if t_first_raw is None or t < t_first_raw]
print(f"idle before first input: {len(idle)} rows, all zero: {all(v == ZERO for v in idle)}")
# Each phase is scored ONLY on its own window: first raw input of the phase -> first raw input of the next phase (or end).
# (Fake-data test, 15 Oct: a capture-wide search attributed a later phase's rows to this one.)
for pid, name, x_in, exp in (("PCG-4", "CLAMP", CL_IN, CL_EXP), ("PCG-5", "PASS", PA_IN, PA_EXP)):
    ph = [t for t, v in raw if v[0] == x_in]
    if not ph: print(f"{pid}/PCG-6 {name}: no raw rows -- FAIL"); continue
    last_in = ph[-1]
    nxt = next((t for t, v in raw if t > last_in and v[0] != x_in), None)
    win = [(t, v) for t, v in safe if t >= ph[0] and (nxt is None or t < nxt)]
    bad = sorted(set(v for _, v in win if v not in (exp, ZERO)))
    print(f"{pid} {name}: {'PASS' if any(v == exp for _, v in win) and not bad else 'FAIL'} -- {sum(v == exp for _, v in win)} rows "
          f"== {exp}; other values in window: {len(bad)} {bad[:3]}")
    after = [(t, v) for t, v in win if t > last_in]
    nz = [t for t, v in after if v != ZERO]
    z = next((t for t, v in after if v == ZERO), None)
    window = [(t, v) for t, v in after if z is not None and t >= z]
    held = all(v == ZERO for _, v in window)
    print(f"PCG-6 {name}: raw msgs {len(ph)}; last input -> last non-zero out {((nz[-1] - last_in) if nz else float('nan')):.3f} s; "
          f"-> FIRST ZERO out {(z - last_in) if z else float('nan'):.3f} s (band 0.40-0.50); zeros after: {len(window)} rows, all zero {held}")
ts = [t for t, _ in safe]
if len(ts) > 2:
    dt = [b - a for a, b in zip(ts, ts[1:])]
    print(f"PCG-7 rate: {(len(ts) - 1) / (ts[-1] - ts[0]):.3f} Hz over {ts[-1] - ts[0]:.2f} s (band 19.5-20.5); "
          f"interval median {statistics.median(dt) * 1e3:.1f} ms, max {max(dt) * 1e3:.1f} ms")
PY
log "$(date -u +%FT%TZ) end; ip_forward=$(cat /proc/sys/net/ipv4/ip_forward)"
