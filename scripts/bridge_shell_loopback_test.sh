#!/usr/bin/env bash
# bridge_shell.sh + pub_test.sh, end to end on loopback (15 Oct 2026). NO ROBOT.
#   Melodic roscore on localhost stands in for the hexapod; HEXAPOD_IP=127.0.0.1; bridge_shell.sh runs in its own process
#   group and is stopped by SIGINT to the group (no tty: the interactive -it/Ctrl+C path is NOT exercised here).
# Predictions PBS-1..PBS-4: Claude_Research cmd_guard_humble_port.md section 7.
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
IFACE="${GO2_CABLE_IFACE:-enp46s0}"
if [ "$IFACE" != none ] && [ "$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo 0)" = "1" ]; then
  echo "REFUSING: $IFACE has carrier. Unplug the Go2 cable." >&2; exit 2; fi
if ss -ltn 'sport = :11311' | grep -q LISTEN; then echo "REFUSING: something already listens on :11311." >&2; exit 2; fi
if docker inspect hexapod_ros1_bridge hexapod_cmd_guard >/dev/null 2>&1; then echo "REFUSING: bridge containers already exist." >&2; exit 2; fi

OUT="$REPO/logs/bridge_humble/cmd_guard/$(date -u +%Y%m%dT%H%M%SZ)_bridge_shell"; mkdir -p "$OUT"; S="$OUT/summary.txt"
log() { echo "$*" | tee -a "$S"; }
log "$(date -u +%FT%TZ) start; repo $(git -C "$REPO" rev-parse --short HEAD); ip_forward=$(cat /proc/sys/net/ipv4/ip_forward)"
R1=(-e ROS_MASTER_URI=http://localhost:11311 -e ROS_HOSTNAME=localhost -e PYTHONUNBUFFERED=1)
m1() { docker exec "${R1[@]}" bs_core bash -c "source /opt/ros/melodic/setup.bash && $1"; }
cleanup() { docker rm -f bs_core >/dev/null 2>&1; [ -n "${BS:-}" ] && kill -INT -- -"$BS" 2>/dev/null; }
trap cleanup EXIT

docker run -d --name bs_core --network host "${R1[@]}" ros:melodic-ros-base roscore >/dev/null
for i in $(seq 30); do m1 'rosparam get /rosdistro' >/dev/null 2>&1 && break; sleep 1; done

# Start bridge_shell.sh in its own session/process group.
HEXAPOD_IP=127.0.0.1 setsid "$REPO/scripts/bridge_shell.sh" > "$OUT/bridge_shell_stdout.txt" 2>&1 < /dev/null & BS=$!
sleep 8

# PBS-1
m1 'rosnode list' > "$OUT/rosnode_list.txt" 2>&1
log "PBS-1 nodes: $(tr '\n' ' ' < "$OUT/rosnode_list.txt")| rosdistro=$(m1 'rosparam get /rosdistro' | tr -d '\n ') | guard out=$(m1 'rosparam get /cmd_guard/output_topic' | tr -d '\n')"

# Recorders: the dead-end sink AND the real twist_mux input (which must stay silent / absent).
m1 'exec rostopic echo -p /cmd_vel_safety_test_out' > "$OUT/sink.csv" 2>&1 & RS=$!
m1 'exec rostopic echo -p /cmd_vel_ros2_safe' > "$OUT/mux_input.csv" 2>&1 & RM=$!
sleep 2

# PBS-3 first half: domain 0 refused before any container starts.
ROS_DOMAIN_ID=0 "$REPO/scripts/pub_test.sh" 0.5 -0.5 0.5 3 > "$OUT/pub_domain0.txt" 2>&1 < /dev/null
log "PBS-3a pub_test.sh ROS_DOMAIN_ID=0: rc=$? -- $(head -1 "$OUT/pub_domain0.txt")"
# Phase W (wrong domain, the control), then phase R (right domain), each followed by 3 s of silence.
T_W0=$(date +%s.%N); ROS_DOMAIN_ID=43 "$REPO/scripts/pub_test.sh" 0.5 -0.5 0.5 3 > "$OUT/pub_domain43.txt" 2>&1 < /dev/null; T_W1=$(date +%s.%N)
sleep 3
T_R0=$(date +%s.%N); "$REPO/scripts/pub_test.sh" 0.5 -0.5 0.5 3 > "$OUT/pub_domain42.txt" 2>&1 < /dev/null; T_R1=$(date +%s.%N)
sleep 3
docker exec bs_core pkill -INT -f 'rostopic echo'; wait $RS $RM 2>/dev/null

# PBS-4: stop bridge_shell.sh as Ctrl+C would (SIGINT to its process group), then check it cleaned up.
kill -INT -- -"$BS"; for i in $(seq 30); do kill -0 "$BS" 2>/dev/null || break; sleep 1; done
STILL=$(kill -0 "$BS" 2>/dev/null && echo RUNNING || echo exited); BS=""
LD=$(grep -o 'logs in .*' "$OUT/bridge_shell_stdout.txt" | awk '{print $3}')
log "PBS-4 bridge_shell.sh $STILL after ${i}s; containers left: $(docker ps -a --format '{{.Names}}' | grep -cE '^hexapod_(ros1_bridge|cmd_guard)$'); logdir: ${LD:-NONE} -> $(ls "${LD:-/nonexistent}" 2>/dev/null | tr '\n' ' '); cmd_guard up lines: $(grep -c 'cmd_guard up' "${LD:-/x}/cmd_guard.txt" 2>/dev/null)"
[ -n "${LD:-}" ] && cp -a "$LD" "$OUT/bridge_shell_logdir"

python3 - "$OUT" "$T_W0" "$T_W1" "$T_R0" "$T_R1" <<'PY' | tee -a "$S"
import csv, sys
out, tw0, tw1, tr0, tr1 = sys.argv[1], *map(float, sys.argv[2:])
def load(n):
    rows = []
    for r in csv.reader(open(f"{out}/{n}")):
        if r and r[0].isdigit(): rows.append((int(r[0]) / 1e9, tuple(r[1:7])))
    return rows
sink, mux = load("sink.csv"), load("mux_input.csv")
Z = ("0.0",) * 6; CL = ("0.03", "-0.03", "0.0", "0.0", "0.0", "0.1")
w = [v for t, v in sink if tw0 <= t <= tw1 + 1.0]
r = [v for t, v in sink if tr0 <= t <= tr1 + 1.0]
other = sorted(set(v for _, v in sink if v not in (Z, CL)))
print(f"sink rows {len(sink)}; OTHER values {len(other)} {other[:3]}")
print(f"PBS-3b domain 43 window: {len(w)} rows, non-zero {sum(v != Z for v in w)} -> {'PASS' if w and all(v == Z for v in w) else 'FAIL'}")
print(f"PBS-2 domain 42 window: {len(r)} rows, clamp {sum(v == CL for v in r)}, zero {sum(v == Z for v in r)} -> "
      f"{'PASS' if any(v == CL for v in r) and not other else 'FAIL'}; last rows zero: {all(v == Z for v in r[-5:]) if r else None}")
print(f"PBS-2 /cmd_vel_ros2_safe (mux input) rows: {len(mux)} -> {'PASS (silent)' if not mux else 'FAIL'}")
PY
log "$(date -u +%FT%TZ) end; ip_forward=$(cat /proc/sys/net/ipv4/ip_forward)"
