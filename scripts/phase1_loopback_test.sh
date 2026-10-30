#!/usr/bin/env bash
# Phase 1 STAGE 2 on loopback -- the Go2 -> hexapod feedback chain through the REAL ros1_bridge. NO ROBOT. Refuses with the Go2 cable in.
#   hexapod side: Melodic roscore on localhost = the hexapod master; hexapod/go2_feedback_check.py (Python 2.7) = the hexapod checker;
#                 scripts/phase1_ros1_probe.py records the ROS 1 side (and injects PFC-0's ROS 1 -> ROS 2 messages in phase X).
#   bridge:       scripts/gate8_bridge.sh with BRIDGE_YAML=config/phase1_bridge.yaml, loopback Cyclone, domain 0.
#   Go2 side:     go2_humble:cyclone with go2_cmd_bridge (DRY, input_type twist_stamped) and go2_adapter's go2_state_node built from the
#                 repo and started by `ros2 run`; scripts/fake_go2_phase1.py = the robot (body follows the DRY request log) + ROS 2 recorder.
# Expectations: registered in Claude_Research phase1_go2_feedback_plan.md §9 "Stage 2" BEFORE the first run; scored by
#   scripts/phase1_loopback_analyse.py.
# Phases: C control (bridge on domain 42) | X PFC-0/PFC-1 injections | S1..S7, S8a, S8b, S8c, S9 (plan §6)
#   phase1_loopback_test.sh [PHASES]     e.g. "C S3"  (default: all)
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd -P)"
IFACE="${GO2_CABLE_IFACE:-enp46s0}"
PHASES="${1:-C X S1 S2 S3 S4 S5 S6 S7 S8a S8b S8c S9}"
[ "$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo 0)" = 1 ] && { echo "REFUSING: $IFACE has carrier. Unplug the Go2 cable." >&2; exit 2; }
[ "$(cat /proc/sys/net/ipv4/ip_forward)" = 1 ] && { echo "REFUSING: ip_forward is 1." >&2; exit 2; }
ss -ltn 'sport = :11311' | grep -q LISTEN && { echo "REFUSING: something already listens on :11311." >&2; exit 2; }
for c in p1_core p1_go2 g8_ros1_bridge; do docker inspect "$c" >/dev/null 2>&1 && { echo "REFUSING: container $c exists." >&2; exit 2; }; done

OUT="$REPO/logs/phase1/loopback/$(date -u +%Y%m%dT%H%M%SZ)"; mkdir -p "$OUT"; S="$OUT/summary.txt"
log() { echo "$*" | tee -a "$S"; }
fwd() { cat /proc/sys/net/ipv4/ip_forward; }
log "$(date -u +%FT%TZ) start; repo $(git -C "$REPO" rev-parse --short HEAD)$(git -C "$REPO" diff --quiet || echo +dirty); phases: $PHASES; ip_forward=$(fwd); $IFACE carrier=$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo none)"
U=(--user "$(id -u):$(id -g)" -e HOME=/tmp)          # logs stay Doug's, not root's
R1=(-e ROS_MASTER_URI=http://localhost:11311 -e ROS_HOSTNAME=localhost -e PYTHONUNBUFFERED=1 -e ROS_LOG_DIR=/tmp/roslog)
m1() { docker exec "${R1[@]}" p1_core bash -c "source /opt/ros/melodic/setup.bash && $1"; }
g2() { docker exec p1_go2 bash -c "source /tmp/env.sh && $1"; }
BS=""
cleanup() {
  [ -n "$BS" ] && kill -INT -- -"$BS" 2>/dev/null && sleep 3
  docker rm -f p1_core p1_go2 g8_ros1_bridge >/dev/null 2>&1
  echo "$(date -u +%FT%TZ) cleanup; ip_forward=$(fwd)" >> "$S"
}
trap cleanup EXIT

docker run -d --name p1_core --network host "${U[@]}" "${R1[@]}" -v "$REPO/hexapod":/r/hexapod:ro -v "$REPO/scripts":/r/scripts:ro \
  -v "$OUT":/out ros:melodic-ros-base roscore >/dev/null
log "after docker run p1_core: ip_forward=$(fwd)"
for i in $(seq 30); do m1 'rosparam get /rosdistro' >/dev/null 2>&1 && break; sleep 1; done
m1 'python --version 2>&1; rosversion -d' > "$OUT/ros1_versions.txt" 2>&1

# go2_cmd_bridge + go2_adapter, built as dance_loopback_test.sh builds go2_action_bridge (isolated install under /tmp/i).
docker run -d --name p1_go2 --network host --ipc=host "${U[@]}" \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///config/cyclonedds_lo.xml -e PYTHONUNBUFFERED=1 \
  -v "$HOME/go2_ws":/ws:ro -v "$REPO/scripts":/scripts:ro -v "$REPO/config":/config:ro \
  -v "$REPO/ros2_ws/src/go2_cmd_bridge":/src/go2_cmd_bridge:ro -v "$REPO/ros2_ws/src/go2_adapter":/src/go2_adapter:ro -v "$OUT":/out \
  go2_humble:cyclone bash -c 'source /ws/install/setup.bash && mkdir -p /tmp/src && cp -r /src/go2_cmd_bridge /src/go2_adapter /tmp/src/ \
    && colcon --log-base /tmp/l build --base-paths /tmp/src --build-base /tmp/b --install-base /tmp/i > /out/colcon.txt 2>&1 \
    && printf "set +u\nsource /ws/install/setup.bash\nsource /tmp/i/setup.bash\n" > /tmp/env.sh && touch /tmp/BUILT; exec sleep infinity' >/dev/null
log "after docker run p1_go2: ip_forward=$(fwd)"
for i in $(seq 180); do docker exec p1_go2 test -f /tmp/BUILT 2>/dev/null && break; sleep 1; done
docker exec p1_go2 test -f /tmp/BUILT || { log "FAIL: build did not finish (see colcon.txt)"; exit 1; }
docker exec p1_go2 bash -c 'env | grep -E "ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_LOCALHOST_ONLY" | sort' > "$OUT/env_go2.txt"

WARM_S=9          # the state node's clock estimators (100 samples), ros1_bridge discovery, the bridge going IDLE

# phase NAME BRIDGE_DOMAIN FAKE_SCENARIO HEX   (HEX: listen:SECONDS | send:PROFILE | inject | kill:PROFILE)
phase() {
  local N=$1 DOM=$2 FAKE=$3 HEX=$4 P="$OUT/$1"
  mkdir -p "$P"; log "--- phase $N: bridge domain $DOM, fake $FAKE, hexapod $HEX  [$(date -u +%TZ)]"
  docker exec p1_go2 bash -c "source /tmp/env.sh && exec python3 -u /scripts/fake_go2_phase1.py $FAKE 120 /out/$N/ros2.jsonl" \
    > "$P/fake.txt" 2>&1 & local FK=$!
  g2 "exec ros2 run go2_cmd_bridge go2_cmd_bridge_node --ros-args -p armed:=false -p input_type:=twist_stamped" > "$P/cmd_bridge.txt" 2>&1 &
  local CB=$!
  g2 "exec ros2 run go2_adapter go2_state_node" > "$P/state_node.txt" 2>&1 & local SN=$!
  GATE8_DOMAIN=$DOM ALLOW_DOMAIN_0=1 HEXAPOD_IP=127.0.0.1 CYCLONE_XML="$REPO/config/cyclonedds_lo.xml" \
    BRIDGE_YAML="$REPO/config/phase1_bridge.yaml" GATE8_LOGROOT="$P/bridge_logs" \
    setsid "$REPO/scripts/gate8_bridge.sh" > "$P/bridge_stdout.txt" 2>&1 < /dev/null & BS=$!
  local PMODE=listen; [ "$HEX" = inject ] && PMODE=inject
  m1 "exec python /r/scripts/phase1_ros1_probe.py $PMODE 120 /out/$N/ros1.jsonl" > "$P/probe.txt" 2>&1 & local PR=$!
  sleep "$WARM_S"
  log "  after warm-up: ip_forward=$(fwd)"
  echo "$(date +%s.%N)" > "$P/t_hex_start.txt"
  local CHK="exec python /r/hexapod/go2_feedback_check.py --outdir /out/$N/check"
  case $HEX in
    listen:*) m1 "$CHK --duration ${HEX#listen:}" > "$P/check.txt" 2>&1; echo "rc=$?" >> "$P/check.txt" ;;
    send:*)   m1 "$CHK --send ${HEX#send:} --expect-armed false --duration 6" > "$P/check.txt" 2>&1; echo "rc=$?" >> "$P/check.txt" ;;
    kill:*)   m1 "$CHK --send ${HEX#kill:} --expect-armed false --duration 6" > "$P/check.txt" 2>&1 & local CK=$!
              for i in $(seq 300); do [ "$(wc -l 2>/dev/null < "$P/check/sent.csv" || echo 0)" -ge 16 ] && break; sleep 0.02; done
              echo "$(date +%s.%N)" > "$P/t_kill_before.txt"
              docker kill g8_ros1_bridge > "$P/kill.txt" 2>&1
              echo "$(date +%s.%N)" > "$P/t_kill_after.txt"
              log "  ros1_bridge SIGKILLed after $(($(wc -l < "$P/check/sent.csv") - 1)) sends"
              wait $CK; echo "rc=$?" >> "$P/check.txt" ;;
    inject)   sleep 13 ;;          # the probe injects at t_0 + 14..18 s; the fake publishes at 14..16 s
  esac
  echo "$(date +%s.%N)" > "$P/t_hex_end.txt"
  sleep 1
  m1 'rostopic info /go2/robot_status; rostopic info /go2/cmd_vel_stamped' > "$P/rostopic_info.txt" 2>&1
  docker exec p1_core pkill -INT -f phase1_ros1_probe; wait $PR 2>/dev/null
  kill -INT -- -"$BS" 2>/dev/null; for i in $(seq 30); do kill -0 "$BS" 2>/dev/null || break; sleep 1; done
  kill -0 "$BS" 2>/dev/null && log "  WARNING: gate8_bridge.sh still running after 30 s"; BS=""
  docker exec p1_go2 pkill -INT -f go2_state_node; docker exec p1_go2 pkill -INT -f go2_cmd_bridge_node
  wait $SN $CB 2>/dev/null
  docker exec p1_go2 pkill -INT -f fake_go2_phase1; wait $FK 2>/dev/null
  m1 'rosparam list' > "$P/master_after.txt" 2>&1
  log "  after phase: ip_forward=$(fwd); gate8 params left: $(grep -c gate8 "$P/master_after.txt"); $(tail -1 "$P/check.txt" 2>/dev/null)"
}

for ph in $PHASES; do
  case $ph in
    C)   phase C   42 healthy      send:zero ;;
    X)   phase X   0  pfc0         inject ;;
    S1)  phase S1  0  lying        listen:8 ;;
    S2)  phase S2  0  healthy      send:zero ;;
    S3)  phase S3  0  healthy      send:walk_03 ;;
    S4)  phase S4  0  healthy      send:turn_10 ;;
    S5)  phase S5  0  healthy      send:walk_03_silent ;;
    S6)  phase S6  0  healthy      kill:walk_03 ;;
    S7)  phase S7  0  handset      send:walk_03 ;;
    S8a) phase S8a 0  mode0        listen:10 ;;
    S8b) phase S8b 0  hip51        send:walk_03 ;;
    S8c) phase S8c 0  sport_freeze send:walk_03 ;;
    S9)  phase S9  0  healthy      send:replay ;;
    *)   log "unknown phase $ph" ;;
  esac
done

python3 "$REPO/scripts/phase1_loopback_analyse.py" "$OUT" | tee -a "$S"
rc=${PIPESTATUS[0]}
log "$(date -u +%FT%TZ) end; ip_forward=$(fwd); $IFACE carrier=$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo none)"
exit $rc
