#!/usr/bin/env bash
# Friday dance demo, S4 rehearsal on loopback -- hexapod ROS 1 /go2/action -> gate8_bridge.sh with BRIDGE_YAML=dance_bridge.yaml
# (ros1_bridge, domain 0) -> go2_action_bridge -> FAKE Go2. NO ROBOT. Refuses with the Go2 cable in.
# Predictions PDL-0..PDL-7: printed by scripts/dance_loopback_analyse.py; plan Claude_Research friday_dance_demo_plan.md §3.4.
#   Melodic roscore on localhost = the hexapod master; `rostopic pub` in it = the S4 hexapod command.
#   dl_go2 = go2_humble:cyclone with go2_action_bridge built from the repo, started with the RUN SHEET's --params-file path.
#   The fake Go2 = test_action_bridge_integration_rosonly.py --fake ext (no trigger of its own; dances 3 s, answers at the end).
# Phases: C control (bridge on domain 42) | D dry, 10 Hz burst | O dry, `rostopic pub --once` | A armed vs fake, 10 Hz burst
#         | R a ROS 2 publish on /go2/action, echoed on ROS 1 (the bridge is bidirectional, gate 8 D2)
#   dance_loopback_test.sh [PHASES]     e.g. "C D"  (default: all five)
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
IFACE="${GO2_CABLE_IFACE:-enp46s0}"
PHASES="${1:-C D O A R}"
[ "$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo 0)" = 1 ] && { echo "REFUSING: $IFACE has carrier. Unplug the Go2 cable." >&2; exit 2; }
[ "$(cat /proc/sys/net/ipv4/ip_forward)" = 1 ] && { echo "REFUSING: ip_forward is 1." >&2; exit 2; }
ss -ltn 'sport = :11311' | grep -q LISTEN && { echo "REFUSING: something already listens on :11311." >&2; exit 2; }
for c in dl_core dl_go2 g8_ros1_bridge; do docker inspect "$c" >/dev/null 2>&1 && { echo "REFUSING: container $c exists." >&2; exit 2; }; done

OUT="$REPO/logs/dance/loopback/$(date -u +%Y%m%dT%H%M%SZ)"; mkdir -p "$OUT"; S="$OUT/summary.txt"
log() { echo "$*" | tee -a "$S"; }
fwd() { cat /proc/sys/net/ipv4/ip_forward; }
log "$(date -u +%FT%TZ) start; repo $(git -C "$REPO" rev-parse --short HEAD)$(git -C "$REPO" diff --quiet || echo +dirty); phases: $PHASES; ip_forward=$(fwd); $IFACE carrier=$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo none)"
R1=(-e ROS_MASTER_URI=http://localhost:11311 -e ROS_HOSTNAME=localhost -e PYTHONUNBUFFERED=1)
m1() { docker exec "${R1[@]}" dl_core bash -c "source /opt/ros/melodic/setup.bash && $1"; }
BS=""
cleanup() {
  [ -n "$BS" ] && kill -INT -- -"$BS" 2>/dev/null && sleep 3
  docker rm -f dl_core dl_go2 g8_ros1_bridge >/dev/null 2>&1
  echo "$(date -u +%FT%TZ) cleanup; ip_forward=$(fwd)" >> "$S"
}
trap cleanup EXIT

docker run -d --name dl_core --network host "${R1[@]}" ros:melodic-ros-base roscore >/dev/null
log "after docker run dl_core: ip_forward=$(fwd)"
for i in $(seq 30); do m1 'rosparam get /rosdistro' >/dev/null 2>&1 && break; sleep 1; done

# Built exactly as dance_go2_side.sh builds it (isolated install under /tmp/i), but on loopback Cyclone.
docker run -d --name dl_go2 --network host --ipc=host --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///config/cyclonedds_lo.xml -e PYTHONUNBUFFERED=1 \
  -v "$HOME/go2_ws":/ws:ro -v "$REPO/scripts":/scripts:ro -v "$REPO/config":/config:ro \
  -v "$REPO/ros2_ws/src/go2_action_bridge":/src/go2_action_bridge:ro -v "$OUT":/out \
  go2_humble:cyclone bash -c 'source /ws/install/setup.bash && mkdir -p /tmp/src && cp -r /src/go2_action_bridge /tmp/src/ \
    && colcon --log-base /tmp/l build --base-paths /tmp/src --build-base /tmp/b --install-base /tmp/i > /out/colcon.txt 2>&1 \
    && printf "set +u\nsource /ws/install/setup.bash\nsource /tmp/i/setup.bash\n" > /tmp/env.sh && touch /tmp/BUILT; exec sleep infinity' >/dev/null
log "after docker run dl_go2: ip_forward=$(fwd)"
for i in $(seq 120); do docker exec dl_go2 test -f /tmp/BUILT 2>/dev/null && break; sleep 1; done
docker exec dl_go2 test -f /tmp/BUILT || { log "FAIL: go2_action_bridge build did not finish (see colcon.txt)"; exit 1; }
docker exec dl_go2 bash -c 'env | grep -E "ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_LOCALHOST_ONLY" | sort' > "$OUT/env_go2.txt"

# phase NAME BRIDGE_DOMAIN ARMED SOURCE(burst|once|ros2pub)
phase() {
  local N=$1 DOM=$2 ARMED=$3 SRC=$4 P="$OUT/$1" RUN_S=24
  mkdir -p "$P"; log "--- phase $N: bridge domain $DOM, go2_action_bridge armed=$ARMED, source $SRC  [$(date -u +%TZ)]"
  GATE8_DOMAIN=$DOM ALLOW_DOMAIN_0=1 HEXAPOD_IP=127.0.0.1 CYCLONE_XML="$REPO/config/cyclonedds_lo.xml" \
    BRIDGE_YAML="$REPO/config/dance_bridge.yaml" GATE8_LOGROOT="$P/bridge_logs" \
    setsid "$REPO/scripts/gate8_bridge.sh" > "$P/bridge_stdout.txt" 2>&1 < /dev/null & BS=$!
  sleep 4
  log "  after bridge start: ip_forward=$(fwd)"
  docker exec dl_go2 bash -c "source /tmp/env.sh && python3 -u /scripts/test_action_bridge_integration_rosonly.py --fake ext /out/$N/fake.jsonl $RUN_S" \
    > "$P/fake_stdout.txt" 2>&1 & local FK=$!
  local NODE=""
  if [ "$SRC" != ros2pub ]; then
    sleep 1
    # The run sheet's terminal-B command (§3.4), params-file path and all; armed only as the phase says.
    docker exec dl_go2 bash -c "source /tmp/env.sh && exec ros2 run go2_action_bridge go2_action_bridge_node \
      --ros-args --params-file /tmp/i/go2_action_bridge/share/go2_action_bridge/config/go2_action_bridge.yaml \
      -p armed:=$ARMED -p lockout_s:=5.0" > "$P/node.txt" 2>&1 & NODE=$!
  fi
  m1 'exec rostopic echo -p /go2/action' > "$P/ros1_echo.csv" 2>&1 & local RE=$!
  sleep 6
  echo "$(date +%s.%N)" > "$P/t_source_start.txt"
  case $SRC in
    burst)   m1 "timeout -s INT 5 rostopic pub -r 10 /go2/action std_msgs/String 'data: dance1'" > "$P/source.txt" 2>&1 ;;
    once)    m1 "rostopic pub --once /go2/action std_msgs/String 'data: dance1'" > "$P/source.txt" 2>&1 ;;
    ros2pub) docker exec dl_go2 bash -c 'source /tmp/env.sh && ros2 topic pub -w 0 --times 5 -r 5 /go2/action std_msgs/msg/String "{data: from_ros2}"' \
               > "$P/source.txt" 2>&1 ;;
  esac
  echo "$(date +%s.%N)" > "$P/t_source_end.txt"
  m1 'rostopic info /go2/action' > "$P/rostopic_info.txt" 2>&1
  wait $FK
  if [ -n "$NODE" ]; then
    docker exec dl_go2 pkill -INT -f go2_action_bridge_node; wait $NODE 2>/dev/null
  fi
  docker exec dl_core pkill -INT -f 'rostopic echo'; wait $RE 2>/dev/null
  kill -INT -- -"$BS" 2>/dev/null; for i in $(seq 30); do kill -0 "$BS" 2>/dev/null || break; sleep 1; done
  kill -0 "$BS" 2>/dev/null && log "  WARNING: gate8_bridge.sh still running after 30 s"; BS=""
  m1 'rosparam get /rosdistro; rosparam list' > "$P/master_after.txt" 2>&1
  log "  after phase: ip_forward=$(fwd); gate8 params left: $(grep -c gate8 "$P/master_after.txt")"
}

for ph in $PHASES; do
  case $ph in
    C) phase C 42 false burst ;;
    D) phase D 0 false burst ;;
    O) phase O 0 false once ;;
    A) phase A 0 true burst ;;
    R) phase R 0 false ros2pub ;;
  esac
done

python3 "$REPO/scripts/dance_loopback_analyse.py" "$OUT" | tee -a "$S"
rc=${PIPESTATUS[0]}
log "$(date -u +%FT%TZ) end; ip_forward=$(fwd); $IFACE carrier=$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo none)"
exit $rc
