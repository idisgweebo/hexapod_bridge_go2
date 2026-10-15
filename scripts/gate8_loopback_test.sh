#!/usr/bin/env bash
# Gate 8 stage 1 -- loopback rehearsal of hexapod ROS 1 /go2/cmd_vel -> gate8_bridge.sh (ros1_bridge, domain 0) -> go2_cmd_bridge -> FAKE Go2.
# NO ROBOT. Refuses with the Go2 cable in. Predictions PG8-0..PG8-11: Claude_Research gate8_hexapod_to_go2_plan.md §4.
#   Melodic roscore on localhost = the hexapod master; `rostopic pub -r 10` in it = the hexapod source (the stage 2/3 command).
#   gate8_bridge.sh is the stage-3 launcher with HEXAPOD_IP=127.0.0.1 and CYCLONE_XML=config/cyclonedds_lo.xml.
#   g8_go2 = go2_humble:cyclone with go2_cmd_bridge built from the repo; each phase runs gate8_go2_side.sh (fake + recorder + node).
# Phases: C control (bridge on domain 42) | D dry | X clamp | A armed vs fake | K bridge SIGKILLed mid-stream | R ROS 2 -> ROS 1 echo.
#   gate8_loopback_test.sh [PHASES]     e.g. "C D"  (default: all six)
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
IFACE="${GO2_CABLE_IFACE:-enp46s0}"
PHASES="${1:-C D X A K R}"
[ "$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo 0)" = 1 ] && { echo "REFUSING: $IFACE has carrier. Unplug the Go2 cable." >&2; exit 2; }
[ "$(cat /proc/sys/net/ipv4/ip_forward)" = 1 ] && { echo "REFUSING: ip_forward is 1." >&2; exit 2; }
ss -ltn 'sport = :11311' | grep -q LISTEN && { echo "REFUSING: something already listens on :11311." >&2; exit 2; }
for c in g8_core g8_go2 g8_ros1_bridge; do docker inspect "$c" >/dev/null 2>&1 && { echo "REFUSING: container $c exists." >&2; exit 2; }; done

OUT="$REPO/logs/gate8/stage1/$(date -u +%Y%m%dT%H%M%SZ)"; mkdir -p "$OUT"; S="$OUT/summary.txt"
log() { echo "$*" | tee -a "$S"; }
fwd() { cat /proc/sys/net/ipv4/ip_forward; }
log "$(date -u +%FT%TZ) start; repo $(git -C "$REPO" rev-parse --short HEAD)$(git -C "$REPO" diff --quiet || echo +dirty); phases: $PHASES; ip_forward=$(fwd); $IFACE carrier=$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo none)"
R1=(-e ROS_MASTER_URI=http://localhost:11311 -e ROS_HOSTNAME=localhost -e PYTHONUNBUFFERED=1)
m1() { docker exec "${R1[@]}" g8_core bash -c "source /opt/ros/melodic/setup.bash && $1"; }
BS=""
cleanup() {
  [ -n "$BS" ] && kill -INT -- -"$BS" 2>/dev/null && sleep 3
  docker rm -f g8_core g8_go2 g8_ros1_bridge >/dev/null 2>&1
  echo "$(date -u +%FT%TZ) cleanup; ip_forward=$(fwd)" >> "$S"
}
trap cleanup EXIT

docker run -d --name g8_core --network host "${R1[@]}" ros:melodic-ros-base roscore >/dev/null
log "after docker run g8_core: ip_forward=$(fwd)"
for i in $(seq 30); do m1 'rosparam get /rosdistro' >/dev/null 2>&1 && break; sleep 1; done

docker run -d --name g8_go2 --network host --ipc=host --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///config/cyclonedds_lo.xml -e PYTHONUNBUFFERED=1 \
  -v "$HOME/go2_ws":/ws:ro -v "$REPO/scripts":/scripts:ro -v "$REPO/config":/config:ro \
  -v "$REPO/ros2_ws/src/go2_cmd_bridge":/src/go2_cmd_bridge:ro -v "$OUT":/out \
  go2_humble:cyclone bash -c 'source /ws/install/setup.bash && mkdir -p /tmp/src && cp -r /src/go2_cmd_bridge /tmp/src/ \
    && colcon --log-base /tmp/l build --base-paths /tmp/src --build-base /tmp/b --install-base /tmp/i > /out/colcon.txt 2>&1 \
    && touch /tmp/BUILT; exec sleep infinity' >/dev/null
log "after docker run g8_go2: ip_forward=$(fwd)"
for i in $(seq 120); do docker exec g8_go2 test -f /tmp/BUILT 2>/dev/null && break; sleep 1; done
docker exec g8_go2 test -f /tmp/BUILT || { log "FAIL: go2_cmd_bridge build did not finish (see colcon.txt)"; exit 1; }

# phase NAME DOMAIN ARMED VX WZ STREAM_S EVENT(none|kill|ros2pub)
phase() {
  local N=$1 DOM=$2 ARMED=$3 VX=$4 WZ=$5 DUR=$6 EV=$7 P="$OUT/$1" RUN_S=17
  mkdir -p "$P"; log "--- phase $N: bridge domain $DOM, go2_cmd_bridge armed=$ARMED, source ($VX, $WZ) ${DUR}s, event $EV  [$(date -u +%TZ)]"
  GATE8_DOMAIN=$DOM ALLOW_DOMAIN_0=1 HEXAPOD_IP=127.0.0.1 CYCLONE_XML="$REPO/config/cyclonedds_lo.xml" GATE8_LOGROOT="$P/bridge_logs" \
    setsid "$REPO/scripts/gate8_bridge.sh" > "$P/bridge_stdout.txt" 2>&1 < /dev/null & BS=$!
  sleep 4
  log "  after bridge start: ip_forward=$(fwd)"
  docker exec g8_go2 /scripts/gate8_go2_side.sh "/out/$N" "$ARMED" "$RUN_S" > "$P/go2_side_stdout.txt" 2>&1 & local GS=$!
  m1 'exec rostopic echo -p /go2/cmd_vel' > "$P/ros1_echo.csv" 2>&1 & local RE=$!
  sleep 7
  if [ "$EV" = ros2pub ]; then
    echo "$(date +%s.%N)" > "$P/t_ros2pub.txt"
    docker exec g8_go2 bash -c 'source /ws/install/setup.bash && ros2 topic pub -w 1 --times 5 -r 5 /go2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"' \
      > "$P/ros2_pub.txt" 2>&1
  elif [ "$DUR" != 0 ]; then
    echo "$(date +%s.%N)" > "$P/t_source_start.txt"
    m1 "timeout -s INT $DUR rostopic pub -r 10 /go2/cmd_vel geometry_msgs/Twist -- '[$VX, 0.0, 0.0]' '[0.0, 0.0, $WZ]'" > "$P/source.txt" 2>&1 & local SP=$!
    if [ "$EV" = kill ]; then
      sleep 2.5; echo "$(date +%s.%N)" > "$P/t_kill.txt"; docker kill g8_ros1_bridge > "$P/kill.txt" 2>&1
    fi
    [ "$N" = D ] && { sleep 1.2; m1 'rostopic info /go2/cmd_vel' > "$P/rostopic_info.txt" 2>&1; }
    wait $SP; echo "$(date +%s.%N)" > "$P/t_source_end.txt"
  fi
  wait $GS
  docker exec g8_core pkill -INT -f 'rostopic echo'; wait $RE 2>/dev/null
  kill -INT -- -"$BS" 2>/dev/null; for i in $(seq 30); do kill -0 "$BS" 2>/dev/null || break; sleep 1; done
  kill -0 "$BS" 2>/dev/null && log "  WARNING: gate8_bridge.sh still running after 30 s"; BS=""
  m1 'rosparam get /rosdistro; rosparam list' > "$P/master_after.txt" 2>&1
  log "  after phase: ip_forward=$(fwd); /rosdistro=$(head -1 "$P/master_after.txt"); gate8 params left: $(grep -c gate8 "$P/master_after.txt")"
}

for ph in $PHASES; do
  case $ph in
    C) phase C 42 false 0.3 0.0 3 none ;;
    D) phase D 0 false 0.3 0.0 3 none ;;
    X) phase X 0 false 0.9 3.0 3 none ;;
    A) phase A 0 true 0.3 0.0 3 none ;;
    K) phase K 0 true 0.3 0.0 5 kill ;;
    R) phase R 0 false 0 0 0 ros2pub ;;
  esac
done

python3 "$REPO/scripts/gate8_stage1_analyse.py" "$OUT" | tee -a "$S"
log "$(date -u +%FT%TZ) end; ip_forward=$(fwd); $IFACE carrier=$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo none)"
