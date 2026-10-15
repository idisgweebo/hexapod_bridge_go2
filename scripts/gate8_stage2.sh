#!/usr/bin/env bash
# Gate 8 stage 2 -- the REAL hexapod as the source over dogspider, Go2 OFF. Measures what Wi-Fi does to the command stream.
#   Nano: `rostopic pub -r RATE /go2/cmd_vel` (+ a local `rostopic echo` = the published count, no Wi-Fi in it)
#     -> dogspider -> laptop: gate8_bridge.sh (domain 0, LOOPBACK DDS -- no Go2 in this stage)
#     -> g8_go2: fake Go2 + gate8_ros2_recorder.py + go2_cmd_bridge DRY (its StopMoves mid-stream = watchdog trips)
#   Laptop also runs a ROS 1 `rostopic echo` over Wi-Fi: gaps seen there AND at ROS 2 are Wi-Fi; only at ROS 2 = the bridge.
# ⛔ PUBLISHES on the hexapod's ROS 1 master (/go2/cmd_vel only) -- Doug's in-session go-ahead. Refuses if a hexapod node
#    subscribes to /go2/cmd_vel, if the Go2 cable has carrier, or if ip_forward = 1.
# Plan: Claude_Research gate8_hexapod_to_go2_plan.md §6.
#   gate8_stage2.sh [DURATION_S=300] [RATE_HZ=10] [VX=0.3]
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
DUR="${1:-300}"; RATE="${2:-10}"; VX="${3:-0.3}"
IFACE="${GO2_CABLE_IFACE:-enp46s0}"
NANO="${NANO:-xrrobot@192.168.50.50}"; NANO_IP="${NANO#*@}"
NANO_ROS="${NANO_ROS:-source /opt/ros/melodic/setup.bash && export ROS_MASTER_URI=http://localhost:11311 ROS_HOSTNAME=xrrobot-desktop}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=5 "$NANO")
refuse() { echo "REFUSING: $*" >&2; exit 2; }
[ "$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo 0)" = 1 ] && refuse "$IFACE has carrier. Stage 2 is Go2 OFF and cable out."
[ "$(cat /proc/sys/net/ipv4/ip_forward)" = 1 ] && refuse "ip_forward is 1."
for c in g8_go2 g8_ros1_bridge g8_echo; do docker inspect "$c" >/dev/null 2>&1 && refuse "container $c exists."; done
DEV="$(ip route get "$NANO_IP" | awk '{for(i=1;i<=NF;i++) if($i=="dev") print $(i+1)}')"
[ "$DEV" = wlhs0 ] || refuse "route to $NANO_IP is via '$DEV', not wlhs0 (hotspot down?)."
SELF="$(ip route get "$NANO_IP" | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}')"

OUT="$REPO/logs/gate8/stage2/$(date -u +%Y%m%dT%H%M%SZ)"; mkdir -p "$OUT"; S="$OUT/summary.txt"
log() { echo "$*" | tee -a "$S"; }
fwd() { cat /proc/sys/net/ipv4/ip_forward; }
log "$(date -u +%FT%T.%3NZ) stage 2 start; repo $(git -C "$REPO" rev-parse --short HEAD)$(git -C "$REPO" diff --quiet || echo +dirty); ${DUR}s @ ${RATE} Hz vx $VX; ip_forward=$(fwd); $IFACE carrier=$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo none)"

# --- Nano preflight (read-only). A hexapod node subscribing to /go2/cmd_vel would make this a hexapod command -> refuse.
"${SSH[@]}" "$NANO_ROS && date -u +%FT%T.%3NZ && rosparam get /rosdistro && rostopic info /go2/cmd_vel; echo '## power_save'; /sbin/iw dev wlan0 get power_save; /sbin/iw dev wlan0 link | grep -E 'SSID|freq|signal|bitrate'" \
  > "$OUT/nano_preflight.txt" 2>&1
log "nano preflight: $(head -1 "$OUT/nano_preflight.txt"); $(grep -i 'power save' "$OUT/nano_preflight.txt"); $(grep -E 'signal' "$OUT/nano_preflight.txt" | xargs)"
grep -q "melodic" "$OUT/nano_preflight.txt" || refuse "Nano ROS env not working -- see $OUT/nano_preflight.txt"
SUBS="$(sed -n '/Subscribers:/,$p' "$OUT/nano_preflight.txt" | grep -E '^ \* ' || true)"
[ -n "$SUBS" ] && refuse "something on the hexapod already subscribes to /go2/cmd_vel: $SUBS"

R1L=(--network host --add-host "xrrobot-desktop:$NANO_IP" -e ROS_MASTER_URI="http://$NANO_IP:11311" -e ROS_IP="$SELF" -e PYTHONUNBUFFERED=1)
BS=""; NE=""; NP=""
cleanup() {
  trap - EXIT INT TERM
  [ -n "$NP" ] && kill "$NP" 2>/dev/null
  "${SSH[@]}" "pkill -INT -f 'rostopic pub -r $RATE /go2/cmd_vel'; pkill -INT -f 'rostopic echo -p /go2/cmd_vel'" 2>/dev/null
  [ -n "$BS" ] && kill -INT -- -"$BS" 2>/dev/null && sleep 4
  docker rm -f g8_go2 g8_echo >/dev/null 2>&1
  "${SSH[@]}" "$NANO_ROS && rosnode list; rostopic list | grep -c go2; rosparam list | grep -c gate8" > "$OUT/nano_after.txt" 2>&1
  log "$(date -u +%FT%T.%3NZ) cleanup; ip_forward=$(fwd); nano after: $(tr '\n' ' ' < "$OUT/nano_after.txt" | cut -c1-300)"
}
trap cleanup EXIT INT TERM

# --- go2 side: fake + recorder + DRY go2_cmd_bridge, loopback, domain 0.
docker run -d --name g8_go2 --network host --ipc=host --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///config/cyclonedds_lo.xml -e PYTHONUNBUFFERED=1 \
  -v "$HOME/go2_ws":/ws:ro -v "$REPO/scripts":/scripts:ro -v "$REPO/config":/config:ro \
  -v "$REPO/ros2_ws/src/go2_cmd_bridge":/src/go2_cmd_bridge:ro -v "$OUT":/out \
  go2_humble:cyclone bash -c 'source /ws/install/setup.bash && mkdir -p /tmp/src && cp -r /src/go2_cmd_bridge /tmp/src/ \
    && colcon --log-base /tmp/l build --base-paths /tmp/src --build-base /tmp/b --install-base /tmp/i > /out/colcon.txt 2>&1 \
    && touch /tmp/BUILT; exec sleep infinity' >/dev/null
for i in $(seq 120); do docker exec g8_go2 test -f /tmp/BUILT 2>/dev/null && break; sleep 1; done
docker exec g8_go2 test -f /tmp/BUILT || { log "FAIL: build"; exit 1; }
log "after docker run g8_go2: ip_forward=$(fwd)"

GATE8_DOMAIN=0 ALLOW_DOMAIN_0=1 HEXAPOD_IP="$NANO_IP" CYCLONE_XML="$REPO/config/cyclonedds_lo.xml" GATE8_LOGROOT="$OUT/bridge_logs" \
  setsid "$REPO/scripts/gate8_bridge.sh" > "$OUT/bridge_stdout.txt" 2>&1 < /dev/null & BS=$!
sleep 5
log "after bridge start: ip_forward=$(fwd)"
docker exec g8_go2 /scripts/gate8_go2_side.sh /out/go2 false "$((DUR + 25))" > "$OUT/go2_side_stdout.txt" 2>&1 & GS=$!
docker run -d --name g8_echo "${R1L[@]}" bridge_humble:cmd_guard /ros1_env.sh rostopic echo -p /go2/cmd_vel >/dev/null
"${SSH[@]}" "$NANO_ROS && exec rostopic echo -p /go2/cmd_vel" > "$OUT/nano_echo.csv" 2> "$OUT/nano_echo.err" & NE=$!
sleep 8

log "$(date -u +%FT%T.%3NZ) SOURCE START on the Nano: rostopic pub -r $RATE /go2/cmd_vel [$VX,0,0] for ${DUR}s"
date +%s.%N > "$OUT/t_source_start.txt"
"${SSH[@]}" "$NANO_ROS && date -u +%FT%T.%3NZ && timeout -s INT $DUR rostopic pub -r $RATE /go2/cmd_vel geometry_msgs/Twist -- '[$VX, 0.0, 0.0]' '[0.0, 0.0, 0.0]'; date -u +%FT%T.%3NZ" \
  > "$OUT/nano_pub.txt" 2>&1 & NP=$!
wait $NP; NP=""
date +%s.%N > "$OUT/t_source_end.txt"
log "$(date -u +%FT%T.%3NZ) SOURCE END; nano_pub.txt: $(tr '\n' ' ' < "$OUT/nano_pub.txt" | cut -c1-200)"
sleep 3
"${SSH[@]}" "pkill -INT -f 'rostopic echo -p /go2/cmd_vel'" ; wait $NE 2>/dev/null; NE=""
docker kill -s INT g8_echo >/dev/null 2>&1; sleep 1; docker logs g8_echo > "$OUT/laptop_ros1_echo.csv" 2>&1
wait $GS
"${SSH[@]}" "/sbin/iw dev wlan0 link | grep -E 'signal|bitrate'; cat /proc/net/wireless" > "$OUT/nano_wifi_after.txt" 2>&1
python3 "$REPO/scripts/gate8_stage2_analyse.py" "$OUT" | tee -a "$S"
