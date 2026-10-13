#!/usr/bin/env bash
# ros1_bridge loopback test (13 Oct 2026). NO ROBOT. Everything on this machine's loopback:
#   ROS 1: a Melodic (ros-base: has tf2_msgs) roscore on localhost:11311 (ROS_HOSTNAME=localhost) -- stands in for the hexapod.
#   ROS 2: domain 42 (the Orin's), Cyclone pinned to lo, multicast off.
# Usage: scripts/bridge_loopback_test.sh <arm> <bridge_image> <peer_image>
#   A  go2_foxy:cyclone    go2_foxy:cyclone     harness control (Foxy<->Foxy, as 11 Sep)
#   B  go2_foxy:cyclone    go2_humble:cyclone   the two-container fallback: Foxy bridge, Humble peer
#   C  bridge_humble:exp   go2_humble:cyclone   the candidate: Humble throughout
# Checks: ROS 2 -> ROS 1 on /cmd_vel_ros2_raw (bridge.yaml's topic) and ROS 1 -> ROS 2 on /bh_r1_to_r2,
# every Twist field compared as text against distinctive values; md5sums; print-pairs.
set -uo pipefail
ARM="$1"; BRIDGE_IMG="$2"; PEER_IMG="$3"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
IFACE="${GO2_CABLE_IFACE:-enp46s0}"
distro() { [[ $1 == *humble* ]] && echo humble || echo foxy; }
BD=$(distro "$BRIDGE_IMG"); PD=$(distro "$PEER_IMG")

if [ "$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo 0)" = "1" ]; then
  echo "REFUSING: $IFACE has carrier. Unplug the Go2 cable." >&2; exit 2; fi
if ss -ltn 'sport = :11311' | grep -q LISTEN; then
  echo "REFUSING: something already listens on :11311 -- not starting a second ROS master." >&2; exit 2; fi

STAMP=$(date -u +%Y%m%dT%H%M%SZ); OUT="$REPO/logs/bridge_humble/${STAMP}_arm$ARM"; mkdir -p "$OUT"
S="$OUT/summary.txt"
log() { echo "$*" | tee -a "$S"; }
log "$(date -u +%FT%TZ) arm $ARM; bridge $BRIDGE_IMG ($BD); peer $PEER_IMG ($PD); ip_forward=$(cat /proc/sys/net/ipv4/ip_forward); $IFACE no carrier"

# Loopback DDS configs: Foxy's Cyclone 0.7 rejects <Interfaces> (logs/foxy/cyclone_syntax_probe.txt).
lo_xml() { local iface='<Interfaces><NetworkInterface name="lo"/></Interfaces>'
  [ "$1" = foxy ] && iface='<NetworkInterfaceAddress>lo</NetworkInterfaceAddress>'
  cat <<X
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><General>$iface<AllowMulticast>false</AllowMulticast></General>
<Discovery><Peers><Peer address="127.0.0.1"/></Peers><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>20</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>
X
}
lo_xml foxy > "$OUT/lo_foxy.xml"; lo_xml humble > "$OUT/lo_humble.xml"
cat > "$OUT/bridge_test.yaml" <<'Y'
topics:
  - topic: /cmd_vel_ros2_raw
    type: geometry_msgs/msg/Twist
    queue_size: 1
  - topic: /bh_r1_to_r2
    type: geometry_msgs/msg/Twist
    queue_size: 1
Y

R1ENV=(-e ROS_MASTER_URI=http://localhost:11311 -e ROS_HOSTNAME=localhost)
# PYTHONUNBUFFERED: `timeout` kills ros2 topic echo before a block-buffered stdout reaches the file (13 Oct, arm A: empty file
# while the bridge logged "Passing message from ROS 1 ... to ROS 2").
R2ENV() { echo "-e PYTHONUNBUFFERED=1 -e ROS_DOMAIN_ID=42 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///out/lo_$1.xml"; }
cleanup() { docker rm -f bh_roscore bh_bridge bh_peer >/dev/null 2>&1; }
trap cleanup EXIT; cleanup

# Distinctive values, <= 12 significant digits so Melodic's python2 str() prints them exactly.
R2R1='{linear: {x: 0.123456789, y: -0.0271828, z: 0.0314159}, angular: {x: 0.000577215, y: -1.41421356, z: 0.69314718}}'
R2R1_V=(0.123456789 -0.0271828 0.0314159 0.000577215 -1.41421356 0.69314718)
R1R2='{linear: {x: -0.271828182, y: 0.0161803, z: -0.0911}, angular: {x: 0.00299792, y: 1.7320508, z: -0.5772156}}'
R1R2_V=(-0.271828182 0.0161803 -0.0911 0.00299792 1.7320508 -0.5772156)

# 1. ROS 1 master (Melodic).
docker run -d --name bh_roscore --network host "${R1ENV[@]}" -v "$OUT":/out ros:melodic-ros-base roscore > /dev/null
for i in $(seq 30); do docker exec bh_roscore bash -c 'source /opt/ros/melodic/setup.bash && rostopic list' >/dev/null 2>&1 && break; sleep 1; done
docker exec bh_roscore bash -c 'source /opt/ros/melodic/setup.bash && rosparam load /out/bridge_test.yaml && rosparam get /topics' > "$OUT/rosparam.txt" 2>&1
log "roscore up after ${i}s; params: $(grep -c topic: "$OUT/rosparam.txt") topics"

# 2. md5sums: Melodic (rosmsg) vs the bridge image's ROS 1 C++ headers.
for t in geometry_msgs/Twist sensor_msgs/LaserScan nav_msgs/Odometry tf2_msgs/TFMessage; do
  m1=$(docker exec bh_roscore bash -c "source /opt/ros/melodic/setup.bash && rosmsg md5 $t" 2>&1 | tail -1)
  m2=$(docker run --rm "$BRIDGE_IMG" bash -c "f=\$(ls /usr/include/$t.h /opt/ros/noetic/include/$t.h 2>/dev/null | head -1); grep -A5 'struct MD5Sum<' \$f | grep -oE '\"[0-9a-f]{32}\"' | head -1 | tr -d '\"'")
  log "md5 $t melodic=$m1 bridge=$m2 $([ "$m1" = "$m2" ] && echo MATCH || echo MISMATCH)"
done

# 3. print-pairs.
if [ "$BD" = humble ]; then SRC='source /opt/ros/humble/setup.bash && source /bridge_ws/install/setup.bash'
else SRC='source /opt/ros/foxy/setup.bash && source /opt/ros/noetic/setup.bash'; fi
docker run --rm "${R1ENV[@]}" "$BRIDGE_IMG" bash -c "$SRC && ros2 run ros1_bridge dynamic_bridge --print-pairs" > "$OUT/print_pairs.txt" 2>&1
for p in geometry_msgs/msg/Twist sensor_msgs/msg/LaserScan nav_msgs/msg/Odometry tf2_msgs/msg/TFMessage; do
  log "print-pairs $p: $(grep -c "'$p'" "$OUT/print_pairs.txt")"; done

# 4. The bridge.
docker run -d --name bh_bridge --network host --ipc=host "${R1ENV[@]}" $(R2ENV "$BD") -v "$OUT":/out "$BRIDGE_IMG" \
  bash -c "$SRC && exec ros2 run ros1_bridge parameter_bridge" > /dev/null
sleep 5

# 5. ROS 2 -> ROS 1: Melodic echoes, the ROS 2 peer publishes.
docker exec bh_roscore bash -c 'source /opt/ros/melodic/setup.bash && timeout 25 rostopic echo -n 3 /cmd_vel_ros2_raw' > "$OUT/r2_to_r1_echo.txt" 2>&1 &
E1=$!
docker run --rm --name bh_peer --network host --ipc=host $(R2ENV "$PD") -v "$OUT":/out "$PEER_IMG" \
  bash -c "source /opt/ros/$PD/setup.bash && ros2 topic pub -r 5 -t 50 /cmd_vel_ros2_raw geometry_msgs/msg/Twist '$R2R1'" > "$OUT/r2_pub.txt" 2>&1 &
P1=$!
wait $E1; docker rm -f bh_peer >/dev/null 2>&1; wait $P1 2>/dev/null
ok=1; for v in "${R2R1_V[@]}"; do grep -qE "(^|[^0-9.-])${v//./\\.}\$" "$OUT/r2_to_r1_echo.txt" || { ok=0; log "  r2->r1 missing $v"; }; done
log "PHB-3 shape (ROS 2 -> ROS 1): $(grep -c '^---' "$OUT/r2_to_r1_echo.txt") msgs, all six fields exact: $([ $ok = 1 ] && echo YES || echo NO)"

# 6. ROS 1 -> ROS 2: the ROS 2 peer echoes, Melodic publishes.
docker run --rm --name bh_peer --network host --ipc=host $(R2ENV "$PD") -v "$OUT":/out "$PEER_IMG" \
  bash -c "source /opt/ros/$PD/setup.bash && timeout 20 ros2 topic echo /bh_r1_to_r2 geometry_msgs/msg/Twist" > "$OUT/r1_to_r2_echo.txt" 2>&1 &
E2=$!
sleep 4
docker exec bh_roscore bash -c "source /opt/ros/melodic/setup.bash && timeout 14 rostopic pub -r 5 /bh_r1_to_r2 geometry_msgs/Twist '$R1R2'" > "$OUT/r1_pub.txt" 2>&1
wait $E2
ok=1; for v in "${R1R2_V[@]}"; do grep -qE "(^|[^0-9.-])${v//./\\.}\$" "$OUT/r1_to_r2_echo.txt" || { ok=0; log "  r1->r2 missing $v"; }; done
log "PHB-4 shape (ROS 1 -> ROS 2): $(grep -c '^---' "$OUT/r1_to_r2_echo.txt") msgs, all six fields exact: $([ $ok = 1 ] && echo YES || echo NO)"

# 7. Bridge log.
docker logs bh_bridge > "$OUT/bridge.txt" 2>&1
log "bridge log: $(wc -l < "$OUT/bridge.txt") lines; md5/checksum errors: $(grep -ciE 'md5|checksum' "$OUT/bridge.txt"); ROS_DISTRO warnings: $(grep -c 'ROS_DISTRO was set' "$OUT/bridge.txt")"
log "$(date -u +%FT%TZ) end; ip_forward=$(cat /proc/sys/net/ipv4/ip_forward)"
