#!/usr/bin/env bash
#
# bridge_shell.sh — the hexapod command-path bridge: ros1_bridge (Humble) + cmd_guard (ROS 1), one image.
#   ROS 2 /cmd_vel_ros2_raw --parameter_bridge--> ROS 1 /cmd_vel_ros2_raw --cmd_guard (clamp + 0.4 s watchdog)--> $GUARD_OUT
# The bridge runs in the foreground; cmd_guard runs in a second container from the same image. Ctrl+C stops both, and
# both containers' output is saved to logs/bridge/<stamp>/.
#
# Env overrides:
#   HEXAPOD_IP     192.168.88.100 (hexapod AP, ap0) | 192.168.50.50 (laptop hotspot, dogspider)
#   GUARD_OUT      /cmd_vel_safety_test_out (DEFAULT: a dead-end sink, nothing on the hexapod reads it)
#                  /cmd_vel_ros2_safe = the twist_mux priority-10 input -- the real handoff. Doug's go-ahead only.
#   GUARD=0        bridge only, no cmd_guard
#   ROS_DOMAIN_ID  42 (explicit; never inherited). 0 is the Go2's domain and is refused unless ALLOW_DOMAIN_0=1.
#   CYCLONE_XML    config/cyclonedds_lo.xml (loopback only: the bridge's ROS 2 peers must be on this host)
#   BRIDGE_IMG     bridge_humble:cmd_guard  (docker/bridge_humble/Dockerfile)
#
# 15 Oct 2026 (Claude_Research cmd_guard_humble_port.md): moved from the Foxy image (hexapod_bridge:cmd_guard, domain 0 by
# default, FastDDS) to Humble + Cyclone. MEASURED that day, and why this script looks the way it does:
#   * ROS 1 Python tools must run via /ros1_env.sh -- a sourced Humble shadows ROS 1's Python modules.
#   * roslaunch rewrites the master's /rosdistro and /rosversion (melodic -> Debian), and they stay rewritten. So cmd_guard is
#     started by rosparam load + its catkin devel wrapper, which leaves them alone. (rosrun is not in this image.)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

HEXAPOD_IP="${HEXAPOD_IP:-192.168.88.100}"
# Since 1 Oct 2026 the hexapod advertises its nodes by NAME (ROS_HOSTNAME=xrrobot-desktop), not by
# IP. A client that cannot resolve that name reaches the master but silently gets NO topic data, so
# every container maps it to HEXAPOD_IP with --add-host.
HEXAPOD_HOSTNAME="xrrobot-desktop"
GUARD="${GUARD:-1}"
GUARD_OUT="${GUARD_OUT:-/cmd_vel_safety_test_out}"
DOMAIN="${ROS_DOMAIN_ID:-42}"
CYCLONE_XML="${CYCLONE_XML:-$REPO_ROOT/config/cyclonedds_lo.xml}"
BRIDGE_IMG="${BRIDGE_IMG:-bridge_humble:cmd_guard}"

if [ "$DOMAIN" = 0 ] && [ "${ALLOW_DOMAIN_0:-0}" != 1 ]; then
  echo "REFUSING: ROS_DOMAIN_ID=0 is the Go2's domain. Set ALLOW_DOMAIN_0=1 only if that is really intended." >&2; exit 2; fi
docker image inspect "$BRIDGE_IMG" >/dev/null 2>&1 || {
  echo "REFUSING: image $BRIDGE_IMG not found. Build: docker build --network host -t $BRIDGE_IMG -f docker/bridge_humble/Dockerfile ." >&2; exit 2; }
[ -f "$CYCLONE_XML" ] || { echo "REFUSING: $CYCLONE_XML not found." >&2; exit 2; }
# This machine's own IP on the route to the hexapod (the src address).
ROS_IP_SELF="$(ip route get "$HEXAPOD_IP" | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}')"
[ -n "$ROS_IP_SELF" ] || { echo "REFUSING: no route to $HEXAPOD_IP." >&2; exit 2; }

LOGDIR="$REPO_ROOT/logs/bridge/$(date -u +%Y%m%dT%H%M%SZ)"; mkdir -p "$LOGDIR"
{
  echo "$(date -u +%FT%TZ) bridge_shell.sh $(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null)"
  echo "image $BRIDGE_IMG; ROS_MASTER_URI=http://$HEXAPOD_IP:11311 ROS_IP=$ROS_IP_SELF; $HEXAPOD_HOSTNAME -> $HEXAPOD_IP"
  echo "ROS 2: domain $DOMAIN, rmw_cyclonedds_cpp, $(basename "$CYCLONE_XML")"
  echo "cmd_guard: $([ "$GUARD" = 1 ] && echo "ON, output -> $GUARD_OUT" || echo OFF)"
  echo "ip_forward=$(cat /proc/sys/net/ipv4/ip_forward)"
} | tee "$LOGDIR/config.txt"
[ "$GUARD" = 1 ] && [ "$GUARD_OUT" = /cmd_vel_ros2_safe ] && \
  echo "WARNING: cmd_guard output is the twist_mux input /cmd_vel_ros2_safe -- with the mux live, this path can move the hexapod."
[ "$(cat /proc/sys/net/ipv4/ip_forward)" = 1 ] && echo "WARNING: ip_forward is 1 on this machine (rule 5b)."

COMMON=(--network host
  --add-host "$HEXAPOD_HOSTNAME:$HEXAPOD_IP"
  -e ROS_MASTER_URI="http://$HEXAPOD_IP:11311" -e ROS_IP="$ROS_IP_SELF" -e PYTHONUNBUFFERED=1
  -v "$REPO_ROOT/config:/config:ro")
TTY=(); [ -t 0 ] && TTY=(-it)

cleanup() {
  trap - EXIT INT TERM
  for c in hexapod_ros1_bridge hexapod_cmd_guard; do
    docker inspect "$c" >/dev/null 2>&1 || continue
    docker stop -t 5 "$c" >/dev/null 2>&1
    docker logs "$c" > "$LOGDIR/${c#hexapod_}.txt" 2>&1
    docker rm "$c" >/dev/null 2>&1
  done
  echo "$(date -u +%FT%TZ) stopped; logs in $LOGDIR" | tee -a "$LOGDIR/config.txt"
}
trap cleanup EXIT INT TERM

if [ "$GUARD" = 1 ]; then
  # Params into /cmd_guard only (no roslaunch: see header), output topic overridden, then the node itself.
  docker run -d --name hexapod_cmd_guard "${COMMON[@]}" "$BRIDGE_IMG" /ros1_env.sh bash -c \
    "rosparam load /catkin_ws/src/cmd_guard/config/cmd_guard.yaml /cmd_guard \
     && rosparam set /cmd_guard/output_topic '$GUARD_OUT' \
     && exec /catkin_ws/devel/lib/cmd_guard/cmd_guard_node.py" >/dev/null
fi

# The bridge reads its topic list from the parameter server (config/bridge.yaml). rosparam is a ROS 1 tool -> /ros1_env.sh;
# parameter_bridge then runs in the Humble environment. --ipc=host is harmless with Cyclone and kept from the FastDDS days.
docker run "${TTY[@]}" --name hexapod_ros1_bridge "${COMMON[@]}" --ipc=host \
  -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///cyclone.xml \
  -v "$CYCLONE_XML:/cyclone.xml:ro" \
  "$BRIDGE_IMG" bash -c "/ros1_env.sh rosparam load /config/bridge.yaml \
    && source /opt/ros/humble/setup.bash && source /bridge_ws/install/setup.bash \
    && exec ros2 run ros1_bridge parameter_bridge" || true
