#!/usr/bin/env bash
#
# gate8_bridge.sh — Gate 8: the hexapod's ROS 1 /go2/cmd_vel  ->  ros1_bridge (Humble)  ->  ROS 2 /go2/cmd_vel on the Go2's DOMAIN 0.
# What reads it on the ROS 2 side is go2_cmd_bridge (clamps, 0.4 s watchdog, aborts), started separately. This script moves nothing
# by itself: with no go2_cmd_bridge running -- or one running dry -- a Twist here reaches no robot.
# Plan: Claude_Research gate8_hexapod_to_go2_plan.md (decisions D1-D7).
#
# Env:
#   ALLOW_DOMAIN_0=1   REQUIRED. Domain 0 is the Go2's; this script exists to join it (D1, option A).
#   GATE8_DOMAIN       0 (default). Any other value is a control (stage 1 phase C) -- the Go2 side then sees nothing.
#                      Deliberately NOT ROS_DOMAIN_ID: an exported 42 left in a shell must not silently turn a real run into a control.
#   HEXAPOD_IP         192.168.50.50 (hexapod on dogspider) | 127.0.0.1 (loopback rehearsal)
#   CYCLONE_XML        config/go2/cyclonedds_gate8.xml (enp46s0 only) | config/cyclonedds_lo.xml (rehearsal)
#   BRIDGE_IMG         bridge_humble:cmd_guard
#   BRIDGE_YAML        config/gate8_bridge.yaml (/go2/cmd_vel, default) | config/dance_bridge.yaml (Friday demo: /go2/action ONLY)
#                      Must be a file directly in config/. Added 28 Oct: until then NOTHING loaded dance_bridge.yaml (found by
#                      scripts/dance_loopback_test.sh's design, before S4) -- S4 would have bridged /go2/cmd_vel and not /go2/action.
#   GATE8_LOGROOT      logs/gate8
#
# REFUSES (D3, D7, rule 5b): ip_forward = 1; a Cyclone XML naming any interface but lo / enp46s0; domain 0 without ALLOW_DOMAIN_0=1.
#
# ⚠️ parameter_bridge bridges every listed topic in BOTH directions (D2). Its list is read from the ROS 1 master -- the HEXAPOD's --
# so it is loaded under /gate8_bridge just before start, the service lists point at names that are never set, and the parameter is
# deleted again on exit. The created bridges are printed in the log ("create bidirectional bridge"); check there, not in the YAML.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd -P)"   # -P: the same answer via the ~/Claude_Research symlink

HEXAPOD_IP="${HEXAPOD_IP:-192.168.50.50}"
HEXAPOD_HOSTNAME="xrrobot-desktop"     # the Nano advertises by name since 1 Oct; unmapped -> no data, silently
DOMAIN="${GATE8_DOMAIN:-0}"
CYCLONE_XML="${CYCLONE_XML:-$REPO_ROOT/config/go2/cyclonedds_gate8.xml}"
BRIDGE_IMG="${BRIDGE_IMG:-bridge_humble:cmd_guard}"
LOGROOT="${GATE8_LOGROOT:-$REPO_ROOT/logs/gate8}"
NAME="g8_ros1_bridge"
PARAM_NS="/gate8_bridge"

refuse() { echo "REFUSING: $*" >&2; exit 2; }
[ "$DOMAIN" = 0 ] && [ "${ALLOW_DOMAIN_0:-0}" != 1 ] && refuse "domain 0 is the Go2's. Set ALLOW_DOMAIN_0=1 -- that is this script's purpose."
[ "$(cat /proc/sys/net/ipv4/ip_forward)" = 1 ] && refuse "ip_forward is 1. This host bridges dogspider and the Go2 cable (rule 5b): sudo sysctl -w net.ipv4.ip_forward=0"
BRIDGE_YAML="${BRIDGE_YAML:-$REPO_ROOT/config/gate8_bridge.yaml}"
[ -f "$BRIDGE_YAML" ] || refuse "$BRIDGE_YAML not found."
[ "$(cd "$(dirname "$BRIDGE_YAML")" && pwd -P)" = "$REPO_ROOT/config" ] || refuse "$BRIDGE_YAML is not directly in $REPO_ROOT/config/."
BRIDGED="$(grep -oE '^ *- *topic: *[^ ]+' "$BRIDGE_YAML" | sed -E 's/.*topic: *//' | tr '\n' ' ')"
[ -n "$BRIDGED" ] || refuse "$BRIDGE_YAML lists no topics."
[ -f "$CYCLONE_XML" ] || refuse "$CYCLONE_XML not found."
IFACES="$(grep -oE 'NetworkInterface[^>]*name="[^"]*"' "$CYCLONE_XML" | sed -E 's/.*name="([^"]*)"/\1/' | sort -u | tr '\n' ' ')"
[ -n "$IFACES" ] || refuse "$CYCLONE_XML names no interface (Cyclone would pick one itself -- possibly Wi-Fi)."
for i in $IFACES; do case "$i" in lo|enp46s0) ;; *) refuse "$CYCLONE_XML binds '$i'. Only lo and enp46s0 are allowed (rule 5b, CVE-2026-27509)." ;; esac; done
docker image inspect "$BRIDGE_IMG" >/dev/null 2>&1 || refuse "image $BRIDGE_IMG not found (docker/bridge_humble/Dockerfile)."
docker inspect "$NAME" >/dev/null 2>&1 && refuse "container $NAME already exists."
ROS_IP_SELF="$(ip route get "$HEXAPOD_IP" | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}')"
[ -n "$ROS_IP_SELF" ] || refuse "no route to $HEXAPOD_IP."
ROUTE_DEV="$(ip route get "$HEXAPOD_IP" | awk '{for(i=1;i<=NF;i++) if($i=="dev") print $(i+1)}')"

LOGDIR="$LOGROOT/$(date -u +%Y%m%dT%H%M%S.%3NZ)"; mkdir -p "$LOGDIR"
{
  echo "$(date -u +%FT%T.%3NZ) gate8_bridge.sh $(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null)$(git -C "$REPO_ROOT" diff --quiet 2>/dev/null || echo '+dirty')"
  echo "image $BRIDGE_IMG; ROS_MASTER_URI=http://$HEXAPOD_IP:11311 ROS_IP=$ROS_IP_SELF (dev $ROUTE_DEV); $HEXAPOD_HOSTNAME -> $HEXAPOD_IP"
  echo "ROS 2: domain $DOMAIN, rmw_cyclonedds_cpp, $(basename "$CYCLONE_XML") -> interfaces: $IFACES"
  echo "topics: $PARAM_NS/topics from config/$(basename "$BRIDGE_YAML") = [ $BRIDGED]; services: $PARAM_NS/services_1_to_2, _2_to_1 (never set)"
  echo "ip_forward=$(cat /proc/sys/net/ipv4/ip_forward)"
} | tee "$LOGDIR/config.txt"

COMMON=(--network host --add-host "$HEXAPOD_HOSTNAME:$HEXAPOD_IP"
  -e ROS_MASTER_URI="http://$HEXAPOD_IP:11311" -e ROS_IP="$ROS_IP_SELF" -e PYTHONUNBUFFERED=1)
TTY=(); [ -t 0 ] && TTY=(-it)

cleanup() {
  trap - EXIT INT TERM
  if docker inspect "$NAME" >/dev/null 2>&1; then
    docker stop -t 5 "$NAME" >/dev/null 2>&1 || true
    docker logs "$NAME" > "$LOGDIR/bridge.txt" 2>&1 || true
    docker rm "$NAME" >/dev/null 2>&1 || true
  fi
  # The parameter lives on the (hexapod's) master until deleted -- unlike /topics after bridge_shell.sh (cmd_guard port §8).
  docker run --rm "${COMMON[@]}" "$BRIDGE_IMG" /ros1_env.sh timeout 10 rosparam delete "$PARAM_NS" >> "$LOGDIR/config.txt" 2>&1 \
    && echo "deleted $PARAM_NS" >> "$LOGDIR/config.txt" || echo "WARNING: could not delete $PARAM_NS from the master" | tee -a "$LOGDIR/config.txt"
  echo "$(date -u +%FT%T.%3NZ) stopped; ip_forward=$(cat /proc/sys/net/ipv4/ip_forward); logs in $LOGDIR" | tee -a "$LOGDIR/config.txt"
}
trap cleanup EXIT INT TERM

# parameter_bridge is exec'd directly (not `ros2 run`): as PID 1 under ros2 run, SIGINT was swallowed (measured 15 Oct, bridge_shell.sh).
docker run "${TTY[@]}" --name "$NAME" "${COMMON[@]}" --ipc=host \
  -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///cyclone.xml \
  -v "$CYCLONE_XML:/cyclone.xml:ro" -v "$BRIDGE_YAML:/gate8_bridge.yaml:ro" \
  "$BRIDGE_IMG" bash -c "/ros1_env.sh rosparam load /gate8_bridge.yaml $PARAM_NS \
    && source /opt/ros/humble/setup.bash && source /bridge_ws/install/setup.bash \
    && exec /bridge_ws/install/ros1_bridge/lib/ros1_bridge/parameter_bridge \
         $PARAM_NS/topics $PARAM_NS/services_1_to_2 $PARAM_NS/services_2_to_1" || true
