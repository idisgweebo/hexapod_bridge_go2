#!/usr/bin/env bash
#
# pub_test.sh — publish a tiny Twist to /cmd_vel_ros2_raw for a bounded duration,
# from a Foxy-ONLY ROS 2 environment. For Phase 4E motion testing.
#
# Usage:
#   bash scripts/pub_test.sh [linear_x] [linear_y] [angular_z] [duration_s]
#
# Examples:
#   bash scripts/pub_test.sh                 # tiny 0.01 m/s forward nudge, 1 s
#   bash scripts/pub_test.sh 0.0 0.0 0.05    # tiny rotate, 1 s
#   bash scripts/pub_test.sh 0.02 0.0 0.0 1.5  # 0.02 m/s forward, 1.5 s
#
# SAFETY NOTES:
#   * Publishes to /cmd_vel_ros2_raw (the bridge INPUT), never to /cmd_vel directly.
#     The command still has to pass the bridge -> cmd_guard (clamp + watchdog) chain.
#   * The publish is count-bounded (DURATION x 10 Hz messages, after the bridge subscribes). When it stops, no fresh commands arrive, so
#     cmd_guard's watchdog zeroes its output ~0.4 s later. That IS the stop mechanism.
#   * For real motion (Stage 4D onward) the hexapod must be armed with the joystick
#     Y button. That hardware arming stays authoritative; this script never bypasses it.
#   * Requires bridge_shell.sh to already be running in another terminal.
#
set -euo pipefail

LINEAR_X="${1:-0.01}"   # m/s forward(+)/back(-)
LINEAR_Y="${2:-0.0}"    # m/s left(+)/right(-) strafe
ANGULAR_Z="${3:-0.0}"   # rad/s rotate
DURATION="${4:-1.0}"    # seconds to publish

# 15 Oct 2026: Humble image + Cyclone, and the SAME explicit domain / DDS config as bridge_shell.sh
# (it used to inherit domain 0 inside the container -- the Go2's domain).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
IMAGE="${BRIDGE_IMG:-bridge_humble:cmd_guard}"
DOMAIN="${ROS_DOMAIN_ID:-42}"
CYCLONE_XML="${CYCLONE_XML:-$REPO_ROOT/config/cyclonedds_lo.xml}"
TOPIC="/cmd_vel_ros2_raw"
RATE_HZ=10              # 10 Hz keeps fresh commands inside cmd_guard's 0.4 s watchdog

if [ "$DOMAIN" = 0 ] && [ "${ALLOW_DOMAIN_0:-0}" != 1 ]; then
  echo "REFUSING: ROS_DOMAIN_ID=0 is the Go2's domain. Set ALLOW_DOMAIN_0=1 only if that is really intended." >&2; exit 2; fi

# MEASURED 15 Oct 2026 (Orin, real hexapod): a wall-clock `timeout ${DURATION}` around `ros2 topic pub` includes the CLI's
# ~1.2-1.5 s startup in Docker, so `pub_test.sh` with the 1 s default published NOTHING and 2 s published 2 messages.
# Now: a message COUNT (--times) after waiting for the bridge's subscription (-w 1); the timeout is only a hang guard.
COUNT=$(awk -v d="$DURATION" -v r="$RATE_HZ" 'BEGIN{n=int(d*r+0.5); print (n<1?1:n)}')
STARTUP_S=20            # hang guard: no matching subscriber (bridge not running) within this, give up
GUARD_S=$(awk -v d="$DURATION" -v s="$STARTUP_S" 'BEGIN{print d+s}')

echo "Publishing $COUNT messages to $TOPIC at ${RATE_HZ} Hz (~${DURATION}s) once a subscriber matches (domain $DOMAIN, $(basename "$CYCLONE_XML")):"
echo "  linear.x=$LINEAR_X  linear.y=$LINEAR_Y  angular.z=$ANGULAR_Z"
echo "(auto-stops; cmd_guard watchdog then zeroes output. Ctrl+C to abort early.)"

# Build the in-container command. Host shell expands the velocity vars now, baking
# the numbers into the string. Single quotes around the YAML are passed through
# literally to the container's ros2 as one argument.
#
# The image's default entrypoint sources Humble ONLY -- exactly what the ros2 CLI needs.
# -w 1 --times N     : wait for one matching subscription (the bridge), publish exactly N, exit.
# timeout ${GUARD_S}  : hang guard only; exit 124 means it gave up (no subscriber, or stuck).
CMD="source /opt/ros/humble/setup.bash && timeout ${GUARD_S} ros2 topic pub -w 1 --times ${COUNT} --rate ${RATE_HZ} ${TOPIC} geometry_msgs/msg/Twist '{linear: {x: ${LINEAR_X}, y: ${LINEAR_Y}, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${ANGULAR_Z}}}'"

TTY=(); [ -t 0 ] && TTY=(-it)
docker run --rm "${TTY[@]}" \
  --network host \
  --ipc=host \
  -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///cyclone.xml \
  -v "$CYCLONE_XML:/cyclone.xml:ro" \
  "$IMAGE" \
  bash -c "$CMD" && rc=0 || rc=$?

if [ "$rc" = 0 ]; then
  echo "Published $COUNT messages. Output should now be zeroed by the watchdog."
elif [ "$rc" = 124 ]; then
  echo "TIMED OUT after ${GUARD_S}s: no subscriber matched, or the publish stalled. Is bridge_shell.sh running on domain $DOMAIN?" >&2; exit 124
else
  echo "ros2 topic pub exited $rc." >&2; exit "$rc"
fi
