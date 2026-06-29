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
#   * The publish is time-bounded. When it stops, no fresh commands arrive, so
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

IMAGE="hexapod_bridge:cmd_guard"
TOPIC="/cmd_vel_ros2_raw"
RATE_HZ=10              # 10 Hz keeps fresh commands inside cmd_guard's 0.4 s watchdog

echo "Publishing to $TOPIC for ${DURATION}s at ${RATE_HZ} Hz:"
echo "  linear.x=$LINEAR_X  linear.y=$LINEAR_Y  angular.z=$ANGULAR_Z"
echo "(auto-stops; cmd_guard watchdog then zeroes output. Ctrl+C to abort early.)"

# Build the in-container command. Host shell expands the velocity vars now, baking
# the numbers into the string. Single quotes around the YAML are passed through
# literally to the container's ros2 as one argument.
#
# --entrypoint bash : SKIP the image entrypoint (which sources Noetic on top of Foxy
#                     and shadows ROS 2's Python geometry_msgs, breaking `ros2 topic
#                     pub`). We source Foxy ONLY here so the ros2 CLI works.
# timeout           : bounds the publish; on expiry it SIGTERMs ros2 (exit 124),
#                     so `|| true` keeps the container exit clean.
CMD="source /opt/ros/foxy/setup.bash && timeout ${DURATION} ros2 topic pub --rate ${RATE_HZ} ${TOPIC} geometry_msgs/msg/Twist '{linear: {x: ${LINEAR_X}, y: ${LINEAR_Y}, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${ANGULAR_Z}}}' || true"

docker run --rm -it \
  --network host \
  --ipc=host \
  --entrypoint bash \
  "$IMAGE" \
  -c "$CMD"

echo "Publish window ended. Output should now be zeroed by the watchdog."
