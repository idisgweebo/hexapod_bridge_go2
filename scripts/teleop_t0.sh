#!/usr/bin/env bash
# Gate 7 §10f stage T0 -- host launcher. Fake Go2 + ARMED bridge + teleop on THIS display.
# NO ROBOT: refuses if the Go2 cable has carrier. Logs land in logs/teleop/t0/.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
IFACE="${GO2_CABLE_IFACE:-enp46s0}"

if [ "$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo 0)" = "1" ]; then
  echo "REFUSING: $IFACE has carrier. Unplug the Go2 cable for T0." >&2; exit 2
fi
: "${DISPLAY:?no DISPLAY -- run this from your desktop session}"
XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
mkdir -p "$REPO/logs/teleop/t0"
echo "$(date -u +%FT%TZ)  T0 start; ip_forward=$(cat /proc/sys/net/ipv4/ip_forward); $IFACE no carrier"

rc=0
docker run -it --rm --name go2_teleop_t0 --network host --ipc=host \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/.Xauthority \
  -v "$XAUTH":/tmp/.Xauthority:ro -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -e GO2_CABLE_IFACE="$IFACE" \
  -v "$HOME/go2_ws":/ws:ro \
  -v "$REPO/scripts":/scripts:ro \
  -v "$REPO/ros2_ws/src/go2_cmd_bridge":/src/go2_cmd_bridge:ro \
  -v "$REPO/logs/teleop":/logs \
  go2_humble:cyclone bash -c '
    set -e
    source /ws/install/setup.bash
    mkdir -p /tmp/src && cp -r /src/go2_cmd_bridge /tmp/src/
    colcon --log-base /tmp/l build --base-paths /tmp/src --build-base /tmp/b --install-base /tmp/i \
      > /tmp/colcon.txt 2>&1 || { cat /tmp/colcon.txt; exit 3; }
    source /tmp/i/setup.bash
    python3 /scripts/teleop_t0.py --out /logs/t0' || rc=$?
echo "$(date -u +%FT%TZ)  T0 end rc=$rc; ip_forward=$(cat /proc/sys/net/ipv4/ip_forward)"
exit $rc
