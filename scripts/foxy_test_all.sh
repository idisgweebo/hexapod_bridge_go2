#!/usr/bin/env bash
# Foxy port check (12 Oct 2026): build go2_adapter + go2_cmd_bridge on the Foxy image and run
# every offline and *_integration_rosonly suite there. NO ROBOT: refuses if the Go2 cable has carrier.
# One file per suite in logs/foxy/<stamp>/, plus summary.txt.
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
IFACE="${GO2_CABLE_IFACE:-enp46s0}"
IMAGE="${1:-go2_foxy:cyclone}"   # the control: scripts/foxy_test_all.sh go2_humble:cyclone
TESTS="${TESTS:-test_*_offline.py test_*_integration_rosonly.py}"   # e.g. TESTS=test_cmd_bridge_integration_rosonly.py
DISTRO="$( [[ $IMAGE == *humble* ]] && echo humble || echo foxy )"
if [ "$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo 0)" = "1" ]; then
  echo "REFUSING: $IFACE has carrier. Unplug the Go2 cable." >&2; exit 2
fi
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="$REPO/logs/foxy/${STAMP}_$DISTRO"; mkdir -p "$OUT"
echo "$(date -u +%FT%TZ) start; ip_forward=$(cat /proc/sys/net/ipv4/ip_forward); $IFACE no carrier; image $IMAGE; tests: $TESTS" | tee "$OUT/summary.txt"
docker run --rm --name go2_foxy_tests --network host --ipc=host \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e GO2_CABLE_IFACE="$IFACE" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v "$REPO/scripts":/repo/scripts:ro -v "$REPO/ros2_ws/src":/repo/ros2_ws/src:ro -v "$REPO/hexapod":/repo/hexapod:ro -v "$REPO/logs/ros2_scan_echo.txt":/repo/logs/ros2_scan_echo.txt:ro -v "$OUT":/out \
  -v "$HOME/go2_ws":/hws:ro -e DISTRO=$DISTRO -e TESTS="$TESTS" \
  "$IMAGE" bash -c '
    source /opt/ros/$DISTRO/setup.bash
    if [ $DISTRO = humble ]; then source /hws/install/setup.bash; else source /ws/install/setup.bash; fi
    env | grep -E "ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_LOCALHOST_ONLY|ROS_DISTRO" > /out/env.txt
    python3 --version >> /out/env.txt
    # Writable copy, same layout: the tests find ../ros2_ws/src and write scratch files beside themselves.
    mkdir -p /tmp/repo && cp -r /repo/scripts /repo/ros2_ws /repo/logs /repo/hexapod /tmp/repo/   # hexapod/: phase 1 checker (29 Oct)
    colcon --log-base /tmp/l build --base-paths /tmp/repo/ros2_ws/src --build-base /tmp/b --install-base /tmp/i \
      > /out/colcon.txt 2>&1; echo "colcon rc=$?" | tee -a /out/summary.txt
    source /tmp/i/setup.bash
    cd /tmp/repo/scripts || { echo "NO WORKSPACE COPY" | tee -a /out/summary.txt; exit 4; }
    for t in $TESTS; do
      case $t in *integration*) cmd="timeout 900 python3 $t";; *) cmd="timeout 300 python3 $t";; esac
      $cmd > /out/${t%.py}.txt 2>&1; rc=$?
      printf "%-48s rc=%s  %s\n" "$t" "$rc" "$(grep -iE "[0-9]+/[0-9]+|PASS|FAIL" /out/${t%.py}.txt | tail -1)" | tee -a /out/summary.txt
    done'
echo "$(date -u +%FT%TZ) end; ip_forward=$(cat /proc/sys/net/ipv4/ip_forward)" | tee -a "$OUT/summary.txt"
