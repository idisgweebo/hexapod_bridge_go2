#!/usr/bin/env bash
#
# bridge_shell.sh — launch the ros1_bridge with the cmd_guard image and bridge
# /cmd_vel_ros2_raw (ROS 2 -> ROS 1). This is the Phase 4 command-path bridge.
#
# Reads the topic list from config/bridge.yaml via the ROS 1 parameter server,
# then runs parameter_bridge. Leave this running in its own terminal.
#
set -euo pipefail

# Resolve the repo root from THIS script's own location, so the script works no
# matter which directory you launch it from. ${BASH_SOURCE[0]} is the path to the
# script; dirname strips the filename; cd+pwd makes it a clean absolute path.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

HEXAPOD_IP="192.168.88.100"
# Find the laptop's own IP on the route to the hexapod (the src address).
LAPTOP_ROS_IP="$(ip route get "$HEXAPOD_IP" | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}')"

echo "Using ROS_MASTER_URI=http://$HEXAPOD_IP:11311"
echo "Using ROS_IP=$LAPTOP_ROS_IP"

# --network host : share the host network so ROS 1 (TCP to master) and ROS 2 (DDS
#                  discovery over multicast) both work.
# --ipc=host     : share the host's /dev/shm. REQUIRED — FastDDS moves message DATA
#                  between same-host containers over shared memory, not the network.
#                  Without this, discovery succeeds but data silently never arrives.
docker run --rm -it \
  --name hexapod_ros1_bridge \
  --network host \
  --ipc=host \
  -e ROS_MASTER_URI="http://$HEXAPOD_IP:11311" \
  -e ROS_IP="$LAPTOP_ROS_IP" \
  -v "$REPO_ROOT/logs:/logs" \
  -v "$REPO_ROOT/config:/config" \
  hexapod_bridge:cmd_guard \
  bash -c "rosparam load /config/bridge.yaml && ros2 run ros1_bridge parameter_bridge"
