#!/usr/bin/env bash
set -euo pipefail

# HEXAPOD_IP: the address this machine reaches the Nano at — 192.168.88.100 on the hexapod AP
# (ap0, default), 192.168.50.50 on the laptop hotspot (dogspider). Override: HEXAPOD_IP=... ./script
HEXAPOD_IP="${HEXAPOD_IP:-192.168.88.100}"
# Since 1 Oct 2026 the hexapod advertises its nodes by NAME (ROS_HOSTNAME=xrrobot-desktop), not by
# IP. A client that cannot resolve that name reaches the master but silently gets NO topic data, so
# every container maps it to HEXAPOD_IP with --add-host.
HEXAPOD_HOSTNAME="xrrobot-desktop"
LAPTOP_ROS_IP="$(ip route get "$HEXAPOD_IP" | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}')"

echo "Using ROS_MASTER_URI=http://$HEXAPOD_IP:11311"
echo "Using ROS_IP=$LAPTOP_ROS_IP"
echo "Mapping $HEXAPOD_HOSTNAME -> $HEXAPOD_IP inside the container"

docker run --rm -it \
  --name hexapod_ros1_test \
  --network host \
  --add-host "$HEXAPOD_HOSTNAME:$HEXAPOD_IP" \
  -e ROS_MASTER_URI="http://$HEXAPOD_IP:11311" \
  -e ROS_IP="$LAPTOP_ROS_IP" \
  -v "$PWD/logs:/logs" \
  ros:noetic-ros-base-focal \
  bash
