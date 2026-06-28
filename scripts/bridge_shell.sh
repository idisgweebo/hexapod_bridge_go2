#!/usr/bin/env bash
set -euo pipefail

HEXAPOD_IP="192.168.88.100"
LAPTOP_ROS_IP="$(ip route get "$HEXAPOD_IP" | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}')"

echo "Using ROS_MASTER_URI=http://$HEXAPOD_IP:11311"
echo "Using ROS_IP=$LAPTOP_ROS_IP"

docker run --rm -it \
  --name hexapod_ros1_bridge \
  --network host \
  -e ROS_MASTER_URI="http://$HEXAPOD_IP:11311" \
  -e ROS_IP="$LAPTOP_ROS_IP" \
  -v "$PWD/logs:/logs" \
  ros:foxy-ros1-bridge-focal \
  bash
