#!/usr/bin/env bash
set -euo pipefail

HEXAPOD_IP="192.168.88.100"

echo "=== Docker check ==="
docker ps >/dev/null
echo "Docker works without sudo."

echo
echo "=== Host route to Hexapod ==="
ip route get "$HEXAPOD_IP"

echo
echo "=== Ping Hexapod ==="
ping -c 4 "$HEXAPOD_IP"

echo
echo "=== Suggested ROS variables ==="
LAPTOP_ROS_IP="$(ip route get "$HEXAPOD_IP" | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}')"
echo "export ROS_MASTER_URI=http://$HEXAPOD_IP:11311"
echo "export ROS_IP=$LAPTOP_ROS_IP"
