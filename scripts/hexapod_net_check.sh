#!/usr/bin/env bash
set -euo pipefail

# HEXAPOD_IP: the address this machine reaches the Nano at — 192.168.88.100 on the hexapod AP
# (ap0, default), 192.168.50.50 on the laptop hotspot (dogspider). Override: HEXAPOD_IP=... ./script
HEXAPOD_IP="${HEXAPOD_IP:-192.168.88.100}"
# Since 1 Oct 2026 the hexapod advertises its nodes by NAME (ROS_HOSTNAME=xrrobot-desktop), not by
# IP. A client that cannot resolve that name reaches the master but silently gets NO topic data, so
# every container maps it to HEXAPOD_IP with --add-host.
HEXAPOD_HOSTNAME="xrrobot-desktop"

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

echo
echo "=== Hexapod node-name resolution ($HEXAPOD_HOSTNAME) ==="
# Only matters for ROS 1 tools run directly on this host; the docker scripts use --add-host.
resolved="$(getent ahostsv4 "$HEXAPOD_HOSTNAME" | awk 'NR==1{print $1}')"
if [[ "$resolved" == "$HEXAPOD_IP" ]]; then
  echo "$HEXAPOD_HOSTNAME resolves to $resolved — host-native ROS 1 tools will get data."
else
  echo "WARNING: $HEXAPOD_HOSTNAME resolves to '${resolved:-nothing}', not $HEXAPOD_IP."
  echo "Host-native ROS 1 tools will reach the master but get NO topic data. Add to /etc/hosts:"
  echo "  $HEXAPOD_IP $HEXAPOD_HOSTNAME"
fi
