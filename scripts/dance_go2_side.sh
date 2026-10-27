#!/usr/bin/env bash
# Friday dance demo, robot stages S1-S4 -- start the Go2-side session container `dance_go2` on the CABLE
# (domain 0, config/go2/cyclonedds.xml -> enp46s0), with go2_action_bridge built from the repo and /tmp/env.sh
# to source. Starts NOTHING that publishes: the container idles. Every command after this is a `docker exec`
# from the run sheet in Claude_Research friday_dance_demo_plan.md §3.3;
# ⛔ the ARMED go2_action_bridge is Doug's to run.
#   dance_go2_side.sh            -> logs in logs/dance/<UTC date>/ (mounted at /logs)
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
GO2_IP=192.168.123.161
refuse() { echo "REFUSING: $*" >&2; exit 2; }
[ "$(cat /sys/class/net/enp46s0/carrier 2>/dev/null || echo 0)" = 1 ] || refuse "enp46s0 has no carrier (Go2 cable in, go2-direct up?)."
[ "$(cat /proc/sys/net/ipv4/ip_forward)" = 1 ] && refuse "ip_forward is 1 (rule 6)."
DEV="$(ip route get $GO2_IP | awk '{for(i=1;i<=NF;i++) if($i=="dev") print $(i+1)}')"
[ "$DEV" = enp46s0 ] || refuse "route to $GO2_IP is via '$DEV', not enp46s0 (orin-direct active? gotcha #1/#2)."
nmcli -t -f NAME con show --active | grep -qx orin-direct && refuse "NM has orin-direct active on the port; bring up go2-direct (27 Oct, again)."
docker inspect dance_go2 >/dev/null 2>&1 && refuse "container dance_go2 exists."

LOGS="$REPO/logs/dance/$(date -u +%Y%m%d)"; mkdir -p "$LOGS"
{
  echo "$(date -u +%FT%T.%3NZ) dance_go2_side.sh $(git -C "$REPO" rev-parse --short HEAD)$(git -C "$REPO" diff --quiet || echo +dirty)"
  echo "route $GO2_IP via $DEV; active: $(nmcli -t -f NAME con show --active | tr '\n' ' ')"
  echo "ip_forward before docker run: $(cat /proc/sys/net/ipv4/ip_forward)"
} | tee -a "$LOGS/session.txt"

docker run -d --name dance_go2 --network host --ipc=host --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///cfg/cyclonedds.xml -e PYTHONUNBUFFERED=1 \
  -v "$HOME/go2_ws":/ws:ro -v "$REPO/scripts":/scripts:ro -v "$REPO/config/go2":/cfg:ro \
  -v "$REPO/ros2_ws/src/go2_action_bridge":/src/go2_action_bridge:ro -v "$LOGS":/logs \
  go2_humble:cyclone bash -c 'source /ws/install/setup.bash && mkdir -p /tmp/src && cp -r /src/go2_action_bridge /tmp/src/ \
    && colcon --log-base /tmp/l build --base-paths /tmp/src --build-base /tmp/b --install-base /tmp/i > /logs/colcon.txt 2>&1 \
    && printf "set +u\nsource /ws/install/setup.bash\nsource /tmp/i/setup.bash\n" > /tmp/env.sh && touch /tmp/BUILT; exec sleep infinity' >/dev/null
echo "ip_forward after docker run: $(cat /proc/sys/net/ipv4/ip_forward)" | tee -a "$LOGS/session.txt"
for i in $(seq 120); do docker exec dance_go2 test -f /tmp/BUILT 2>/dev/null && break; sleep 1; done
docker exec dance_go2 test -f /tmp/BUILT || refuse "go2_action_bridge build failed -- $LOGS/colcon.txt"
docker exec dance_go2 bash -c 'env | grep -E "ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_LOCALHOST_ONLY" | sort' | tee -a "$LOGS/session.txt"
echo "dance_go2 ready; logs -> $LOGS (container /logs)" | tee -a "$LOGS/session.txt"
