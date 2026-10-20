#!/usr/bin/env bash
# Firmware-update baseline capture -- gate5_firmware_update_protocol.md §2 Tier 1. READ-ONLY: subscriptions and
# graph queries only; nothing in here publishes. The SAME script runs before (PHASE=pre) and after (PHASE=post)
# the update, so a post-update difference is not confounded by a procedure change (§1, §6).
#
#   fw_update_capture.sh start PHASE                  preflight + start the read-only container `fwu` (PHASE = pre|post)
#   fw_update_capture.sh clock                        clock offset, ros2 topic delay /utlidar/robot_pose, 30 s
#   fw_update_capture.sh census                       Tier 1 #1 + #7: topic list -t, info -v for every topic, participant map
#   fw_update_capture.sh static                       Tier 1 #2 + #6: LowState.head (>=200), LidarState strings (>=10)
#   fw_update_capture.sh probe TAG lying|standing     Tier 1 #3-5, #8-10: torque probe (6000) + layout acceptance (3600)
#   fw_update_capture.sh watch                        rear-hip thermal watch, 50 C ceiling (run in its own terminal)
#   fw_update_capture.sh stop                         remove the container
# Logs: logs/go2/fw_update/<PHASE>_<UTC date>/ (container /logs). Every command's full output goes to a file.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
GO2_IP=192.168.123.161
C=fwu
refuse() { echo "REFUSING: $*" >&2; exit 2; }
ex() { docker exec "$C" bash -c "set +u; source /ws/install/setup.bash; $1"; }
logdir() { docker inspect -f '{{range .Mounts}}{{if eq .Destination "/logs"}}{{.Source}}{{end}}{{end}}' "$C"; }
stamp() { echo "$(date -u +%FT%T.%3NZ) $*" | tee -a "$(logdir)/session.txt"; }

case "${1:-}" in
start)
  PHASE="${2:-}"; [[ "$PHASE" == pre || "$PHASE" == post ]] || refuse "PHASE must be pre or post."
  [ "$(cat /sys/class/net/enp46s0/carrier 2>/dev/null || echo 0)" = 1 ] || refuse "enp46s0 has no carrier (Go2 cable in, go2-direct up?)."
  [ "$(cat /proc/sys/net/ipv4/ip_forward)" = 1 ] && refuse "ip_forward is 1 (rule 5b)."
  DEV="$(ip route get $GO2_IP | awk '{for(i=1;i<=NF;i++) if($i=="dev") print $(i+1)}')"
  [ "$DEV" = enp46s0 ] || refuse "route to $GO2_IP is via '$DEV', not enp46s0 (gotcha #1/#2)."
  nmcli -t -f NAME con show --active | grep -qx orin-direct && refuse "NM has orin-direct active on the port; bring up go2-direct."
  docker inspect "$C" >/dev/null 2>&1 && refuse "container $C exists (fw_update_capture.sh stop)."
  LOGS="$REPO/logs/go2/fw_update/${PHASE}_$(date -u +%Y%m%d)"; mkdir -p "$LOGS"
  {
    echo "$(date -u +%FT%T.%3NZ) fw_update_capture.sh start $PHASE  repo $(git -C "$REPO" rev-parse HEAD)$(git -C "$REPO" diff --quiet HEAD -- scripts || echo ' +DIRTY scripts')"
    echo "image $(docker image inspect -f '{{.Id}}' go2_humble:cyclone)"
    echo "route $GO2_IP via $DEV; active: $(nmcli -t -f NAME con show --active | tr '\n' ' ')"
    echo "ip_forward before docker run: $(cat /proc/sys/net/ipv4/ip_forward)"
  } | tee -a "$LOGS/session.txt"
  docker run -d --name "$C" --network host --ipc=host --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///cfg/cyclonedds.xml -e PYTHONUNBUFFERED=1 \
    -v "$HOME/go2_ws":/ws:ro -v "$REPO/scripts":/scripts:ro -v "$REPO/config/go2":/cfg:ro -v "$LOGS":/logs \
    go2_humble:cyclone sleep infinity >/dev/null
  echo "ip_forward after docker run: $(cat /proc/sys/net/ipv4/ip_forward)" | tee -a "$LOGS/session.txt"
  ex 'env | grep -E "ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_LOCALHOST_ONLY" | sort' | tee "$LOGS/env.txt"
  ex 'ros2 interface show unitree_go/msg/MotorState' > "$LOGS/motorstate_iface.txt" 2>&1   # control: unitree_go imports
  echo "$C ready; logs -> $LOGS" | tee -a "$LOGS/session.txt" ;;
clock)
  stamp "clock start"
  ex 'echo "wall BEFORE: $(date -u +%FT%T.%3NZ)"; echo "--- type check ---"; ros2 topic type /utlidar/robot_pose
      echo "--- ros2 topic delay, 30 s ---"; timeout -s INT 30 ros2 topic delay /utlidar/robot_pose --window 1000 || true
      echo "wall AFTER : $(date -u +%FT%T.%3NZ)"' > "$(logdir)/clock_offset.txt" 2>&1
  tail -4 "$(logdir)/clock_offset.txt" ;;
census)
  stamp "census start"
  ex 'ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 8
      ros2 topic list -t > /logs/census_topic_list_t.txt 2>&1
      ros2 topic list --include-hidden-topics > /logs/census_topic_list_hidden.txt 2>&1
      : > /logs/census_info_v.txt
      for t in $(cut -d" " -f1 /logs/census_topic_list_t.txt); do echo "=== $t" >> /logs/census_info_v.txt; ros2 topic info -v "$t" >> /logs/census_info_v.txt 2>&1; done
      ros2 node list --all > /logs/census_node_list_all.txt 2>&1   # control: our own daemon must appear
      python3 - <<EOF > /logs/census_participant_map.txt
t = kind = None
for line in open("/logs/census_info_v.txt"):
    s = line.strip()
    if s.startswith("=== "): t = s[4:]
    elif s.startswith("Endpoint type:"): kind = s.split(":", 1)[1].strip()
    elif s.startswith("GID:"): print(s.split(":", 1)[1].strip().replace(".", "")[:24], kind, t)
EOF
      sort -o /logs/census_participant_map.txt /logs/census_participant_map.txt'
  echo "topics: $(wc -l < "$(logdir)/census_topic_list_t.txt")  endpoints: $(wc -l < "$(logdir)/census_participant_map.txt")"
  grep -c programming_actuator "$(logdir)/census_topic_list_t.txt" | sed 's/^/programming_actuator lines (presence only -- PFU4): /' ;;
static)
  stamp "static start"
  ex 'timeout -s INT 3 ros2 topic echo /lowstate --field head > /logs/lowstate_head.txt 2>&1 || true
      timeout -s INT 15 ros2 topic echo /utlidar/lidar_state > /logs/lidar_state.txt 2>&1 || true'
  L="$(logdir)"; python3 -I - "$L/lowstate_head.txt" <<'EOF'
import collections, sys
msgs = [" ".join(m.split()) for m in open(sys.argv[1]).read().split("\n---") if m.strip()]
print(f"head: {len(msgs)} samples; distinct values:", dict(collections.Counter(msgs)))
EOF
  echo "lidar_state messages: $(grep -c '^firmware_version' "$L/lidar_state.txt")"; grep -E '^(firmware|software|sdk)_version' "$L/lidar_state.txt" | sort | uniq -c ;;
probe)
  TAG="${2:?TAG}"; POS="${3:?lying|standing}"; [[ "$POS" == lying || "$POS" == standing ]] || refuse "posture must be lying or standing."
  YAW="${YAW_REF:?export YAW_REF (last boot's signed yaw offset)}"; OFF="${OFFSET_REF:?export OFFSET_REF (today's clock offset)}"
  LPOS=$([ "$POS" = lying ] && echo "LYING / RESTING" || echo "STANDING")
  stamp "probe $TAG $POS start"
  ex "python3 /scripts/go2_torque_probe.py --session ${TAG}_torque --posture $POS --samples 6000 --csv /logs/${TAG}_torque_samples.csv" \
    > "$(logdir)/${TAG}_torque_report.txt" 2>&1 || echo "torque probe exit $?"
  stamp "probe $TAG torque done"
  ex "python3 /scripts/go2_layout_acceptance.py --session ${TAG}_layout --posture '$LPOS' --samples 3600 --yaw-ref $YAW --offset-ref $OFF --csv /logs/${TAG}_layout_samples.csv" \
    > "$(logdir)/${TAG}_layout_report.txt" 2>&1 || echo "layout acceptance exit $?"
  stamp "probe $TAG done"
  grep -E '^Collected|^RESULT|^(PT1|PT10|P1|P4|P7z|P8r|P8p|P8s|P9|P17) ' "$(logdir)/${TAG}_torque_report.txt" "$(logdir)/${TAG}_layout_report.txt" || true ;;
watch)
  ex "python3 /scripts/go2_thermal_watch.py --ceiling 50 --label fwu 2>&1 | tee -a /logs/thermal_watch.txt" ;;
stop)
  [ -n "$(docker ps -aq -f name=^${C}$)" ] && { stamp "stop"; docker rm -f "$C" >/dev/null; }; echo "stopped" ;;
*) sed -n 2,13p "$0"; exit 1 ;;
esac
