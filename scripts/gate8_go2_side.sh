#!/usr/bin/env bash
# Gate 8 stage 1 -- the ROS 2 / Go2 side of one rehearsal phase. Runs INSIDE the g8_go2 container (gate8_loopback_test.sh),
# loopback DDS, domain 0, NO ROBOT:
#   fake Go2 (test_cmd_bridge_integration_rosonly.py --fake robot_only) + gate8_ros2_recorder.py + go2_cmd_bridge (dry|armed).
#   gate8_go2_side.sh OUTDIR armed(true|false) RUN_S
set -uo pipefail
OUT="$1"; ARMED="$2"; RUN_S="$3"
mkdir -p "$OUT"
[ "$ROS_DOMAIN_ID" = 0 ] || { echo "expected ROS_DOMAIN_ID=0, got '$ROS_DOMAIN_ID'" >&2; exit 2; }
grep -q '<NetworkInterface name="lo"/>' "${CYCLONEDDS_URI#file://}" || { echo "CYCLONEDDS_URI is not loopback" >&2; exit 2; }
# colcon setup scripts read unset variables (COLCON_TRACE): set -u killed this script at this line in the first run (15 Oct).
set +u; source /ws/install/setup.bash && source /tmp/i/setup.bash; set -u
date -u +%FT%T.%3NZ > "$OUT/go2_side_start.txt"
python3 /scripts/test_cmd_bridge_integration_rosonly.py --fake robot_only "$OUT/fake.jsonl" "$RUN_S" > "$OUT/fake.txt" 2>&1 & FK=$!
python3 /scripts/gate8_ros2_recorder.py "$OUT/recorder.jsonl" "$RUN_S" > "$OUT/recorder.txt" 2>&1 & RC=$!
sleep 2
setsid ros2 run go2_cmd_bridge go2_cmd_bridge_node --ros-args -p armed:="$ARMED" > "$OUT/node.txt" 2>&1 & BR=$!
wait $FK $RC
kill -INT -- -"$BR" 2>/dev/null
for i in $(seq 50); do kill -0 "$BR" 2>/dev/null || break; sleep 0.1; done
kill -0 "$BR" 2>/dev/null && { echo "node still running after 5 s; SIGKILL" >> "$OUT/node.txt"; kill -KILL -- -"$BR"; }
date -u +%FT%T.%3NZ > "$OUT/go2_side_end.txt"
