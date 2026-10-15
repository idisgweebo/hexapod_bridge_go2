#!/usr/bin/env bash
# Gate 8 -- the hexapod as the command source: ONE bounded `rostopic pub -r 10 /go2/cmd_vel` on the Nano, over ssh.
#   gate8_hex_source.sh SECONDS VX [WZ]     e.g. 3.8 0.3  -> ~2.5 s of stream (rostopic pub takes ~1.27 s to start on the Nano,
#                                            measured stage 2) -> SIGINT from `timeout` = the stop path (go2_cmd_bridge's 0.4 s watchdog).
# Whether this moves the Go2 is decided by go2_cmd_bridge (dry or ARMED). ⛔ With an ARMED bridge running, this is Doug's to run.
# Bounds (the bridge clamps anyway; these keep a typo from becoming a long walk): SECONDS <= 6, |VX| <= 0.3, |WZ| <= 1.0.
# Ctrl+C here also stops it (ssh -t carries the SIGINT to the Nano).
set -euo pipefail
NANO="${NANO:-xrrobot@192.168.50.50}"
S="${1:?SECONDS}"; VX="${2:?VX}"; WZ="${3:-0.0}"
python3 - "$S" "$VX" "$WZ" <<'EOF' || { echo "REFUSING: need 0 < SECONDS <= 6, |VX| <= 0.3, |WZ| <= 1.0" >&2; exit 2; }
import sys; s, vx, wz = map(float, sys.argv[1:]); sys.exit(0 if 0 < s <= 6 and abs(vx) <= 0.3 and abs(wz) <= 1.0 else 1)
EOF
echo "$(date -u +%FT%T.%3NZ) hexapod source: ${S}s (timeout, incl. ~1.3 s startup) vx $VX wz $WZ"
ssh -t -o BatchMode=yes -o ConnectTimeout=5 "$NANO" \
  "source /opt/ros/melodic/setup.bash && export ROS_MASTER_URI=http://localhost:11311 ROS_HOSTNAME=xrrobot-desktop && \
   timeout -s INT $S rostopic pub -r 10 /go2/cmd_vel geometry_msgs/Twist -- '[$VX, 0.0, 0.0]' '[0.0, 0.0, $WZ]'" || true
echo "$(date -u +%FT%T.%3NZ) hexapod source: ended"
