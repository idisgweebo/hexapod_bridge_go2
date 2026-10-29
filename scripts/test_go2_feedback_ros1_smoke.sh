#!/usr/bin/env bash
# Smoke test of hexapod/go2_feedback_check.py's rospy shell -- ROS 1 ONLY, no ROS 2, no robot.
# Runs inside ros:melodic-ros-base with --network none (Python 2.7, as on the Nano -- the
# Nano's own version is still unread). The fake Go2 side is scripts/fake_go2_status_ros1.py.
#
#   docker run --rm --network none -v $REPO/hexapod:/r/hexapod:ro -v $REPO/scripts:/r/scripts:ro \
#     -v $OUT:/out ros:melodic-ros-base bash /r/scripts/test_go2_feedback_ros1_smoke.sh
#
#   A  listen only: the fake dies after 4 s -> the checker must go UNKNOWN, and never STOPPED after
#   B  --send walk_03 --expect-armed false against a fake reporting armed=false: preflight ok, RECEIVED, MOVING
#   C  --send zero --expect-armed true against armed=false: REFUSED, exit 2, nothing sent
#   D  --send replay: the checker counts the replayed sends
set -u
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://localhost:11311 ROS_HOSTNAME=localhost
roscore > /out/roscore.txt 2>&1 & RC=$!
for i in $(seq 50); do rostopic list >/dev/null 2>&1 && break; sleep 0.2; done
python --version 2>&1 | tee /out/python_version.txt
FAILS=0
ok()   { echo "  ok   $1"; }
fail() { echo "  FAIL $1"; FAILS=$((FAILS+1)); }

echo "A    listen; the status dies at 4 s"
python /r/scripts/fake_go2_status_ros1.py false 4 > /out/A_fake.txt 2>&1 &
python /r/hexapod/go2_feedback_check.py --duration 7 --outdir /out/A > /out/A_check.txt 2>&1
A_LAST=$(tail -1 /out/A/eval.csv | cut -d, -f4)
grep -q ",STOPPED," /out/A/eval.csv && ok "A: STOPPED while the fake was alive" || fail "A: never STOPPED"
[ "$A_LAST" = UNKNOWN ] && ok "A: UNKNOWN after the status died" || fail "A: last reading '$A_LAST', not UNKNOWN"
python - <<'EOF' && ok "A: never STOPPED after the first UNKNOWN that follows a STOPPED" || fail "A: STOPPED after going UNKNOWN"
import csv, sys
rows = list(csv.DictReader(open("/out/A/eval.csv")))
seen_stop = False; unk = False
for r in rows:
    if r["cls"] == "STOPPED":
        if unk: sys.exit(1)
        seen_stop = True
    elif r["cls"] == "UNKNOWN" and seen_stop:
        unk = True
sys.exit(0 if unk else 1)
EOF
grep -q '!!!!! .*UNKNOWN' /out/A_check.txt && ok "A: UNKNOWN printed loudly" || fail "A: UNKNOWN not printed loudly"

echo "B    send walk_03, bridge reports armed=false"
python /r/scripts/fake_go2_status_ros1.py false 12 > /out/B_fake.txt 2>&1 & FK=$!
sleep 1
python /r/hexapod/go2_feedback_check.py --send walk_03 --expect-armed false --duration 3 --outdir /out/B > /out/B_check.txt 2>&1
echo "     rc=$?"
grep -q "preflight ok" /out/B_check.txt && ok "B: preflight passed" || fail "B: preflight did not pass"
grep -q "^[^,]*,[^,]*,True,MOVING," /out/B/eval.csv && ok "B: RECEIVED + MOVING seen" || fail "B: never RECEIVED + MOVING"
N_SENT=$(($(wc -l < /out/B/sent.csv) - 1))
[ "$N_SENT" -ge 26 ] && [ "$N_SENT" -le 30 ] && ok "B: $N_SENT sends (25 + 3 zeros, +/-2)" || fail "B: $N_SENT sends"
grep -q "echo latency n=[1-9]" /out/B_check.txt && ok "B: echo latency measured" || fail "B: no echo latency"
wait $FK

echo "C    send with --expect-armed true, bridge reports armed=false"
python /r/scripts/fake_go2_status_ros1.py false 8 > /out/C_fake.txt 2>&1 & FK=$!
sleep 1
python /r/hexapod/go2_feedback_check.py --send zero --expect-armed true --duration 1 --outdir /out/C > /out/C_check.txt 2>&1
C_RC=$?
[ "$C_RC" = 2 ] && grep -q "REFUSING to send: bridge reports armed='false', expected 'true'" /out/C_check.txt \
  && ok "C: refused, exit 2" || fail "C: rc=$C_RC, not refused as expected"
[ "$(wc -l < /out/C/sent.csv)" = 1 ] && ok "C: nothing sent (header only)" || fail "C: something was sent"
wait $FK

echo "D    send replay"
python /r/scripts/fake_go2_status_ros1.py false 8 > /out/D_fake.txt 2>&1 & FK=$!
sleep 1
python /r/hexapod/go2_feedback_check.py --send replay --expect-armed false --duration 2 --outdir /out/D > /out/D_check.txt 2>&1
grep -qE "'replay_sent': (9|10|11)" /out/D_check.txt && ok "D: ~10 replayed sends counted" || fail "D: replay count wrong"
wait $FK

kill $RC 2>/dev/null
echo
echo "$FAILS failure(s)"
exit $FAILS
