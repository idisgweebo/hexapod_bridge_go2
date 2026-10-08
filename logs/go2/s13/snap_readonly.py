import time, rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState, LowState
rclpy.init(); n = Node("s13_snapshot_readonly"); got = {}
n.create_subscription(SportModeState, "/sportmodestate", lambda m: got.setdefault("s", m), 10)
n.create_subscription(LowState, "/lowstate", lambda m: got.setdefault("l", m), 10)
t_end = time.time() + 5
while time.time() < t_end and len(got) < 2: rclpy.spin_once(n, timeout_sec=0.05)
now = time.time(); print("laptop UTC", time.strftime("%FT%TZ", time.gmtime(now)))
s, l = got.get("s"), got.get("l")
if s: print("body_height %.4f  error_code %d  position %s" % (s.body_height, s.error_code, [round(x,4) for x in s.position]))
if l:
    print("tick %d -> uptime %.1f s -> booted ~%s (PT-TICK inference: tick = ms since boot)" % (l.tick, l.tick/1000, time.strftime("%H:%M:%SZ", time.gmtime(now - l.tick/1000))))
    print("temps", [m.temperature for m in l.motor_state[:12]], " RR_hip %d RL_hip %d" % (l.motor_state[6].temperature, l.motor_state[9].temperature))
    print("modes", [m.mode for m in l.motor_state[:12]], " head", [hex(x) for x in l.head], " power_v %.3f" % l.power_v)
print("got", sorted(got))
