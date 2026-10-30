#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS 1 side of phase 1 stage 2 (scripts/phase1_loopback_test.sh): records what the
hexapod's master sees on the four bridged topics, and in `inject` mode sends PFC-0's
ROS 1 -> ROS 2 messages. Python 2.7 + 3 (rospy). NO ROBOT.

    phase1_ros1_probe.py listen|inject RUN_S OUT.jsonl

RECORDS every /go2/cmd_vel_stamped, /go2/robot_status (+ /go2/odom, /go2/imu in inject
mode only: phase X is the one place they are compared; 30 Oct) with the
fields the analyser compares, the ROS 1 header.seq (PFC-1a), the publisher's callerid
(so the probe's own sends are told apart from bridged-in ones), and t = time.time().

INJECT (from t_0 + 14 s, 1 s apart): three TwistStamped and two DiagnosticArray with
hardware_id PFC0-FROM-ROS1. rospy overwrites header.seq on publish
(rospy.msg.serialize_message); the value it used is recorded as sent.
"""
from __future__ import print_function

import json
import sys
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

PFC0_TWIST = [((1761800000, 123456789), (0.123456789012345, -0.2718281828459045, 3.141592653589793,
                                          -0.000001, 1.0000000000000002, -6.02214076e23)),
              ((1761800001, 999999999), (0.1, 0.2, 0.3, -0.1, -0.2, -0.3)),
              ((1761800002, 1), (2.0 ** -30, -(2.0 ** 30), 0.0, -0.0, 7.0, -7.0))]
PFC0_DIAG = [((1761800003, 500000000), 1, "pfc0 one", [("schema_version", "1"), ("empty", ""), ("k", "v" * 200)]),
             ((1761800004, 7), 3, "pfc0 two", [("a", "1"), ("b", "2;3"), ("c", "x=y")])]

lock = threading.Lock()
out = None


def write(kind, **kw):
    kw.update({"kind": kind, "t": time.time()})
    with lock:
        out.write(json.dumps(kw) + "\n")
        out.flush()


def hdr(h):
    return {"seq": h.seq, "stamp": [h.stamp.secs, h.stamp.nsecs], "frame_id": h.frame_id}


def caller(m):
    return m._connection_header.get("callerid", "") if getattr(m, "_connection_header", None) else ""


def level_int(lvl):
    return lvl if isinstance(lvl, int) else ord(lvl)


def on_cmd(m):
    t = m.twist
    write("cmd", header=hdr(m.header), callerid=caller(m),
          twist=[t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z])


def on_status(m):
    write("robot_status", header=hdr(m.header), callerid=caller(m), status=[
        {"level": level_int(s.level), "name": s.name, "message": s.message, "hardware_id": s.hardware_id,
         "values": [[kv.key, kv.value] for kv in s.values]} for s in m.status])


def on_odom(m):
    p, q, tw = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
    write("odom", header=hdr(m.header), callerid=caller(m), child_frame_id=m.child_frame_id,
          pose=[p.x, p.y, p.z, q.x, q.y, q.z, q.w],
          twist=[tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.x, tw.angular.y, tw.angular.z],
          pose_cov=list(m.pose.covariance), twist_cov=list(m.twist.covariance))


def on_imu(m):
    o, w, a = m.orientation, m.angular_velocity, m.linear_acceleration
    write("imu", header=hdr(m.header), callerid=caller(m), orientation=[o.x, o.y, o.z, o.w],
          angular_velocity=[w.x, w.y, w.z], linear_acceleration=[a.x, a.y, a.z],
          orientation_cov=list(m.orientation_covariance))


def main():
    global out
    mode, run_s, path = sys.argv[1], float(sys.argv[2]), sys.argv[3]
    if mode not in ("listen", "inject"):
        sys.exit("mode must be listen or inject")
    out = open(path, "a")
    rospy.init_node("phase1_ros1_probe", anonymous=False)
    me = rospy.get_name()
    write("start", mode=mode, node=me, python=sys.version.split()[0])
    rospy.Subscriber("/go2/cmd_vel_stamped", TwistStamped, on_cmd, queue_size=50)
    rospy.Subscriber("/go2/robot_status", DiagnosticArray, on_status, queue_size=50)
    if mode == "inject":
        rospy.Subscriber("/go2/odom", Odometry, on_odom, queue_size=50)
        rospy.Subscriber("/go2/imu", Imu, on_imu, queue_size=50)
    p_cmd = p_diag = None
    if mode == "inject":
        p_cmd = rospy.Publisher("/go2/cmd_vel_stamped", TwistStamped, queue_size=10)
        p_diag = rospy.Publisher("/go2/robot_status", DiagnosticArray, queue_size=10)
    t0 = time.time()
    plan = []
    if mode == "inject":
        plan = [(14.0 + i, "cmd", x) for i, x in enumerate(PFC0_TWIST)] + \
               [(17.0 + i, "diag", x) for i, x in enumerate(PFC0_DIAG)]
    while not rospy.is_shutdown() and time.time() - t0 < run_s:
        while plan and time.time() - t0 >= plan[0][0]:
            _, what, x = plan.pop(0)
            if what == "cmd":
                (sec, ns), tw = x
                m = TwistStamped()
                m.header.stamp = rospy.Time(sec, ns)
                m.header.frame_id = "pfc0_from_ros1"
                (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z,
                 m.twist.angular.x, m.twist.angular.y, m.twist.angular.z) = tw
                p_cmd.publish(m)
                write("pfc0_sent", what="cmd", header=hdr(m.header), twist=list(tw), node=me)
            else:
                (sec, ns), level, message, values = x
                st = DiagnosticStatus(level=level, name="go2", message=message, hardware_id="PFC0-FROM-ROS1",
                                      values=[KeyValue(key=k, value=v) for k, v in values])
                m = DiagnosticArray(status=[st])
                m.header.stamp = rospy.Time(sec, ns)
                p_diag.publish(m)
                write("pfc0_sent", what="diag", header=hdr(m.header), level=level, message=message,
                      hardware_id="PFC0-FROM-ROS1", values=[list(kv) for kv in values], node=me)
        time.sleep(0.01)
    write("end")
    out.close()


if __name__ == "__main__":
    main()
