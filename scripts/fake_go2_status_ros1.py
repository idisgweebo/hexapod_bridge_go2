#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""A fake Go2 side, in ROS 1, for smoke-testing hexapod/go2_feedback_check.py with NO ROS 2
and NO robot. Publishes /go2/robot_status (schema v1) at 5 Hz, echoing the newest
/go2/cmd_vel_stamped stamp it has heard; motion is MOVING while |vx| or |wz| > 0 was
heard in the last 0.4 s. Publishes nothing else.

    fake_go2_status_ros1.py ARMED(true|false) RUN_S
"""
from __future__ import print_function

import sys

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped

ARMED, RUN_S = sys.argv[1], float(sys.argv[2])
state = {"echo": None, "moving_until": 0.0}


def on_cmd(m):
    s = (m.header.stamp.secs, m.header.stamp.nsecs)
    if s != (0, 0) and (state["echo"] is None or s > state["echo"]):      # the bridge's D3 rule
        state["echo"] = s
        if abs(m.twist.linear.x) > 0 or abs(m.twist.angular.z) > 0:
            state["moving_until"] = rospy.get_time() + 0.4


rospy.init_node("fake_go2_status")
pub = rospy.Publisher("/go2/robot_status", DiagnosticArray, queue_size=1)
rospy.Subscriber("/go2/cmd_vel_stamped", TwistStamped, on_cmd)
end = rospy.get_time() + RUN_S
rate = rospy.Rate(5)
while not rospy.is_shutdown() and rospy.get_time() < end:
    moving = rospy.get_time() < state["moving_until"]
    vals = {"schema_version": "1", "motion": "MOVING" if moving else "STOPPED", "fault": "none", "warn": "none",
            "bridge_state": "DRIVING" if moving else "IDLE", "bridge_input": "twist_stamped", "bridge_armed": ARMED,
            "cmd_last_stamp": "%d.%09d" % state["echo"] if state["echo"] else ""}
    st = DiagnosticStatus(level=0, name="go2", message="fake", hardware_id="B42N6000PBCD7AG9",
                          values=[KeyValue(key=k, value=v) for k, v in vals.items()])
    arr = DiagnosticArray(status=[st])
    arr.header.stamp = rospy.Time.now()
    pub.publish(arr)
    rate.sleep()
