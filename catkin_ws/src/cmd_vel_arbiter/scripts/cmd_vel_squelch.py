#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# cmd_vel_squelch.py
# 4F-2: activity-gate ("squelch") node for the hexapod twist_mux arbitration.
#
# Re-publishes the joystick's Twist to the output topic ONLY while the command
# is meaningfully nonzero, plus a short hold-over after it goes to zero. When the
# hold-over expires it goes SILENT, so the joystick input in twist_mux goes stale
# and the ROS 2 path (cmd_guard) resumes driving. This is what makes
# "release stick -> ROS 2 resumes" work.

import rospy
from geometry_msgs.msg import Twist


class CmdVelSquelch(object):
    def __init__(self):
        # --- parameters (overridable via launch / rosparam) ---
        # input: the joystick's raw Twist (its cmd_vel, runtime-remapped to this)
        self.in_topic  = rospy.get_param("~input_topic",  "/cmd_vel_joy_raw")
        # output: feeds twist_mux's "joystick" handler (/cmd_vel_joy)
        self.out_topic = rospy.get_param("~output_topic", "/cmd_vel_joy")
        # deadband: |component| <= epsilon is treated as zero (kills float noise)
        self.epsilon   = rospy.get_param("~deadband", 0.01)
        # hold-over (s): keep forwarding zeros this long after the last nonzero,
        # so a momentary stick-center mid-motion doesn't hand control back.
        self.hold_over = rospy.get_param("~hold_over", 0.3)

        # last time we saw a genuinely nonzero command
        self.last_active = rospy.Time(0)

        self.pub = rospy.Publisher(self.out_topic, Twist, queue_size=1)
        self.sub = rospy.Subscriber(self.in_topic, Twist, self.cb, queue_size=1)

        rospy.loginfo(
            "cmd_vel_squelch: %s -> %s | deadband=%.3f hold_over=%.2fs",
            self.in_topic, self.out_topic, self.epsilon, self.hold_over)

    def is_nonzero(self, t):
        # True if ANY twist component exceeds the deadband.
        return (abs(t.linear.x)  > self.epsilon or
                abs(t.linear.y)  > self.epsilon or
                abs(t.linear.z)  > self.epsilon or
                abs(t.angular.x) > self.epsilon or
                abs(t.angular.y) > self.epsilon or
                abs(t.angular.z) > self.epsilon)

    def cb(self, msg):
        now = rospy.Time.now()

        if self.is_nonzero(msg):
            # Active command: forward it and reset the hold-over clock.
            self.last_active = now
            self.pub.publish(msg)
            return

        # Command is (near) zero. Still inside the hold-over window?
        if (now - self.last_active).to_sec() < self.hold_over:
            # Yes: keep the joystick "active" in the mux, commanding zero.
            self.pub.publish(msg)
        # Else: stay SILENT. Joystick input goes stale -> mux hands back to ROS 2.


def main():
    rospy.init_node("cmd_vel_squelch")
    CmdVelSquelch()
    rospy.spin()


if __name__ == "__main__":
    main()
