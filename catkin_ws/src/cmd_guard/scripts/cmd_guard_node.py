#!/usr/bin/env python3
import threading
import rospy
from geometry_msgs.msg import Twist


def clamp(value, limit):
    """Clamp value into [-limit, +limit]."""
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


class CmdGuard:
    def __init__(self):
        # --- Parameters: all overridable via YAML/launch, no image rebuild needed ---
        self.input_topic      = rospy.get_param("~input_topic",  "/cmd_vel_ros2_raw")
        self.output_topic     = rospy.get_param("~output_topic", "/cmd_vel_safety_test_out")
        self.max_linear_x     = rospy.get_param("~max_linear_x",  0.03)   # m/s
        self.max_linear_y     = rospy.get_param("~max_linear_y",  0.03)   # m/s
        self.max_angular_z    = rospy.get_param("~max_angular_z", 0.10)   # rad/s
        self.watchdog_timeout = rospy.get_param("~watchdog_timeout", 0.4) # s
        self.publish_rate     = rospy.get_param("~publish_rate", 20.0)    # Hz
        self.enabled          = rospy.get_param("~enabled", True)

        # --- State, guarded by a lock because the subscriber callback runs in a
        #     different thread from the publish loop ---
        self._lock = threading.Lock()
        self._last_cmd = Twist()      # most recent raw command (starts all-zero)
        self._last_stamp = None       # arrival time; None = nothing received yet

        self.pub = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, Twist, self._on_raw_cmd, queue_size=1)

        rospy.loginfo("cmd_guard up. in=%s out=%s limits=(x=%.3f y=%.3f wz=%.3f) "
                      "timeout=%.2fs rate=%.0fHz enabled=%s",
                      self.input_topic, self.output_topic,
                      self.max_linear_x, self.max_linear_y, self.max_angular_z,
                      self.watchdog_timeout, self.publish_rate, self.enabled)
        if self.output_topic == "/cmd_vel":
            rospy.logwarn("cmd_guard output is the REAL /cmd_vel — robot can move. "
                          "Confirm no other publishers and that Y/A arming is understood.")

    def _on_raw_cmd(self, msg):
        # Just record it; the publish loop decides what actually goes out.
        with self._lock:
            self._last_cmd = msg
            self._last_stamp = rospy.Time.now()

    def _build_safe_cmd(self):
        """Decide this tick's output: enable check, watchdog, then clamp."""
        out = Twist()  # all-zero default = safe stop

        if not self.enabled:
            return out

        with self._lock:
            last_cmd = self._last_cmd
            last_stamp = self._last_stamp

        # Watchdog: nothing yet, or the last command is stale -> stop.
        if last_stamp is None:
            return out
        if (rospy.Time.now() - last_stamp).to_sec() > self.watchdog_timeout:
            return out

        # Fresh command: clamp supported axes; everything else stays zero.
        out.linear.x  = clamp(last_cmd.linear.x,  self.max_linear_x)
        out.linear.y  = clamp(last_cmd.linear.y,  self.max_linear_y)
        out.angular.z = clamp(last_cmd.angular.z, self.max_angular_z)
        # linear.z, angular.x, angular.y deliberately left at 0 (unsupported).
        return out

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            safe = self._build_safe_cmd()
            self.pub.publish(safe)
            if safe.linear.x or safe.linear.y or safe.angular.z:
                rospy.loginfo_throttle(0.5, "cmd_guard out: x=%.3f y=%.3f wz=%.3f",
                                       safe.linear.x, safe.linear.y, safe.angular.z)
            rate.sleep()


def main():
    rospy.init_node("cmd_guard")
    CmdGuard().spin()


if __name__ == "__main__":
    main()
