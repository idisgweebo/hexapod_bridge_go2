#!/bin/bash
# ros1_env.sh -- run a command with ROS 1 ONLY (Ubuntu's ROS 1 + /catkin_ws/devel), nothing from Humble.
#   docker run ... bridge_humble:cmd_guard /ros1_env.sh roslaunch cmd_guard cmd_guard.launch
# Why (MEASURED 15 Oct 2026, Claude_Research cmd_guard_humble_port.md section 3): ROS 1's Python is in a system path, so a
# sourced Humble PYTHONPATH shadows rospy's and geometry_msgs' modules; and the image's ROS_DISTRO=humble breaks catkin's
# env hook. env -i drops both. Only the ROS 1 networking variables are passed through.
set -e
keep=(PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin HOME="${HOME:-/root}")
for v in TERM ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROS_LOG_DIR PYTHONUNBUFFERED; do
  [ -n "${!v:-}" ] && keep+=("$v=${!v}")
done
exec env -i "${keep[@]}" bash -c 'source /catkin_ws/devel/setup.bash && exec "$@"' ros1_env "$@"
