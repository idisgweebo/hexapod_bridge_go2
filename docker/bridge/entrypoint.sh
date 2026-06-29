#!/bin/bash
set -e
source /opt/ros/foxy/setup.bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
exec "$@"
