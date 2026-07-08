cmake_minimum_required(VERSION 3.0.2)
project(cmd_vel_arbiter)

find_package(catkin REQUIRED)

catkin_package()

# The squelch is a Python 2.7 (Melodic) node. This makes
# `rosrun cmd_vel_arbiter cmd_vel_squelch.py` resolve after build.
catkin_install_python(PROGRAMS
  scripts/cmd_vel_squelch.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Ship launch + config so `roslaunch cmd_vel_arbiter ...` and $(find ...) resolve.
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
