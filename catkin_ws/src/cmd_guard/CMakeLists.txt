cmake_minimum_required(VERSION 3.0.2)
project(cmd_guard)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/cmd_guard_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
