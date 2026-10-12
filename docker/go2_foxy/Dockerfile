# Foxy twin of docker/go2 (12 Oct 2026): run go2_adapter / go2_cmd_bridge and their tests on
# the Orin bridge image's base, ros:foxy-ros1-bridge-focal (Foxy, Ubuntu 20.04, Python 3.8).
# ⚠️ Built on amd64 here; the Orin is arm64 -- a pass here is not an arm64 result.
FROM ros:foxy-ros1-bridge-focal
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-foxy-rmw-cyclonedds-cpp \
      ros-foxy-rosidl-generator-dds-idl \
      ros-foxy-pointcloud-to-laserscan \
      python3-numpy python3-tk xvfb xauth xdotool git \
 && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# unitree_go / unitree_api at the SAME commit as ~/go2_ws (docker/go2/unitree_ros2_commit.txt).
ARG UNITREE_ROS2_COMMIT=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
RUN git clone https://github.com/unitreerobotics/unitree_ros2 /opt/unitree_ros2 \
 && cd /opt/unitree_ros2 && git checkout "$UNITREE_ROS2_COMMIT" \
 && source /opt/ros/foxy/setup.bash \
 && mkdir -p /ws && cd /ws \
 && colcon build --base-paths /opt/unitree_ros2/cyclonedds_ws/src \
      --packages-select unitree_go unitree_api \
 && rm -rf /ws/build /ws/log
