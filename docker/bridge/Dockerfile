# Custom bridge image: ros1_bridge base with the cmd_guard safety gate baked in.
FROM ros:foxy-ros1-bridge-focal

# Use bash so we can 'source' ROS setup files during the build.
SHELL ["/bin/bash", "-c"]

# Build tools + ROS 1 deps for the catkin workspace.
# Installing packages that already exist is a harmless no-op.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      ros-noetic-catkin \
      ros-noetic-rospy \
      ros-noetic-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# Copy ONLY the source in, then build it against ROS 1 Noetic.
WORKDIR /catkin_ws
COPY catkin_ws/src /catkin_ws/src
RUN source /opt/ros/noetic/setup.bash && catkin_make

# Source ROS 1 + our workspace automatically on container start.
COPY docker/bridge/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
