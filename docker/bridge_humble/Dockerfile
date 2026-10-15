# EXPERIMENT (13 Oct 2026): ros1_bridge on Humble, so the whole ROS 2 side is one distro.
# There is no official Humble bridge image (ros:humble-ros1-bridge does not exist) and no Noetic for
# Jammy, so ROS 1 comes from UBUNTU'S OWN archive (universe: roscpp 1.15.14, *_msgs 1.13.1) and
# ros1_bridge is built from source at a PINNED commit. Recipe follows TommyChangUMD/ros-humble-ros1-
# bridge-builder, minus its tarball step. Predictions: Claude_Research humble_ros1_bridge_check.md.
# 15 Oct 2026: + cmd_guard (catkin, ROS 1) -- Claude_Research cmd_guard_humble_port.md.
# Build ONLY with:  docker build --network host -t bridge_humble:cmd_guard -f docker/bridge_humble/Dockerfile .
# Run ROS 1 Python nodes (cmd_guard) ONLY via /ros1_env.sh -- never in a shell with Humble sourced (see below).
FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]

# Upstream master by default (PHB-0). Fallback: --build-arg ROS1_BRIDGE_REPO=https://github.com/smith-doug/ros1_bridge.git
#   --build-arg ROS1_BRIDGE_COMMIT=8f1b7e903e1fb4462218db659bb4981228c12206  (the builder's fork, PHB-0b)
ARG ROS1_BRIDGE_REPO=https://github.com/ros2/ros1_bridge.git
ARG ROS1_BRIDGE_COMMIT=611755fd917285316051cbea80507e8b2f6b7ec1

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rmw-cyclonedds-cpp git build-essential pkg-config \
 && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ROS 1 from Ubuntu universe, with the ROS 2 apt source hidden so apt cannot mix the two.
# MEASURED 13 Oct: Humble's python3-catkin-pkg-modules declares "Conflicts: catkin" and blocks this.
# ros-tf2-msgs: MEASURED 13 Oct, build 1 -- libtf2-msgs-dev ships /usr/share/tf2_msgs WITHOUT msg/, so ros1_bridge
# paired no tf2_msgs type at all (print-pairs 129 vs Foxy's 166). The hexapod's /tf is tf2_msgs/TFMessage.
# Workaround (the builder's): strip that Conflicts line from dpkg's status db, then force-overwrite
# Ubuntu's python3-catkin-pkg / -rospkg / -rosdistro over the *-modules files. A dpkg-database edit --
# recorded, not hidden; this image is an experiment until it is measured.
# python3-geometry-msgs: MEASURED 15 Oct -- ROS 1's Python geometry_msgs was absent (cmd_guard imports it). Must be
# installed HERE, before the dpkg hack leaves apt in a state nothing should be installed into afterwards.
RUN mv /etc/apt/sources.list.d/ros2.sources /root/ && apt-get update \
 && sed -i -e 's|^Conflicts: catkin|#Conflicts: catkin|' /var/lib/dpkg/status \
 && apt-get install -f -y \
 && cd /tmp && apt-get download python3-catkin-pkg python3-rospkg python3-rosdistro \
 && dpkg --force-overwrite -i python3-catkin-pkg_*.deb python3-rospkg_*.deb python3-rosdistro_*.deb \
 && apt-get install -f -y \
 && apt-get install -y --no-install-recommends ros-core-dev libtf2-msgs-dev ros-tf2-msgs python3-geometry-msgs \
 && rm -f /tmp/*.deb && mv /root/ros2.sources /etc/apt/sources.list.d/ \
 && rm -rf /var/lib/apt/lists/*

# arm64 only (the builder's note, UNTESTED here): Ubuntu's ROS 1 .pc files land under the x86_64
# triplet even on arm64. A no-op on the amd64 laptop. INSTRUMENTED 13 Oct (PHB-A4): print where the .pc
# files actually are BEFORE copying, and -v so the log shows what (if anything) the copy did.
RUN echo "PHB-A4 arch=$(uname -m) x86_64_pc=$(ls /usr/lib/x86_64-linux-gnu/pkgconfig/*.pc 2>/dev/null | wc -l)" \
         "aarch64_pc=$(ls /usr/lib/aarch64-linux-gnu/pkgconfig/*.pc 2>/dev/null | wc -l)" \
         "roscpp.pc=[$(ls /usr/lib/*/pkgconfig/roscpp.pc 2>/dev/null | tr '\n' ' ')]" \
 && if [[ $(uname -m) = aarch64 ]]; then cp -nv /usr/lib/x86_64-linux-gnu/pkgconfig/* /usr/lib/aarch64-linux-gnu/pkgconfig/ 2>&1 || true; fi

# ros1_bridge from source. Jobs = min(cores, GB of RAM): ~1 GB per compile job (builder's figure).
WORKDIR /bridge_ws
RUN git clone "$ROS1_BRIDGE_REPO" src/ros1_bridge \
 && git -C src/ros1_bridge checkout --detach "$ROS1_BRIDGE_COMMIT" \
 && git -C src/ros1_bridge log -1 --format='ros1_bridge at %H %cI' | tee /bridge_ws/ROS1_BRIDGE_COMMIT.txt \
 && source /opt/ros/humble/setup.bash \
 && MEMG=$(free -g | awk '/^Mem:/{print $2}'); NPROC=$(nproc); MIN=$(( MEMG < NPROC ? MEMG : NPROC )) \
 && echo "building ros1_bridge with -j$MIN (nproc $NPROC, mem ${MEMG}G)" \
 && time MAKEFLAGS="-j$MIN" colcon build --packages-select ros1_bridge --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf build log

# cmd_guard (+ cmd_vel_arbiter, which only needs catkin to build) against Ubuntu's ROS 1.
# MEASURED 15 Oct: the base image bakes ENV ROS_DISTRO=humble; Ubuntu's /etc/catkin/profile.d/1.ros_distro.sh then
# prints a "ROS_DISTRO was set to 'humble'" warning to STDOUT, which catkin's generate_cached_setup.py parses as
# Python -> SyntaxError -> configure fails. Hence env -u. catkin_make repairs the host's dangling toplevel symlink itself.
WORKDIR /catkin_ws
COPY catkin_ws/src /catkin_ws/src
RUN env -u ROS_DISTRO catkin_make

# MEASURED 15 Oct: ROS 1's Python is in /usr/lib/python3/dist-packages (a system path), so ANY sourced Humble PYTHONPATH
# shadows it -- rospy fails on ROS 2's rosgraph_msgs, and geometry_msgs silently resolves to ROS 2's module.
# ros1_env.sh runs a command under env -i with only the ROS 1 workspace sourced.
COPY docker/bridge_humble/ros1_env.sh /ros1_env.sh
RUN chmod +x /ros1_env.sh

# Source ROS 2 + the bridge on start. ROS 1 needs no setup file: Ubuntu installs it into /usr.
RUN echo 'source /opt/ros/humble/setup.bash && source /bridge_ws/install/setup.bash' >> /root/.bashrc
WORKDIR /bridge_ws
CMD ["bash"]
