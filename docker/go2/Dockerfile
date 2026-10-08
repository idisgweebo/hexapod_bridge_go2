FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-rosidl-generator-dds-idl \
 && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Gate 7 §10 teleop (8 Oct 2026): Tk for go2_teleop.py's window; Xvfb + xdotool so its
# integration test can drive real X key events (incl. server auto-repeat) headless.
# A SEPARATE layer on purpose: editing the RUN above would re-resolve the Cyclone packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-tk xvfb xauth xdotool \
 && rm -rf /var/lib/apt/lists/*
