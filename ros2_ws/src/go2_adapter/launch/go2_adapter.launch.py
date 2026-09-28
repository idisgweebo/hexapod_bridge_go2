#!/usr/bin/env python3
"""Bring up the Go2 sensor adapter. READ-ONLY with respect to the robot.

Launches, all of them subscribe-and-republish only:

    go2_clock_offset_node    /utlidar/robot_pose -> /go2/clock_offset[_state]
    go2_lidar_adapter_node   /utlidar/cloud      -> /go2/lidar/points
    go2_link_watchdog_node   /utlidar/robot_pose -> /go2/link_ok
    pointcloud_to_laserscan  /go2/lidar/points   -> /go2/scan

WHY THIS FILE CAN REFUSE TO START
---------------------------------
config/go2_adapter.yaml deliberately omits pointcloud_to_laserscan's min_height and
max_height. They select the horizontal slab of the point cloud that becomes the 2D
scan, and they cannot be chosen until /utlidar/cloud's frame and Z convention are
MEASURED (open item O-1 -- and note that not even the cloud's frame_id has ever
been recorded).

A guessed slab does not fail. It produces a LaserScan that looks entirely plausible
and is wrong: ranges to whatever happened to fall in the wrong height band, fed to
navigation. That is the worst failure mode available here, so the launch refuses
rather than defaulting.

pointcloud_to_laserscan's own defaults are min_height=0.0, max_height=1.0, which
would silently apply if we said nothing -- a default is not an absence.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

PKG = 'go2_adapter'
P2L_NODE = 'pointcloud_to_laserscan_node'
REQUIRED_P2L_PARAMS = ('min_height', 'max_height')


def _load_config():
    path = os.path.join(get_package_share_directory(PKG), 'config', 'go2_adapter.yaml')
    if not os.path.isfile(path):
        raise RuntimeError(
            'go2_adapter: config not found at %s. It is installed by setup.py from '
            'config/*.yaml -- if this is a fresh checkout, colcon build first.' % path)
    with open(path, encoding='utf-8') as fh:
        return path, yaml.safe_load(fh)


def _check_projection_is_configured(path, cfg):
    """Refuse to launch unless the unmeasurable slab bounds have been supplied.

    Checked here rather than inside pointcloud_to_laserscan because that is stock
    upstream code we do not own and will not patch, and because a launch-time refusal
    is louder and earlier than a node that comes up on defaults.
    """
    params = ((cfg or {}).get(P2L_NODE) or {}).get('ros__parameters') or {}
    missing = [k for k in REQUIRED_P2L_PARAMS if k not in params]
    if missing:
        raise RuntimeError(
            'go2_adapter: refusing to launch. %s is missing %s in %s.\n'
            '\n'
            'These are DELIBERATELY absent, not forgotten. They select the height slab\n'
            'of the cloud that becomes /go2/scan, and /utlidar/cloud\'s frame and Z\n'
            'convention have never been measured (open item O-1). A guessed slab\n'
            'produces a scan that looks plausible and is wrong, which is worse than no\n'
            'scan at all.\n'
            '\n'
            'To proceed: measure the cloud frame and Z convention on a live robot,\n'
            'record it in inventory-go2_ros2.md, then set both values here and update\n'
            'scripts/test_go2_scan_geometry_offline.py, which asserts their absence.'
            % (P2L_NODE, ', '.join(missing), path))


def generate_launch_description():
    config_path, cfg = _load_config()
    _check_projection_is_configured(config_path, cfg)

    common = dict(package=PKG, output='screen', parameters=[config_path])

    return LaunchDescription([
        # Node NAMES must match the yaml's top-level keys -- ROS 2 matches parameter
        # blocks by node name, and a mismatch silently yields a node on its defaults.
        Node(executable='go2_clock_offset_node', name='go2_clock_offset_node', **common),
        Node(executable='go2_lidar_adapter_node', name='go2_lidar_adapter_node', **common),
        Node(executable='go2_link_watchdog_node', name='go2_link_watchdog_node', **common),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name=P2L_NODE,
            output='screen',
            parameters=[config_path],
            # Stock node, no projection code of our own. Its input is our OUTPUT, so
            # it consumes clouds that are already correctly stamped and framed -- it
            # must never be pointed at /utlidar/cloud directly, or it would project
            # Go2-clock stamps that every consumer rejects as 23 minutes stale.
            remappings=[('cloud_in', '/go2/lidar/points'), ('scan', '/go2/scan')],
        ),
    ])
