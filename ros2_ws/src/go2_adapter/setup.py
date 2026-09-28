from glob import glob
import os

from setuptools import setup

package_name = 'go2_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # config and launch are INSTALLED, not read from the source tree, so that a
        # deployed container has them without the repo mounted. The launch file
        # resolves the yaml through get_package_share_directory for the same reason.
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Doug Dial',
    maintainer_email='dmdial96@gmail.com',
    description='Go2 DDS sensor topics -> standard stamped ROS 2 messages. Read-only.',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Executable names match the node names, and the node names match the
            # top-level keys in config/go2_adapter.yaml. ROS 2 matches parameter
            # blocks by NODE NAME, so a rename here silently drops every parameter
            # and the node comes up on its defaults instead. Change all three or none.
            'go2_clock_offset_node = go2_adapter.clock_offset_node:main',
            'go2_lidar_adapter_node = go2_adapter.lidar_adapter_node:main',
            'go2_link_watchdog_node = go2_adapter.link_watchdog_node:main',
        ],
    },
)
