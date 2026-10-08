from glob import glob
import os

from setuptools import setup

package_name = 'go2_cmd_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Doug Dial',
    maintainer_email='dmdial96@gmail.com',
    description='/go2/cmd_vel -> streamed Move on the Go2. Dry run unless armed. Gate 7.',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Executable name = node name = YAML top-level key. Change all three or none.
            'go2_cmd_bridge_node = go2_cmd_bridge.bridge_node:main',
        ],
    },
)
