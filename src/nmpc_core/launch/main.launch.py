#!/usr/bin/env python3
"""
Launch file for NMPC Core fault‐tolerant control stack

Starts the following nodes in the nmpc_core package:
  • uORB_listener.py
  • sensor_processor.py
  • nmpc_trigger.py
  • nmpc_core_CASADI.py
  • command_dispatcher.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nmpc_core')

    return LaunchDescription([
        Node(
            package='nmpc_core',
            executable='uORB_listener.py',
            name='uorb_listener',
            output='screen',
            parameters=[{
                # If you have any parameters, e.g. frame IDs or QoS settings
            }],
        ),

        Node(
            package='nmpc_core',
            executable='sensor_processor.py',
            name='sensor_processor',
            output='screen',
            parameters=[{
                # e.g. filter cutoffs, sync tolerance
            }],
        ),

        Node(
            package='nmpc_core',
            executable='nmpc_trigger.py',
            name='nmpc_trigger',
            output='screen',
            parameters=[{
                # e.g. fault debounce time (ms)
            }],
        ),

        Node(
            package='nmpc_core',
            executable='nmpc_core_CASADI.py',
            name='nmpc_core',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'nmpc_params.yaml')
            ],
        ),

        Node(
            package='nmpc_core',
            executable='command_dispatcher.py',
            name='command_dispatcher',
            output='screen',
            parameters=[{
                # e.g. offboard heartbeat rate, timeout
            }],
        ),
    ])
