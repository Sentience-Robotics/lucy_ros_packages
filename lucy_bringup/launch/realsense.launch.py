#!/usr/bin/env python3
# Copyright 2025 Sentience Robotics Team
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""
Minimal launch file for Intel RealSense D435i camera.

Based on official realsense-ros documentation:
https://github.com/realsenseai/realsense-ros

This launch file uses only required parameters:
- Enable streams (color, depth, IMU)
- Disable unused streams (infrared, pointcloud)
- Namespace: /realsense

All other parameters use defaults from the realsense-ros package.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = {
        # Enable required streams
        'enable_color': True,
        'enable_depth': True,

        # Disable unused streams
        'enable_gyro': False,
        'enable_accel': False,
        'enable_infra1': False,
        'enable_infra2': False,
        'enable_pointcloud': False,
        
        # Enable RGBD topic (combines color and depth into RGB-D image)
        'enable_rgbd': True,
        'align_depth.enable': True,  # Required for RGBD
        'enable_sync': True,  # Required for RGBD synchronization
    }
    
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        namespace='realsense',
        parameters=[params],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )
    
    return LaunchDescription([
        realsense_node,
    ])
