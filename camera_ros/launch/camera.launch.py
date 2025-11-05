#!/usr/bin/env python3
# Copyright 2024 Sentience Robotics Team
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
Launch file for zero-copy MJPEG camera publisher.

This launch file starts the camera node with optimized settings for NVIDIA Jetson AGX Orin."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='15.0',
        description='Camera frame rate (1.0-30.0 FPS)'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/video0',
        description='Camera device path'
    )

    # Camera node
    camera_node = Node(
        package='camera_ros',
        executable='camera_publisher.py',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'fps': LaunchConfiguration('fps'),
            'device': LaunchConfiguration('device'),
        }],
    )

    return LaunchDescription([
        fps_arg,
        device_arg,
        LogInfo(msg='Starting zero-copy MJPEG camera publisher with automatic client-based activation'),
        camera_node,
    ])
