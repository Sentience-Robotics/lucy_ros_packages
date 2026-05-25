#!/usr/bin/env python3
# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Start lucy_control_supervisor (RSP + ros2_control + spawners, /lucy_control/restart)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("urdf_path", default_value=""),
            DeclareLaunchArgument("base_path", default_value=""),
            DeclareLaunchArgument("controllers_yaml", default_value=""),
            DeclareLaunchArgument("use_gazebo_sim", default_value="false"),
            DeclareLaunchArgument("gazebo_only", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            Node(
                package="lucy_control_supervisor",
                executable="control_supervisor_node",
                name="lucy_control_supervisor",
                output="screen",
                parameters=[
                    {
                        "urdf_path": LaunchConfiguration("urdf_path"),
                        "base_path": LaunchConfiguration("base_path"),
                        "controllers_yaml": LaunchConfiguration("controllers_yaml"),
                        "use_gazebo_sim": LaunchConfiguration("use_gazebo_sim"),
                        "gazebo_only": LaunchConfiguration("gazebo_only"),
                        "autostart": LaunchConfiguration("autostart"),
                    }
                ],
            ),
        ]
    )
