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
Single entry bringup for Lucy: web panel APIs, optional real peripherals, RViz, Gazebo.

Always starts ``web_ros_api`` (rosbridge + lucy_config_pipeline).

Invalid combinations raise at launch parse time (clear ``RuntimeError``).

Arguments:
---------
- ``real`` (default ``true``): micro-ROS agents, USB webcam, RealSense. When false, those
  nodes are not constructed (``OpaqueFunction``), so e.g. Docker without ``micro_ros_agent``
  can run ``real:=false``.
- ``rviz`` (default ``false``): RViz2 (real/sim time set per mode). With ``gazebo:=true``,
  forwarded as ``start_rviz`` to ``thais_urdf/gazebo.launch.py`` (no second RViz).
- ``gazebo`` (default ``false``): GZ Sim stack from ``thais_urdf``; requires ``real:=false``.

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _validate_lucy_launch(context):
    gz = LaunchConfiguration("gazebo").perform(context).lower()
    real = LaunchConfiguration("real").perform(context).lower()

    def _is_true(val):
        return val in ("true", "1", "yes")

    if _is_true(gz) and _is_true(real):
        raise RuntimeError(
            "lucy.launch.py: gazebo:=true conflicts with real:=true. "
            "Use real:=false for simulation (e.g. "
            '"ros2 launch lucy_bringup lucy.launch.py gazebo:=true real:=false").'
        )
    return []


def create_micro_ros_nodes(device0: str, device1: str):
    """Create micro-ROS agent nodes for left and right arms (device paths resolved)."""
    return [
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent_right",
            arguments=["serial", "--dev", device0],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            emulate_tty=True,
        ),
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent_left",
            arguments=["serial", "--dev", device1],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            emulate_tty=True,
        ),
    ]


def _real_hardware_stack(context, *args, **kwargs):
    """Build micro-ROS / camera / RealSense only when ``real`` is true (lazy package load)."""
    real = LaunchConfiguration("real").perform(context).lower().strip()
    if real not in ("true", "1", "yes"):
        return []
    device0 = LaunchConfiguration("device0").perform(context)
    device1 = LaunchConfiguration("device1").perform(context)
    out = list(create_micro_ros_nodes(device0, device1))
    cam_share = get_package_share_directory("camera_ros")
    out.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(cam_share, "launch", "camera.launch.py")
            ),
        )
    )
    lucy_share = get_package_share_directory("lucy_bringup")
    out.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lucy_share, "launch", "realsense.launch.py")
            ),
        )
    )
    return out


def generate_launch_description():
    """Generate launch description for Lucy robot system."""
    device0_arg = DeclareLaunchArgument(
        "device0",
        default_value="/dev/ttyACM0",
        description="Serial device for first micro-ROS agent (right arm)",
    )

    device1_arg = DeclareLaunchArgument(
        "device1",
        default_value="/dev/ttyACM1",
        description="Serial device for second micro-ROS agent (left arm)",
    )

    audio_sample_rate_arg = DeclareLaunchArgument(
        "audio_sample_rate",
        default_value="48000",
        description="Audio sample rate in Hz (e.g., 44100, 48000)",
    )

    audio_capture_device_arg = DeclareLaunchArgument(
        "audio_capture_device",
        default_value="-1",
        description="Audio capture device index (-1 for default)",
    )

    audio_playback_device_arg = DeclareLaunchArgument(
        "audio_playback_device",
        default_value="-1",
        description="Audio playback device index (-1 for default)",
    )

    robot_package_arg = DeclareLaunchArgument(
        "robot_package",
        default_value="thais_urdf",
        description="Robot package: control.launch.py + config paths + RViz config",
    )

    config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value="",
        description=(
            "Override hardware config directory for lucy_config_pipeline "
            "(empty = <robot_package>/config/hardware)"
        ),
    )

    real_arg = DeclareLaunchArgument(
        "real",
        default_value="false",
        description="If true: micro-ROS agents, USB webcam, RealSense",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="If true: RViz (or start_rviz when gazebo:=true)",
    )

    gazebo_arg = DeclareLaunchArgument(
        "gazebo",
        default_value="false",
        description="If true: thais_urdf gazebo sim (requires real:=false)",
    )

    share = get_package_share_directory("thais_urdf")
    default_base = os.path.join(share, "description")
    default_urdf = os.path.join(default_base, "urdf", "inmoov.urdf.xacro")

    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=default_urdf,
        description="Forwarded to thais_urdf gazebo.launch.py when gazebo:=true",
    )
    base_path_arg = DeclareLaunchArgument(
        "base_path",
        default_value=default_base,
        description="Forwarded to thais_urdf gazebo.launch.py when gazebo:=true",
    )
    urdf_path = LaunchConfiguration("urdf_path")
    base_path = LaunchConfiguration("base_path")
    thais_urdf_pkg = get_package_share_directory("thais_urdf")
    controller_config_path = os.path.join(thais_urdf_pkg, "config", "controllers.yaml")

    validate = OpaqueFunction(function=_validate_lucy_launch)

    web_ros_api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("lucy_bringup"),
                        "launch",
                        "web_ros_api.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("robot_package", LaunchConfiguration("robot_package")),
            ("config_dir", LaunchConfiguration("config_dir")),
        ],
    )

    robot_description = Command(
        [
            "xacro ",
            urdf_path,
            " base_path:=",
            base_path,
            " use_gazebo_sim:=true",
            " controller_config:=",
            controller_config_path,
        ]
    )
    robot_description_dict = {"robot_description": robot_description}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_dict, {"use_sim_time": True}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    real_hardware = OpaqueFunction(function=_real_hardware_stack)
    use_sim_time = LaunchConfiguration("gazebo")

    rviz = GroupAction(
        condition=IfCondition(LaunchConfiguration("rviz")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("thais_urdf"),
                                "launch",
                                "rviz.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments=[
                    ("use_sim_time", use_sim_time),
                ],
            ),
        ],
    )

    gazebo = GroupAction(
        condition=IfCondition(LaunchConfiguration("gazebo")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("thais_urdf"),
                                "launch",
                                "gazebo.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments=[
                    ("urdf_path", LaunchConfiguration("urdf_path")),
                    ("base_path", LaunchConfiguration("base_path")),
                    ("robot_package", LaunchConfiguration("robot_package")),
                    ("use_sim_time", use_sim_time),
                ],
            ),
        ],
    )

    ros2_control_launch = GroupAction(
        condition=UnlessCondition(LaunchConfiguration("gazebo")),
        actions=[
            TimerAction(
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare(
                                            LaunchConfiguration("robot_package")
                                        ),
                                        "launch",
                                        "control.launch.py",
                                    ]
                                )
                            ]
                        ),
                    ),
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            device0_arg,
            device1_arg,
            audio_sample_rate_arg,
            audio_capture_device_arg,
            audio_playback_device_arg,
            robot_package_arg,
            config_dir_arg,
            real_arg,
            rviz_arg,
            gazebo_arg,
            urdf_path_arg,
            base_path_arg,
            validate,
            LogInfo(msg="========================================"),
            LogInfo(msg="Starting lucy_bringup lucy.launch.py"),
            LogInfo(msg="========================================"),
            web_ros_api_launch,
            real_hardware,
            ros2_control_launch,
            robot_state_publisher,
            rviz,
            gazebo,
            LogInfo(msg="========================================"),
        ]
    )
