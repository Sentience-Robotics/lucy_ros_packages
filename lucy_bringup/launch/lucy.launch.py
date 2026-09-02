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
  forwarded as ``start_rviz`` to ``inmoov_urdf/gazebo.launch.py`` (no second RViz).
- ``gazebo`` (default ``false``): GZ Sim stack from ``inmoov_urdf``; requires ``real:=false``.
- ``headless`` (default ``false``): only meaningful with ``gazebo:=true``. Runs gz-sim
  server-only with EGL rendering (``-s -r --headless-rendering``) so camera sensors
  keep producing frames without an X server. Forwarded to ``inmoov_urdf/gazebo.launch.py``.

"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _infer_robot_source_root(robot_package: str, share_dir: str) -> Path:
    """Prefer workspace src tree (pipeline writes) over install share."""
    cwd_candidate = Path.cwd() / 'src' / robot_package
    if cwd_candidate.is_dir():
        return cwd_candidate
    share = Path(share_dir)
    for parent in share.parents:
        if parent.name == 'install':
            src_candidate = parent.parent / 'src' / robot_package
            if src_candidate.is_dir():
                return src_candidate
    return share


def _discover_robot_packages():
    """
    List installed robot-description packages (launch/control.launch.py + description/).

    Identifies URDF/robot packages via their shared
    layout, so the bringup can auto-select when only one is present.
    """
    from ament_index_python.packages import get_packages_with_prefixes

    found = []
    for pkg, prefix in get_packages_with_prefixes().items():
        share = Path(prefix) / 'share' / pkg
        if (share / 'launch' / 'control.launch.py').is_file() and (
            share / 'description'
        ).is_dir():
            found.append(pkg)
    return sorted(found)


def _default_robot_package():
    """
    Auto-pick the sole robot package; prefer inmoov_urdf when several are built.

    Returns an empty string when none is discovered, leaving ``robot_package``
    unset so ``_validate_lucy_launch`` can fail with a clear message instead of
    silently assuming a package that is not installed.
    """
    pkgs = _discover_robot_packages()
    if len(pkgs) == 1:
        return pkgs[0]
    if 'inmoov_urdf' in pkgs:
        return 'inmoov_urdf'
    if pkgs:
        return pkgs[0]
    return ''


def _resolve_robot_paths(context):
    """
    Fill urdf_path / base_path / controllers_yaml from the selected robot_package.

    Lets ``robot_package:=<pkg>`` switch the URDF, base meshes and controllers
    together. Explicit ``urdf_path`` / ``base_path`` / ``controllers_yaml``
    overrides (non-empty) are left untouched.
    """
    from launch.actions import SetLaunchConfiguration

    robot_package = LaunchConfiguration('robot_package').perform(context).strip()
    if not robot_package:
        # _validate_lucy_launch already raised; nothing to resolve.
        return []
    share = get_package_share_directory(robot_package)
    robot_root = _infer_robot_source_root(robot_package, share)

    defaults = {
        'urdf_path': str(robot_root / 'description' / 'urdf' / 'inmoov.urdf.xacro'),
        # Goes into a file:// URI in the xacro, so it must be posix.
        'base_path': (robot_root / 'description').as_posix(),
        'controllers_yaml': str(robot_root / 'config' / 'controllers.yaml'),
    }
    actions = []
    for key, default_value in defaults.items():
        if not LaunchConfiguration(key).perform(context).strip():
            actions.append(SetLaunchConfiguration(key, default_value))
    return actions


def _validate_lucy_launch(context):
    gz = LaunchConfiguration('gazebo').perform(context).lower()
    real = LaunchConfiguration('real').perform(context).lower()
    robot_package = LaunchConfiguration('robot_package').perform(context).strip()

    if not robot_package:
        raise RuntimeError(
            'lucy.launch.py: no robot-description package found in the workspace '
            '(expected one with launch/control.launch.py + description/, e.g. '
            'inmoov_urdf). Clone/build a robot package, or pass one '
            'explicitly with robot_package:=<pkg>.'
        )

    def _is_true(val):
        return val in ('true', '1', 'yes')

    if _is_true(gz) and _is_true(real):
        raise RuntimeError(
            'lucy.launch.py: gazebo:=true conflicts with real:=true. '
            'Use real:=false for simulation (e.g. '
            '"ros2 launch lucy_bringup lucy.launch.py gazebo:=true real:=false").'
        )
    return []


def create_micro_ros_nodes(device0: str, device1: str):
    """Create micro-ROS agent nodes for left and right arms (device paths resolved)."""
    return [
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_right',
            arguments=['serial', '--dev', device0],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            emulate_tty=True,
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_left',
            arguments=['serial', '--dev', device1],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            emulate_tty=True,
        ),
    ]


def _real_hardware_stack(context, *args, **kwargs):
    """Build micro-ROS / camera / RealSense only when ``real`` is true (lazy package load)."""
    real = LaunchConfiguration('real').perform(context).lower().strip()
    if real not in ('true', '1', 'yes'):
        return []
    device0 = LaunchConfiguration('device0').perform(context)
    device1 = LaunchConfiguration('device1').perform(context)
    out = list(create_micro_ros_nodes(device0, device1))
    cam_share = get_package_share_directory('camera_ros')
    out.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(cam_share, 'launch', 'camera.launch.py')
            ),
        )
    )
    lucy_share = get_package_share_directory('lucy_bringup')
    out.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lucy_share, 'launch', 'realsense.launch.py')
            ),
        )
    )
    return out


def generate_launch_description():
    """Generate launch description for Lucy robot system."""
    device0_arg = DeclareLaunchArgument(
        'device0',
        default_value='/dev/ttyACM0',
        description='Serial device for first micro-ROS agent (right arm)',
    )

    device1_arg = DeclareLaunchArgument(
        'device1',
        default_value='/dev/ttyACM1',
        description='Serial device for second micro-ROS agent (left arm)',
    )

    audio_sample_rate_arg = DeclareLaunchArgument(
        'audio_sample_rate',
        default_value='48000',
        description='Audio sample rate in Hz (e.g., 44100, 48000)',
    )

    audio_capture_device_arg = DeclareLaunchArgument(
        'audio_capture_device',
        default_value='-1',
        description='Audio capture device index (-1 for default)',
    )

    audio_playback_device_arg = DeclareLaunchArgument(
        'audio_playback_device',
        default_value='-1',
        description='Audio playback device index (-1 for default)',
    )

    robot_package_arg = DeclareLaunchArgument(
        'robot_package',
        default_value=_default_robot_package(),
        description=(
            'Robot package: control.launch.py + config paths + RViz config + URDF. '
            'Defaults to the only installed robot package when just one is present, '
            'else inmoov_urdf.'
        ),
    )

    config_dir_arg = DeclareLaunchArgument(
        'config_dir',
        default_value='',
        description=(
            'Override hardware config directory for lucy_config_pipeline '
            '(empty = <robot_package>/config/hardware)'
        ),
    )

    real_arg = DeclareLaunchArgument(
        'real',
        default_value='false',
        description='If true: micro-ROS agents, USB webcam, RealSense',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='If true: RViz (or start_rviz when gazebo:=true)',
    )

    gazebo_arg = DeclareLaunchArgument(
        'gazebo',
        default_value='false',
        description='If true: <robot_package> gazebo sim (requires real:=false)',
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description=(
            'Only with gazebo:=true. Server-only gz-sim with EGL rendering '
            '(-s -r --headless-rendering); cameras still work without an X server.'
        ),
    )

    # Empty defaults: _resolve_robot_paths fills these from the selected
    # robot_package at launch time, so robot_package:=<pkg> switches the URDF,
    # base meshes and controllers together. Non-empty overrides are respected.
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='',
        description=(
            'URDF/xacro entry. Empty -> '
            '<robot_package>/description/urdf/inmoov.urdf.xacro'
        ),
    )
    base_path_arg = DeclareLaunchArgument(
        'base_path',
        default_value='',
        description='xacro base_path. Empty -> <robot_package>/description',
    )
    urdf_path = LaunchConfiguration('urdf_path')
    base_path = LaunchConfiguration('base_path')
    controllers_yaml_arg = DeclareLaunchArgument(
        'controllers_yaml',
        default_value='',
        description=(
            'controllers.yaml path. Empty -> <robot_package>/config/controllers.yaml'
        ),
    )

    validate = OpaqueFunction(function=_validate_lucy_launch)
    resolve_robot_paths = OpaqueFunction(function=_resolve_robot_paths)

    web_ros_api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('lucy_bringup'),
                        'launch',
                        'web_ros_api.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments=[
            ('robot_package', LaunchConfiguration('robot_package')),
            ('config_dir', LaunchConfiguration('config_dir')),
        ],
    )

    controllers_yaml = LaunchConfiguration('controllers_yaml')

    # use_mock_hardware := (not gazebo) and (not real) — RViz-only mock_components path.
    use_mock_hardware = PythonExpression(
        [
            "'true' if ('",
            LaunchConfiguration('gazebo'),
            "'.lower() not in ('true','1','yes') and '",
            LaunchConfiguration('real'),
            "'.lower() not in ('true','1','yes')) else 'false'",
        ]
    )

    # Force value_type=str so ROS 2 launch does not try to YAML-parse
    # the xacro output. The URDF starts with `<?xml ...>`, which the YAML
    # loader rejects with "Unable to parse the value of parameter robot_description".
    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                urdf_path,
                ' base_path:=',
                base_path,
                ' use_gazebo_sim:=',
                LaunchConfiguration('gazebo'),
                ' use_mock_hardware:=',
                use_mock_hardware,
                ' controller_config:=',
                controllers_yaml,
            ]
        ),
        value_type=str,
    )
    robot_description_dict = {'robot_description': robot_description}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description_dict,
            {'use_sim_time': LaunchConfiguration('gazebo')},
        ],
        condition=IfCondition(LaunchConfiguration('gazebo')),
    )

    real_hardware = OpaqueFunction(function=_real_hardware_stack)
    use_sim_time = LaunchConfiguration('gazebo')

    rviz = GroupAction(
        condition=IfCondition(LaunchConfiguration('rviz')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare(LaunchConfiguration('robot_package')),
                            'launch',
                            'rviz.launch.py',
                        ]
                    )
                ]
                ),
                launch_arguments=[
                    ('use_sim_time', use_sim_time),
                ],
            ),
        ],
    )

    gazebo = GroupAction(
        condition=IfCondition(LaunchConfiguration('gazebo')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare(LaunchConfiguration('robot_package')),
                                'launch',
                                'gazebo.launch.py',
                            ]
                        )
                    ]
                ),
                launch_arguments=[
                    ('urdf_path', LaunchConfiguration('urdf_path')),
                    ('base_path', LaunchConfiguration('base_path')),
                    ('controllers_yaml', controllers_yaml),
                    ('headless', LaunchConfiguration('headless')),
                ],
            ),
        ],
    )

    ros2_control_launch = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('gazebo')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare(LaunchConfiguration('robot_package')),
                                'launch',
                                'control.launch.py',
                            ]
                        )
                    ]
                ),
                launch_arguments=[
                    ('urdf_path', LaunchConfiguration('urdf_path')),
                    ('base_path', LaunchConfiguration('base_path')),
                    ('controllers_yaml', controllers_yaml),
                    ('use_mock_hardware', use_mock_hardware),
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
            headless_arg,
            urdf_path_arg,
            base_path_arg,
            controllers_yaml_arg,
            validate,
            resolve_robot_paths,
            LogInfo(msg='========================================'),
            LogInfo(msg='Starting lucy_bringup lucy.launch.py'),
            LogInfo(msg='========================================'),
            web_ros_api_launch,
            real_hardware,
            ros2_control_launch,
            robot_state_publisher,
            rviz,
            gazebo,
            LogInfo(msg='========================================'),
        ]
    )
