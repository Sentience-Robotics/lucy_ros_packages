#!/usr/bin/env python3
# Launch Lucy ros2_control with Gazebo (gz sim) and RViz.
# Paths are resolved relative to the workspace containing this package (host-independent).

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _get_workspace_relative_paths():
    """Resolve thais_urdf paths from this launch file's location.
    Works when launch is run from source (src/.../launch/) or from install (install/.../share/.../launch/).
    """
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    # From source: launch -> lucy_ros2_control -> lucy_ros_packages -> src -> workspace (4 up)
    # From install: launch -> lucy_ros2_control -> share -> lucy_ros2_control -> install -> workspace (5 up)
    if os.path.sep + 'install' + os.path.sep in launch_dir or launch_dir.endswith(os.path.sep + 'install'):
        workspace = os.path.normpath(os.path.join(launch_dir, '..', '..', '..', '..', '..'))
    else:
        workspace = os.path.normpath(os.path.join(launch_dir, '..', '..', '..', '..'))
    urdf_path = os.path.join(workspace, 'src', 'thais_urdf', 'inmoov', 'urdf', 'inmoov.urdf.xacro')
    base_path = os.path.join(workspace, 'src', 'thais_urdf', 'inmoov')
    return urdf_path, base_path


def generate_launch_description():
    default_urdf, default_base = _get_workspace_relative_paths()
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=default_urdf,
        description='Path to inmoov.urdf.xacro',
    )
    base_path_arg = DeclareLaunchArgument(
        'base_path',
        default_value=default_base,
        description='Base path for xacro (mesh_dir); overrides auto-detected path if set',
    )

    urdf_path = LaunchConfiguration('urdf_path')
    base_path = LaunchConfiguration('base_path')

    # robot_description with base_path for mesh resolution in Docker
    robot_description = Command(['xacro ', urdf_path, ' base_path:=', base_path])
    robot_description_dict = {'robot_description': robot_description}

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('lucy_ros2_control'),
        'config',
        'lucy_controllers.yaml',
    ])

    # Gazebo Sim (gz) — empty world
    pkg_gz_sim = 'ros_gz_sim'
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(pkg_gz_sim), 'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments={'gz_args': 'empty.sdf'}.items(),
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_dict],
    )

    # ros2_control_node (delayed so robot_description is available)
    ros2_control_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                output='screen',
                parameters=[controllers_yaml, robot_description_dict],
            )
        ],
    )

    # Controller spawners (after ros2_control_node)
    spawn_joint_state = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            )
        ],
    )
    spawn_left_arm = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['left_arm_controller'],
                output='screen',
            )
        ],
    )
    spawn_right_arm = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['right_arm_controller'],
                output='screen',
            )
        ],
    )

    # RViz — RobotModel + Fixed Frame base_node (config preloads so robot is visible)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('lucy_ros2_control'),
        'config',
        'lucy_rviz.rviz',
    ])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config],
    )

    # Spawn robot model in Gazebo (from robot_description param)
    spawn_lucy_gazebo = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_lucy',
                arguments=['-name', 'lucy', '-param', 'robot_description', '-z', '0.5'],
                parameters=[robot_description_dict],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        urdf_path_arg,
        base_path_arg,
        gz_sim_launch,
        robot_state_publisher,
        spawn_lucy_gazebo,
        ros2_control_node,
        spawn_joint_state,
        spawn_left_arm,
        spawn_right_arm,
        rviz,
    ])
