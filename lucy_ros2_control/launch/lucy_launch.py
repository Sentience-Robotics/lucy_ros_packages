from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
import os


def generate_launch_description():
    urdf_path = '/home/dev/thais_urdf/InMoov/urdf/InMoov.urdf.xacro'

    controllers_yaml = '/home/dev/lucy_ws/src/lucy_ros_packages/lucy_ros2_control/config/lucy_controllers.yaml'

    # controllers_yaml = PathJoinSubstitution([
        # FindPackageShare('lucy_ros2_control'),
        # 'config',
        # 'lucy_controllers.yaml'
    # ])

    ros2_control_node = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[controllers_yaml]
        )]
    )

    return LaunchDescription([

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
        ),

        # ros2_control_node
        ros2_control_node,

        # Spawner nodes
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['left_arm_controller'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['right_arm_controller'],
            output='screen'
        )
    ])
