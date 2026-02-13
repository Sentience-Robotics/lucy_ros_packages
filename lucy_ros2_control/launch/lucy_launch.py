import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _get_workspace_relative_paths():
    """Resolve thais_urdf paths from this launch file's location (workspace/src/.../launch/)."""
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace = os.path.normpath(os.path.join(launch_dir, '..', '..', '..', '..'))
    urdf_path = os.path.join(workspace, 'src', 'thais_urdf', 'inmoov', 'urdf', 'inmoov.urdf.xacro')
    base_path = os.path.join(workspace, 'src', 'thais_urdf', 'inmoov')
    return urdf_path, base_path


def generate_launch_description():
    urdf_path, base_path = _get_workspace_relative_paths()
    # pkg_gazebo = get_package_share_directory('gazebo_ros')

    # Lancement de Gazebo
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
    #     )
    # )

    # Publie le robot_description
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
    # )

    # # Spawn le robot dans Gazebo après 20 secondes
    # spawn_entity = TimerAction(
    #     period=20.0,  # délai en secondes
    #     actions=[
    #         Node(
    #             package='gazebo_ros',
    #             executable='spawn_entity.py',
    #             arguments=[
    #             '-topic', 'robot_description',
    #             '-entity', 'mon_robot',
    #             '-x', '0', '-y', '0', '-z', '0.5'  # sur le sol
    #             ],
    #             output='screen'
    #         )
    #     ]
    # )

    # return LaunchDescription([
    #     gazebo,
    #     robot_state_publisher,
    #     spawn_entity
    # ])



    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('lucy_ros2_control'),
        'config',
        'lucy_controllers.yaml'
    ])

    robot_description_deprecated = {
        'robot_description': Command(['xacro ', urdf_path, ' base_path:=', base_path])
    }


    ros2_control_node = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[controllers_yaml, robot_description_deprecated]
        )]
    )

    return LaunchDescription([

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_path, ' base_path:=', base_path])}]
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
