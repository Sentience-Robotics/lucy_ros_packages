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
Launch file for Lucy Robot System.

This launch file starts all core ROS2 components:
- Two micro-ROS agents (for left and right arm RP2040 controllers)
- ROSBridge WebSocket server (for web interface communication)
- Camera publisher node (for vision system)
- Audio capture and playback nodes (for stereo microphones and speakers)

Optimized for NVIDIA Jetson AGX Orin.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def create_micro_ros_nodes(device0, device1):
    """Create micro-ROS agent nodes for left and right arms."""
    return [
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_right',
            arguments=['serial', '--dev', device0],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            emulate_tty=True
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_left',
            arguments=['serial', '--dev', device1],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            emulate_tty=True
        )
    ]


def create_audio_nodes(sample_rate, capture_device, playback_device):
    """Create audio capture and playback nodes."""
    return [
        Node(
            package='audio_common',
            executable='audio_capturer_node',
            name='audio_capturer',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'format': 8,  # paInt16 (PortAudio format constant)
                'channels': 2,  # Stereo microphones
                'rate': sample_rate,
                'chunk': 1024,  # Buffer size
                'device': capture_device,
                'frame_id': 'audio_capture'
            }]
        ),
        Node(
            package='audio_common',
            executable='audio_player_node',
            name='audio_player',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'channels': 2,  # Stereo speakers
                'device': playback_device
            }]
        )
    ]


def generate_launch_description():
    """Generate launch description for Lucy robot system."""
    
    # Declare launch arguments for flexibility
    device0_arg = DeclareLaunchArgument(
        'device0',
        default_value='/dev/ttyACM0',
        description='Serial device for first micro-ROS agent (right arm)'
    )
    
    device1_arg = DeclareLaunchArgument(
        'device1',
        default_value='/dev/ttyACM1',
        description='Serial device for second micro-ROS agent (left arm)'
    )
    
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'
    )
    
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='15.0',
        description='Camera frame rate (1.0-30.0 FPS)'
    )
    
    # Audio launch arguments
    audio_sample_rate_arg = DeclareLaunchArgument(
        'audio_sample_rate',
        default_value='48000',
        description='Audio sample rate in Hz (e.g., 44100, 48000)'
    )
    
    audio_capture_device_arg = DeclareLaunchArgument(
        'audio_capture_device',
        default_value='-1',
        description='Audio capture device index (-1 for default)'
    )
    
    audio_playback_device_arg = DeclareLaunchArgument(
        'audio_playback_device',
        default_value='-1',
        description='Audio playback device index (-1 for default)'
    )
    
    # Include camera subsystem launch file
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('camera_ros'),
                'launch',
                'camera.launch.py'
            ])
        ]),
        launch_arguments={
            'fps': LaunchConfiguration('camera_fps'),
            'device': LaunchConfiguration('camera_device'),
        }.items()
    )
    
    # Create subsystem nodes using helper functions
    micro_ros_nodes = create_micro_ros_nodes(
        LaunchConfiguration('device0'),
        LaunchConfiguration('device1')
    )
    
    audio_nodes = create_audio_nodes(
        LaunchConfiguration('audio_sample_rate'),
        LaunchConfiguration('audio_capture_device'),
        LaunchConfiguration('audio_playback_device')
    )
    
    # ROSBridge WebSocket Server (for web interface)
    rosbridge_server = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen',
        shell=True
    )
    
    return LaunchDescription([
        # Launch arguments
        device0_arg,
        device1_arg,
        camera_device_arg,
        camera_fps_arg,
        audio_sample_rate_arg,
        audio_capture_device_arg,
        audio_playback_device_arg,
        
        # Startup message
        LogInfo(msg='========================================'),
        LogInfo(msg='🤖 Starting Lucy Robot System...'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Note: Audio underrun warnings are normal when no audio is published'),
        
        # Launch all subsystems
        *micro_ros_nodes,  # Unpack micro-ROS nodes
        rosbridge_server,
        camera_launch,
        *audio_nodes,  # Unpack audio nodes
        
        # Success message
        LogInfo(msg='✅ All ROS nodes launched successfully!'),
        LogInfo(msg='   - Micro-ROS Agents: right & left arm'),
        LogInfo(msg='   - ROSBridge Server: WebSocket ready'),
        LogInfo(msg='   - Camera Publisher: Vision system active'),
        LogInfo(msg='   - Audio System: Capture & playback ready'),
        LogInfo(msg='========================================'),
    ])

