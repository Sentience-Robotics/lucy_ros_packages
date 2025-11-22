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

Optimized for NVIDIA Jetson AGX Orin.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration


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
    
    # Node 1: First Micro-ROS Agent (Right Arm - ACM0)
    micro_ros_agent_right = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_right',
        arguments=['serial', '--dev', LaunchConfiguration('device0')],
        output='screen',
        respawn=True,  # Auto-restart if crashes
        respawn_delay=2.0,
        emulate_tty=True
    )
    
    # Node 2: Second Micro-ROS Agent (Left Arm - ACM1)
    micro_ros_agent_left = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_left',
        arguments=['serial', '--dev', LaunchConfiguration('device1')],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        emulate_tty=True
    )
    
    # Node 3: ROSBridge WebSocket Server (for web interface)
    rosbridge_server = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen',
        shell=True
    )
    
    # Node 4: Camera Publisher
    camera_node = Node(
        package='camera_ros',
        executable='camera_publisher.py',
        name='camera_publisher',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'fps': LaunchConfiguration('camera_fps'),
            'device': LaunchConfiguration('camera_device'),
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        device0_arg,
        device1_arg,
        camera_device_arg,
        camera_fps_arg,
        
        # Startup message
        LogInfo(msg='========================================'),
        LogInfo(msg='🤖 Starting Lucy Robot System...'),
        LogInfo(msg='========================================'),
        
        # Launch all nodes
        micro_ros_agent_right,
        micro_ros_agent_left,
        rosbridge_server,
        camera_node,
        
        # Success message
        LogInfo(msg='✅ All ROS nodes launched successfully!'),
        LogInfo(msg='   - Micro-ROS Agents: right & left arm'),
        LogInfo(msg='   - ROSBridge Server: WebSocket ready'),
        LogInfo(msg='   - Camera Publisher: Vision system active'),
        LogInfo(msg='========================================'),
    ])

