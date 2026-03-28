#!/bin/sh

colcon build

source /home/dev/lucy_ws/install/setup.zsh

ros2 launch lucy_ros2_control lucy_launch.xml
