#!/bin/bash

# Clean previous builds to ensure the latest code is used
rm -rf install/lucy_cli

# Build the package
colcon build --packages-select lucy_cli

# Source the new installation
source install/setup.bash

# Run the node
ros2 run lucy_cli tui