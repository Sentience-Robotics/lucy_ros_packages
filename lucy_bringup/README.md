# Lucy Bringup Package

System launch files and scripts for the Lucy robot on NVIDIA Jetson AGX Orin.

## Overview

This package provides a unified way to launch all components of the Lucy robot system:
- Two micro-ROS agents (for RP2040 controllers on left and right arms)
- ROSBridge WebSocket server (for web interface communication)
- Camera publisher node (for vision system)
- Web control panel interface

## Quick Start

### Using the tmux Launcher (Recommended for Development)

```bash
# Launch everything
~/launch_lucy.sh

# Check system status
~/check_lucy.sh

# Stop everything
~/stop_lucy.sh
```

### Using ROS2 Launch Directly

```bash
# Source workspace
source ~/lucy_ws/install/setup.zsh

# Launch only ROS nodes (no web interface)
ros2 launch lucy_bringup lucy.launch.py

# Launch with custom devices
ros2 launch lucy_bringup lucy.launch.py device0:=/dev/ttyACM2 device1:=/dev/ttyACM3
```

## Architecture

### tmux Layout

```
┌─────────────────────────────────────────────────────────────┐
│ Pane 0: ROS2 Nodes (Top 60%)                                │
│  ├─ micro_ros_agent_right                                   │
│  ├─ micro_ros_agent_left                                    │
│  ├─ rosbridge_server                                        │
│  └─ camera_publisher                                        │
├──────┬──────────────────────────────────────────────────────┤
│ Web  │ Pane 2: Debug Terminal                               │
│ ~12% │ ~88%                                                 │
└──────┴──────────────────────────────────────────────────────┘
```

### tmux Controls

**Navigation:**
- **Detach (keep running)**: `Ctrl+B` then `D`
- **Switch panes**: `Ctrl+B` then arrow keys
- **Cycle panes**: `Ctrl+B` then `O`
- **Zoom pane**: `Ctrl+B` then `Z` (toggle fullscreen)
- **Reattach**: `tmux attach -t lucy`

**Scrolling (Copy Mode):**
- **Enter scroll mode**: `Ctrl+B` then `[`
- **Navigate**: Arrow keys, `PgUp`/`PgDn`, mouse wheel (if enabled)
- **Search forward**: `/` then type search term
- **Search backward**: `?` then type search term
- **Exit scroll mode**: `q` or `Esc`

## Launch Arguments

The `lucy.launch.py` file accepts the following arguments:

- `device0` - Serial device for right arm (default: `/dev/ttyACM0`)
- `device1` - Serial device for left arm (default: `/dev/ttyACM1`)
- `camera_device` - Camera device path (default: `/dev/video0`)
- `camera_fps` - Camera frame rate (default: `15.0`)

## System Requirements

- Ubuntu 22.04 on NVIDIA Jetson AGX Orin
- ROS2 Humble
- tmux
- Two RP2040 controllers connected via USB
- Camera device

## Troubleshooting

### Serial Devices Not Found

```bash
# List USB devices
ls -l /dev/ttyACM*

# Check which device is which
ros2 topic echo /trace_publisher
```

### Web Interface Not Starting

Check if the web directory exists:
```bash
ls ~/web_control_panel
```

### ROS Nodes Not Communicating

```bash
# Check all nodes
ros2 node list

# Check topics
ros2 topic list

# Echo joint commands
ros2 topic echo /joints/right_arm
ros2 topic echo /joints/left_arm
```

## Files

```
lucy_bringup/
├── launch/
│   └── lucy.launch.py          # Main ROS2 launch file
├── system_scripts/
│   ├── launch_lucy.sh          # tmux launcher
│   ├── stop_lucy.sh            # Graceful shutdown
│   └── check_lucy.sh           # Health check
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Symlinks

For convenience, symlinks are created in `/home/dev/`:
- `~/launch_lucy.sh` → `lucy_ws/src/lucy_ros_packages/lucy_bringup/system_scripts/launch_lucy.sh`
- `~/stop_lucy.sh` → `lucy_ws/src/lucy_ros_packages/lucy_bringup/system_scripts/stop_lucy.sh`
- `~/check_lucy.sh` → `lucy_ws/src/lucy_ros_packages/lucy_bringup/system_scripts/check_lucy.sh`

## License

GPL-3.0 - See LICENSE file for details.

Copyright 2024 Sentience Robotics Team

