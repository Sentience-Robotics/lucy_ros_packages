# lucy_ros2_control

ROS 2 Control for Lucy: hardware interface, controller config, and launch file.

**Architecture (repo-level):** [`../doc/ROS2_CONTROL.md`](../doc/ROS2_CONTROL.md) — how ros2_control maps to Lucy, `/actuators/*`, launches, and pitfalls.

## Contents

- **Hardware interface** — `LucySystemHardware` (publishes `/actuators/left_arm`, `/actuators/right_arm` for micro-controllers).
- **Config** — `config/lucy_controllers.yaml`.
- **Launch** — `control.launch.py`: robot_state_publisher + ros2_control_node + spawners.

## Building

From your colcon workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lucy_ros2_control
source install/setup.bash
```

## Quick start

```bash
ros2 launch lucy_ros2_control control.launch.py
```

Optional args: `urdf_path:=<path>` `base_path:=<path>` (defaults: **`thais_urdf`** package share → `inmoov/`). Requires **`thais_urdf`** installed in the same workspace / underlay.

## Real + RViz + rosbridge / Gazebo + RViz + rosbridge

Use the **thais_urdf** package:

- `ros2 launch thais_urdf rviz.launch.py` — real robot + RViz + rosbridge (control panel at ws://localhost:9090).
- `ros2 launch thais_urdf gazebo.launch.py` — Gazebo sim + RViz + ros2_control (sim) + rosbridge.

## Dependencies

`controller_manager`, `gz_ros2_control`, `ros_gz_sim`, `rosbridge_server`, `rviz2`.
