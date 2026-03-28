# ros2_control Architecture: Intended vs Actual

This document explains how ros2_control **should** work for the Lucy humanoid robot versus how it **actually** works in the current project. It covers the control panel, micro-ROS nodes, and optional RViz/Gazebo.

---

## 0. Topics listened to by micro-ROS (and why ros2_control ↔ micro-ROS was broken)

### Which topics micro-ROS subscribes to

Each Pico runs a micro-ROS node that **subscribes to exactly one joint-command topic** and **publishes** uptime, log, and trace.

| Build / Pico | Subscribed topic (JointState) | Message type |
|--------------|------------------------------|--------------|
| Left arm (e.g. `USE_LEFT_ARM`)  | `actuators/left_arm`  | `sensor_msgs/msg/JointState` |
| Right arm (e.g. `USE_RIGHT_ARM`)| `actuators/right_arm` | `sensor_msgs/msg/JointState` |

- Topic names come from **`constant.h`**: `JOINTS_TOPIC_NAME` is `"actuators/left_arm"` or `"actuators/right_arm"` depending on compile flag.
- Subscription is created in **`ros.c`**: `rclc_subscription_init_default(..., JOINTS_TOPIC_NAME)` with type `sensor_msgs/msg/JointState`.
- Node name is `"pico_node"`, namespace is empty, so the resolved topic is **`/actuators/left_arm`** or **`/actuators/right_arm`**.

So: **micro-ROS listens only to `/actuators/left_arm` (left Pico) and `/actuators/right_arm` (right Pico)** for joint commands. No other topics are used for that.

### Why communication between ros2_control and micro-ROS was broken

ros2_control is supposed to **publish** `sensor_msgs/msg/JointState` on those same topics from **LucySystemHardware** in `write()`. Communication failed for one or more of these reasons:

1. **ros2_control not running**  
   Default bringup (`lucy.launch.py`) does **not** start ros2_control. So nothing was publishing on `/actuators/left_arm` or `/actuators/right_arm`. Micro-ROS was listening, but there was no publisher.

2. **LucySystemHardware not loaded**  
   In Gazebo (`use_gazebo_sim:=true`) the URDF uses `GazeboSimSystem`, not `LucySystemHardware`. So in sim there is no publisher for `/actuators/*`; only `/joint_states` from the joint_state_broadcaster exists. Micro-ROS on real hardware would see no commands.

3. **Bugs in the hardware interface (now fixed)**  
   - Both left and right hardware used the same node name (`"lucy_hardware_interface"`), and the `node_name` param from the xacro was ignored. That could cause duplicate-node issues and only one effective publisher.
   - There was no check for the `publisher_topic` param; if it was missing, init could fail or publish on a wrong/empty topic.
   - The published `JointState` did not set `msg.name` (only `msg.position`), so even if messages were sent, subscribers might misbehave or ignore them.

4. **Controllers not active**  
   `write()` is only called when the position controllers (e.g. `left_arm_controller`, `right_arm_controller`) are **loaded and active**. If you only spawned `joint_state_broadcaster` and not the arm controllers, the hardware interface would never get `write()` calls, so nothing would be published to `/actuators/left_arm` or `/actuators/right_arm`.

**Summary:** Micro-ROS listens to **`/actuators/left_arm`** and **`/actuators/right_arm`** for **`sensor_msgs/msg/JointState`**. Communication was broken because either ros2_control (and thus the publisher) was not running, or the hardware interface was not loaded/active, or it had bugs that prevented correct or any publication. Fixing the hardware interface (node name, param check, and filling `msg.name` + header) and running ros2_control with both arm controllers active restores the link.

---

## 1. How ros2_control *should* work for a humanoid

ros2_control is the **middleware**: the single place that owns the robot model, controllers, and hardware abstraction. All motion commands and joint state flow through it.

### Intended data flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  HIGH-LEVEL CLIENTS (optional)                                              │
│  • Web control panel  • MoveIt  • Scripts  • Teleop                          │
└───────────────────────────────┬─────────────────────────────────────────────┘
                                 │ JointTrajectory (or other command interfaces)
                                 ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  ROS2_CONTROL (middleware – always in the loop for motion)                   │
│  • Controller manager                                                        │
│  • JointTrajectoryController (left/right)                                   │
│  • joint_state_broadcaster                                                   │
│  • Hardware interfaces (LucySystemHardware left + right)                      │
└───────┬─────────────────────────────────────────────┬─────────────────────────┘
        │                                             │
        │ JointState (commands to actuators)          │ /joint_states (for TF + viz)
        │ /actuators/left_arm, /actuators/right_arm         │
        ▼                                             ▼
┌───────────────────────┐                   ┌─────────────────────────────────┐
│  MICRO-ROS NODES      │                   │  OPTIONAL VISUALIZATION          │
│  (Pico left/right)    │                   │  • RViz (subscribe /joint_states  │
│  Subscribe to         │                   │    + robot_description + TF)     │
│  /actuators/*, drive    │                   │  • Gazebo (sim only; uses        │
│  servos               │                   │    gz_ros2_control, no Picos)   │
└───────────────────────┘                   └─────────────────────────────────┘
```

### Intended behavior

1. **Single authority**  
   All actuator commands come from ros2_control. High-level clients (e.g. control panel) send **commands to ros2_control** (e.g. `JointTrajectory` to `JointTrajectoryController`), not directly to the micro-controllers.

2. **Hardware interface → micro-ROS**  
   The hardware plugin turns controller outputs into the protocol the real hardware expects. For Lucy: publish `sensor_msgs/JointState` on `/actuators/left_arm` and `/actuators/right_arm`. Micro-ROS Picos **subscribe** to these topics and drive servos.

3. **Joint state for TF and visualization**  
   Joint state is provided by ros2_control (e.g. from hardware `read()` or from `joint_state_broadcaster`). It is published on `/joint_states`. `robot_state_publisher` uses it for TF. **RViz and Gazebo are optional consumers** of that data (and of the URDF); they are not required for basic motion.

4. **Default run = no visualization**  
   The default bringup should be: robot_state_publisher + ros2_control + spawners + (optionally) micro-ROS agents and other sensors. RViz and Gazebo are **add-ons**, not part of the minimal “robot + control” stack.

---

## 2. How it *actually* works in the project

### Control panel

- The panel publishes **JointTrajectory** to:
  - `/left_arm_controller/joint_trajectory`
  - `/right_arm_controller/joint_trajectory`
- So the panel is correctly talking **to** ros2_control (the controllers), not directly to the Picos.
- If ros2_control is **not** running, these topics have no subscriber and the commands are effectively dropped.

### ros2_control and hardware interface

- When ros2_control **is** running (e.g. via `control.launch.py` or `rviz.launch.py`):
  - **JointTrajectoryController** receives the panel’s (or other) trajectory messages and updates its command interfaces.
  - **LucySystemHardware** `write()` is called each cycle and publishes **JointState** on `/actuators/left_arm` and `/actuators/right_arm` with the current commands.
  - Micro-ROS Picos subscribe to those topics and drive the servos.
- **joint_state_broadcaster** publishes all state interfaces to **one** topic: `/joint_states`. So visualization (e.g. RViz) and `robot_state_publisher` use `/joint_states`, not `/actuators/left_arm` or `/actuators/right_arm`.
- **State today**: There are no real encoders. In `read()`, `hw_positions_` is set from `hw_commands_`. So “joint state” is last commanded position, not measured. This is acceptable for visualization that follows the command.

### Launch files and default bringup

| Launch file            | ros2_control | RViz | Gazebo | rosbridge | micro-ROS agents |
|------------------------|--------------|------|--------|-----------|------------------|
| **lucy.launch.py**     | ❌ No        | ❌   | ❌     | ✅        | ✅               |
| **control.launch.py**  | ✅ Yes       | ❌   | ❌     | ❌        | ❌               |
| **thais_urdf rviz.launch.py** | ✅ Yes | ✅ Yes | ❌     | ✅        | ❌               |
| **thais_urdf gazebo.launch.py** | ✅ (sim) | ✅ | ✅ | ✅        | N/A (sim)        |

- **lucy.launch.py** is the main bringup (micro-ROS, rosbridge, cameras) but does **not** start ros2_control. So with only `lucy.launch.py`, the panel’s JointTrajectory is not consumed and the Picos do not get commands from ros2_control.
- **control.launch.py** is the minimal “real robot + ros2_control” stack (no RViz, no rosbridge). It does not start micro-ROS agents or rosbridge; those are expected to be running separately if you want the panel and real hardware.
- **rviz.launch.py** starts ros2_control + RViz + rosbridge (real robot + optional visualization). RViz is currently **always** included, not optional.
- **gazebo.launch.py** is for simulation (Gazebo + gz_ros2_control); no real Picos.

### Summary of gaps

1. **ros2_control not in default bringup**  
   Default “Lucy” stack is `lucy.launch.py`, which has no ros2_control. So “actuator positions published by the control panel” only reach the Picos when ros2_control is also running (e.g. by also launching `control.launch.py` or `rviz.launch.py`).

2. **Two separate stacks**  
   To get panel + real hardware you typically need both:
   - `lucy.launch.py` (rosbridge + micro-ROS + cameras), and  
   - `control.launch.py` (or rviz.launch.py) for ros2_control.  
   So ros2_control is not the default middleware in the main bringup.

3. **RViz/Gazebo as options**  
   - “RViz/Gazebo should be options, not defaults” is satisfied for the **minimal** stack: `control.launch.py` has no RViz.  
   - But the “real robot + viz” path is `rviz.launch.py`, which always starts RViz (no launch arg to disable it). So RViz is optional only in the sense that you can choose a different launch file.

4. **Single run with panel + Picos + optional viz**  
   Ideally, one bringup would start: ros2_control + robot_state_publisher + (optional) RViz + (optional) rosbridge + micro-ROS agents + cameras. Today that requires either composing two launch files or a single “full” launch that includes all of these and makes RViz (and optionally Gazebo) configurable.

---

## 3. Target architecture (short)

- **ros2_control as middleware**: All motion goes through it. Panel (and other clients) send commands to ros2_control; ros2_control publishes to `/actuators/left_arm` and `/actuators/right_arm` for micro-ROS; joint state goes to `/joint_states` for TF and optional RViz/Gazebo.
- **Default run**: Real robot + ros2_control + micro-ROS + rosbridge (for panel), **without** RViz/Gazebo by default.
- **Optional**: Add RViz or Gazebo via launch args or separate launch files so they remain optional, not defaults.
