# Developer Guide: Creating a New Lucy Interface

This guide explains how to build a new client application (e.g., a web UI, a different command-line tool, a Python script for automation) that can interact with and control a Lucy-enabled robot.

The core of the Lucy control system is a set of ROS 2 topics and services. By interacting with these, any application can view robot state, get its hardware configuration, and send commands.

The recommended way to interface with Lucy is to use the `lucy_cli.ros_interface.LucyROSInterface` class as a reference. It provides a high-level, well-documented Python API that handles all the underlying ROS 2 complexity. This guide will walk through the key concepts and provide code snippets based on that reference implementation.

> Please note that the LucyROSInterface class is a simple example. You will find that more topics & services are available if needed, with for example a complete configuration workflow. If you need more feature or want to write your interface in another language, please check the differents packages available in this repository. The [Lucy Control Panel](https://github.com/Sentience-Robotics/lucy_control_panel/tree/master/) is a good example of a more in-depth implementation.

## 1. Core Concepts

### Control Arbitration (Taking and Releasing Control)

To prevent multiple users from sending conflicting commands simultaneously, Lucy uses a simple control arbitration system.

- **Topic:** `/control_panel_active_client`
- **Message Type:** `std_msgs/msg/String`
- **QoS:** Latching (`TRANSIENT_LOCAL` durability)

**Protocol:**
1.  **To Take Control:** Publish your unique client ID to this topic.
2.  **To Release Control:** Publish an empty string (`""`) to this topic.
3.  **To Monitor Control:** Subscribe to this topic. If you receive a message containing a client ID that is *not* your own, your application should enter a "read-only" mode and disable its own command publishing features.

A good client ID is one that is both unique and descriptive, for example: `my_app_1678886400_a3f7`.

```python
# From lucy_cli/ros_interface.py

import time
import random

# A unique identifier for this specific client instance.
CLIENT_ID = f"my_app_{int(time.time())}_{random.randint(1000, 9999)}"

# ... inside your ROS Node class ...
# self.control_publisher = self.create_publisher(String, '/control_panel_active_client', qos_profile=LATCHING_QOS)

def take_control(self):
    """Publishes this client's ID to request control of the robot."""
    self.get_logger().info("Requesting control...")
    msg = String(data=CLIENT_ID)
    self.control_publisher.publish(msg)

def release_control(self):
    """Publishes an empty string to release control of the robot."""
    self.get_logger().info("Releasing control...")
    msg = String(data="")
    self.control_publisher.publish(msg)
```

### Dynamic Hardware Configuration

You should **never** hardcode joint names, limits, or controller topics in your application. Lucy provides a service to fetch the currently active hardware configuration from the robot.

- **Service:** `/config/get`
- **Service Type:** `lucy_msgs/srv/GetConfig`

To get the currently active configuration, call this service with an empty `config_name`. The service will return a YAML string containing all the necessary information.

```python
# From lucy_cli/ros_interface.py

def get_hardware_config_yaml(self) -> str | None:
    """
    Synchronously calls the /config/get service to fetch the active config.
    """
    try:
        # Wait for the service to be available
        if not self._get_config_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("'/config/get' service not available.")
            return None
            
        # Call the service with an empty name to get the active config
        req = GetConfig.Request(config_name="")
        future = self._get_config_client.call_async(req)
        
        # This requires spinning a temporary node to wait for the result
        temp_node = Node('temp_service_caller')
        rclpy.spin_until_future_complete(temp_node, future, timeout_sec=5.0)
        temp_node.destroy_node()

        response = future.result()
        if not response or not response.success:
            self.get_logger().error(f"Failed to get active config: {response.message if response else 'timeout'}")
            return None
            
        self.get_logger().info("Successfully fetched active hardware configuration.")
        return response.config_yaml
    except Exception as e:
        self.get_logger().error(f"Exception in get_hardware_config_yaml: {e}")
        return None
```

The returned YAML needs to be parsed to build your UI and command structures. See `tui_node.py`'s `parse_config_yaml` function for a reference implementation.

## 2. Sending Commands

### Commanding Joint Positions

To move the robot's joints, you must publish to the appropriate [`ros2_control`](https://control.ros.org/rolling/index.html) controller topic. These topics are discovered from the hardware configuration YAML.

- **Topic:** Dynamically determined (e.g., `/left_arm_controller/joint_trajectory`)
- **Message Type:** `trajectory_msgs/msg/JointTrajectory`
- **QoS:** Volatile (`VOLATILE` durability)

A `JointTrajectory` message contains a list of joint names and one or more `JointTrajectoryPoint`s. For direct position control, you only need to send one point.

**Important:** The `joint_names` list **must** be in the exact order expected by the controller, which is the order you get from parsing the hardware config YAML.

```python
# From lucy_cli/ros_interface.py

def publish_joint_trajectory(self, controller_topic: str, joint_names: list[str], positions_rad: list[float]):
    """
    Publishes a JointTrajectory message to a specified controller.
    """
    # Create a publisher for this topic if it doesn't exist yet
    if controller_topic not in self._joint_publishers:
        self.get_logger().warn(f"Creating new publisher for an unknown topic: {controller_topic}")
        self._joint_publishers[controller_topic] = self.create_publisher(
            JointTrajectory, controller_topic, qos_profile=VOLATILE_QOS)

    publisher = self._joint_publishers[controller_topic]
    
    msg = JointTrajectory()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.joint_names = joint_names # Must be in the correct order
    
    point = JointTrajectoryPoint()
    point.positions = positions_rad # Must be in radians
    # A small non-zero time is required for the controller to accept the goal
    point.time_from_start = Duration(nanoseconds=200000000).to_msg()  # 0.2s
    
    msg.points.append(point)
    publisher.publish(msg)
```

## 3. Subscribing to State

### Monitoring Connected Clients

You can monitor how many clients are connected to the control system. This is useful for diagnostics and understanding system load.

- **Topic:** `/client_count`
- **Message Type:** `std_msgs/msg/Int32`
- **QoS:** Latching (`TRANSIENT_LOCAL` durability)

- **Service:** `/get_client_count`
- **Service Type:** `lucy_msgs/srv/GetInt`

**Protocol:**
1.  **On Startup:** Call the `/get_client_count` service to get the number of clients already connected.
2.  **Continuously:** Subscribe to the `/client_count` topic to receive live updates whenever a client connects or disconnects.

```python
# From lucy_cli/ros_interface.py

# In __init__:
# self.create_subscription(Int32, '/client_count', self._client_count_callback, qos_profile=LATCHING_QOS)

def _client_count_callback(self, msg: Int32):
    """Handles incoming messages on the client count topic."""
    new_count = msg.data
    # ... (update internal state and notify application) ...

# --- In your startup logic ---
def get_initial_client_count(self) -> int | None:
    """
    Synchronously calls the /get_client_count service.
    """
    # ... (implementation similar to get_hardware_config_yaml) ...
```

### Other Important Topics

- **`/joint_states`** (`sensor_msgs/msg/JointState`): This topic is published by `ros2_control` and contains the *actual*, measured position of every joint on the robot. You should subscribe to this to display the robot's real-time state, which may differ from your last commanded position.

## Summary of Endpoints

| Endpoint | Type | Message/Service Type | QoS Durability | Purpose |
|---|---|---|---|---|
| `/control_panel_active_client` | Topic | `std_msgs/msg/String` | `TRANSIENT_LOCAL` | Take, release, and monitor control. |
| `/client_count` | Topic | `std_msgs/msg/Int32` | `TRANSIENT_LOCAL` | Get live updates on the number of connected clients. |
| `/config/get` | Service | `lucy_msgs/srv/GetConfig` | N/A | Fetch the active robot hardware configuration (YAML). |
| `/get_client_count` | Service | `lucy_msgs/srv/GetInt` | N/A | Get the initial number of connected clients on startup. |
| `/{controller_name}/joint_trajectory` | Topic | `trajectory_msgs/msg/JointTrajectory` | `VOLATILE` | Send joint position commands to a specific controller. |
| `/joint_states` | Topic | `sensor_msgs/msg/JointState` | `VOLATILE` | Read the actual, measured state of all robot joints. |

By using these endpoints, you can build any application to reliably and safely interact with the Lucy robot ecosystem.