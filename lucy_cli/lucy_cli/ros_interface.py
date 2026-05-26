"""
This module provides a high-level, reusable Python interface for interacting
with Lucy's core ROS 2 services and topics.

It is designed to be agnostic of any specific user interface (TUI, GUI, web),
allowing developers to build their own control applications by using this class
as a foundation.

The LucyROSInterface class encapsulates all ROS 2-specific logic, including:
- Node and executor management.
- Service clients for fetching configuration and state.
- Publishers for sending commands.
- Subscribers for receiving state updates.
- Correct QoS (Quality of Service) profile management.

Example Usage:
    import rclpy
    from ros_interface import LucyROSInterface

    def on_client_count_change(count):
        print(f"Client count updated: {count}")

    def on_control_change(controller_id):
        print(f"Control changed to: {controller_id}")

    rclpy.init()
    ros_interface = LucyROSInterface()
    ros_interface.register_on_client_count_change(on_client_count_change)
    ros_interface.register_on_control_change(on_control_change)

    # The interface handles spinning in a background thread.
    # You can now use its methods to interact with the robot.

    try:
        config = ros_interface.get_hardware_config()
        if config:
            # ... parse config and build UI ...
            pass
        # Your application logic here...
    finally:
        ros_interface.shutdown()
        rclpy.shutdown()
"""
import threading
import rclpy
import time
import random
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Int32
from lucy_msgs.srv import GetConfig, GetInt

# --- Unique Client ID ---
# A unique identifier for this specific client instance.
CLIENT_ID = f"cli_{int(time.time())}_{random.randint(1000, 9999)}"

# --- Topic Names ---
CONTROL_MODE_TOPIC = '/control_panel_active_client'
CLIENT_COUNT_TOPIC = '/client_count'

# --- QoS Profiles ---
# For state topics that should be "latched" (i.e., the last published message
# is saved for new subscribers). This is crucial for getting the current
# controller and client count as soon as a client connects.
LATCHING_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
# For command topics where you only care about the latest command and don't
# need persistence. This is the standard for ros2_control joint trajectories.
VOLATILE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE
)


class LucyROSInterface(Node):
    """
    A ROS 2 node providing a high-level API to interact with Lucy's systems.

    This class manages all ROS 2 publishers, subscribers, and service clients,
    exposing simple methods and callbacks for use in an application. It handles
    its own ROS executor in a background thread, so the user does not need to
    manually call `rclpy.spin()`.
    """

    def __init__(self):
        super().__init__('lucy_ros_interface_node')

        # --- Internal State ---
        self._active_controller_id = ""
        self._client_count = 0
        self._has_control = False
        self._control_lock = threading.Lock()
        self._is_shutting_down = False

        # --- Callbacks ---
        # These sets will hold functions to be called when state changes.
        self._on_control_change_callbacks = set()
        self._on_client_count_change_callbacks = set()

        # --- Publishers ---
        self.control_publisher = self.create_publisher(
            String, CONTROL_MODE_TOPIC, qos_profile=LATCHING_QOS)
        # Joint publishers are created dynamically after config is loaded.
        self._joint_publishers = {}

        # --- Subscribers ---
        self.create_subscription(
            String, CONTROL_MODE_TOPIC, self._control_callback, qos_profile=LATCHING_QOS)
        self.create_subscription(
            Int32, CLIENT_COUNT_TOPIC, self._client_count_callback, qos_profile=LATCHING_QOS)

        # --- Service Clients ---
        self._get_config_client = self.create_client(GetConfig, '/config/get')
        self._get_count_client = self.create_client(GetInt, '/get_client_count')

        # --- Executor ---
        # Spin the node in a background thread to handle callbacks automatically.
        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self.get_logger().info(f"LucyROSInterface started with Client ID: {CLIENT_ID}")
        # Take control on startup
        self.take_control()

    # --- Public Methods: Callbacks ---

    def register_on_control_change(self, callback):
        """
        Register a function to be called when the active controller changes.

        The callback will receive one argument:
        - active_client_id (str): The ID of the client now in control, or an
          empty string if control has been released.
        """
        self._on_control_change_callbacks.add(callback)

    def register_on_client_count_change(self, callback):
        """
        Register a function to be called when the client count changes.

        The callback will receive one argument:
        - count (int): The new number of connected clients.
        """
        self._on_client_count_change_callbacks.add(callback)

    # --- Public Methods: State & Commands ---

    @property
    def client_id(self) -> str:
        """Returns the unique ID for this client instance."""
        return CLIENT_ID

    def has_control(self) -> bool:
        """Returns True if this client currently has control, False otherwise."""
        with self._control_lock:
            return self._has_control

    def take_control(self):
        """Publishes this client's ID to request control of the robot."""
        if self._is_shutting_down: return
        self.get_logger().info("Requesting control...")
        msg = String(data=CLIENT_ID)
        try:
            self.control_publisher.publish(msg)
        except Exception:
            pass # Ignore errors if already shutting down

    def release_control(self):
        """Publishes an empty string to release control of the robot."""
        if self._is_shutting_down: return
        self.get_logger().info("Releasing control...")
        msg = String(data="")
        try:
            self.control_publisher.publish(msg)
        except Exception:
            pass # Ignore errors if already shutting down

    def publish_joint_trajectory(self, controller_topic: str, joint_names: list[str], positions_rad: list[float]):
        """
        Publishes a JointTrajectory message to a specified controller.

        Args:
            controller_topic: The full topic name for the joint trajectory
                              controller (e.g., '/left_arm_controller/joint_trajectory').
            joint_names: A list of joint names in the correct order for the controller.
            positions_rad: A list of target joint positions in radians, matching
                           the order of joint_names.
        """
        if self._is_shutting_down: return
        if controller_topic not in self._joint_publishers:
            self.get_logger().warn(f"Creating new publisher for an unknown topic: {controller_topic}")
            self._joint_publishers[controller_topic] = self.create_publisher(
                JointTrajectory, controller_topic, qos_profile=VOLATILE_QOS)

        publisher = self._joint_publishers[controller_topic]
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions_rad
        point.time_from_start = Duration(nanoseconds=200000000).to_msg()  # 0.2s
        msg.points.append(point)
        try:
            publisher.publish(msg)
            self.get_logger().info(f"Published JointTrajectory to {controller_topic}")
        except Exception:
             pass

    # --- Public Methods: Initial State Fetching ---

    def get_hardware_config_yaml(self) -> str | None:
        """
        Synchronously calls the /config/get service to fetch the active config.

        This is a blocking call and should be used during application startup.

        Returns:
            The active hardware configuration as a YAML string, or None if
            the service call fails or times out.
        """
        try:
            if not self._get_config_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("'/config/get' service not available.")
                return None
            req = GetConfig.Request(config_name="")
            future = self._get_config_client.call_async(req)
            
            # Create a temporary node solely to wait for THIS specific future.
            # This avoids interfering with the main executor.
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

    def get_initial_client_count(self) -> int | None:
        """
        Synchronously calls the /get_client_count service.

        This is a blocking call and should be used during application startup.

        Returns:
            The current client count as an integer, or None on failure.
        """
        try:
            if not self._get_count_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn("'/get_client_count' service not available.")
                return None
            req = GetInt.Request()
            future = self._get_count_client.call_async(req)
            
            # Create a temporary node solely to wait for THIS specific future.
            # This avoids interfering with the main executor.
            temp_node = Node('temp_service_caller_2')
            rclpy.spin_until_future_complete(temp_node, future, timeout_sec=3.0)
            temp_node.destroy_node()

            response = future.result()
            if response:
                self.get_logger().info(f"Fetched initial client count: {response.value}")
                # Manually update internal state and call callbacks
                self._client_count = response.value
                for cb in self._on_client_count_change_callbacks:
                    cb(self._client_count)
                return response.value
            return None
        except Exception as e:
            self.get_logger().error(f"Exception in get_initial_client_count: {e}")
            return None

    # --- Internal ROS Callbacks ---

    def _control_callback(self, msg: String):
        """Handles incoming messages on the control topic."""
        with self._control_lock:
            new_controller_id = msg.data
            
            # Only react if the controller has actually changed
            if self._active_controller_id == new_controller_id:
                return

            self._active_controller_id = new_controller_id
            
            # We only lose control if someone else took it.
            # We do NOT lose control if the message is from ourselves,
            # nor do we gain control if the message is empty (released).
            if new_controller_id == CLIENT_ID:
                self._has_control = True
                self.get_logger().info("This client successfully took control.")
            elif new_controller_id == "":
                self._has_control = False
                self.get_logger().info("Control was released by previous client.")
            else:
                self._has_control = False
                self.get_logger().info(f"Control taken by another client: '{new_controller_id}'")
            
            # Notify all registered listeners of the change
            for cb in self._on_control_change_callbacks:
                cb(new_controller_id)

    def _client_count_callback(self, msg: Int32):
        """Handles incoming messages on the client count topic."""
        with self._control_lock:
            new_count = msg.data
            if self._client_count != new_count:
                self._client_count = new_count
                self.get_logger().info(f"Client count updated: {new_count}")
                # Notify all registered listeners
                for cb in self._on_client_count_change_callbacks:
                    cb(new_count)

    # --- Lifecycle ---

    def shutdown(self):
        """Gracefully shuts down the node and its background thread."""
        if self._is_shutting_down: return
        self._is_shutting_down = True
        
        self.get_logger().info("Shutting down LucyROSInterface.")
        self.release_control()
        # Give a moment for the release message to be sent
        time.sleep(0.1)
        self._executor.shutdown()
        # self._spin_thread.join() # This can hang, shutdown is enough
        self.destroy_node()