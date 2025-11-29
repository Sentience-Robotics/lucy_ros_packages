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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from camera_ros.srv import GetInt
import cv2
import os
import stat

FPS = 15.0
CAMERA_DEVICE = "/dev/video0"

class CameraPublisher(Node):
    """
    Zero-copy MJPEG camera publisher using GStreamer pipeline.

    Publishes compressed JPEG images from camera with zero CPU transcoding overhead.
    Automatically activates when clients are detected and pauses when no clients are present.
    Listens to /client_count topic for automatic activation/deactivation."""

    def __init__(self):
        super().__init__('sensor_interface')
        
        self.declare_parameter('fps', FPS)
        self.declare_parameter('device', CAMERA_DEVICE)
        
        # Get parameter values
        self.target_fps = self.get_parameter('fps').get_parameter_value().double_value
        self.camera_device = self.get_parameter('device').get_parameter_value().string_value
        
        self.jpg_pub = self.create_publisher(CompressedImage, 'camera/mobius/jpg', 10)
        self.active = False
        self.client_count = 0
        self.cap = None
        self.last_reconnect_attempt = 0.0
        self.reconnect_interval = 2.0
        self.reconnect_log_interval = 1.0
        self.last_device_check = 0.0
        self.device_check_interval = 1.0
        self.was_connected = False
        
        self.init_cap()

        self.last_publish_time = 0.0
        self.frame_skip_counter = 0

        self.timer = self.create_timer(0.01, self.publish_frame)
        self.reconnect_timer = self.create_timer(1.0, self.attempt_reconnect)
        
        if self.cap is not None:
            self.get_logger().info(f"Camera publisher node started using device {self.camera_device} at {self.target_fps} FPS")
            self.was_connected = True
        else:
            self.get_logger().warn(f"Camera publisher node started but camera on {self.camera_device} is not available")
            self.get_logger().warn("Node will continue running and will automatically reconnect when camera becomes available")
            self.was_connected = False
    

        self.create_subscription(Int32, '/client_count', self.client_count_callback, 10)
        
        self.create_service(GetInt, 'get_client_count', self.get_client_count)

    def get_client_count(self, request, response):
        response.value = self.client_count
        return response

    def client_count_callback(self, msg):
        """Automatically activate/deactivate camera based on client count"""
        self.client_count = msg.data
        
        if self.client_count == 0 and self.active:
            self.active = False
            self.get_logger().info(f"No clients detected - pausing camera on {self.camera_device}")
        elif self.client_count > 0 and not self.active:
            self.active = True
            self.get_logger().info(f"{self.client_count} client(s) detected - activating camera on {self.camera_device}")
        elif self.client_count > 0 and self.active:
            self.get_logger().debug(f"Camera on {self.camera_device} active with {self.client_count} client(s)")

    def set_fps(self, fps):
        self.target_fps = max(1.0, min(30.0, fps))
        self.get_logger().info(f"Frame rate set to {self.target_fps} FPS")

    def attempt_reconnect(self):
        """Periodically check camera status and attempt to reconnect if needed"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Check device availability periodically
        if current_time - self.last_device_check >= self.device_check_interval:
            self.last_device_check = current_time
            device_available = self.check_device(verbose=False)
            
            # Check if camera is connected
            camera_connected = False
            if self.cap is not None:
                try:
                    camera_connected = self.cap.isOpened()
                except:
                    camera_connected = False
            
            # Detect disconnection
            if self.was_connected and (not device_available or not camera_connected):
                if not device_available:
                    self.get_logger().warn(f"Camera device {self.camera_device} disconnected (device no longer available)")
                else:
                    self.get_logger().warn(f"Camera on {self.camera_device} disconnected (capture no longer opened)")
                # Clean up
                try:
                    if self.cap is not None:
                        self.cap.release()
                except:
                    pass
                self.cap = None
                self.was_connected = False
            
            # Attempt reconnection if camera is not available
            if self.cap is None:
                # Throttle reconnection attempts
                if current_time - self.last_reconnect_attempt >= self.reconnect_interval:
                    self.last_reconnect_attempt = current_time
                    
                    # Check if device is now available
                    if device_available:
                        self.get_logger().info(f"Camera device {self.camera_device} detected, attempting to reconnect...")
                        self.init_cap()
                        
                        if self.cap is not None:
                            self.get_logger().info(f"Successfully reconnected to camera on {self.camera_device}")
                            self.was_connected = True
                        else:
                            # Log failed reconnection attempt
                            self.get_logger().warn(f"Reconnection attempt failed, will retry in {self.reconnect_interval}s")
            else:
                # Camera is connected, update state
                if camera_connected:
                    self.was_connected = True

    def check_device(self, verbose=True):
        """Check if camera device exists and is accessible
        
        Args:
            verbose: If True, log errors and info. If False, only return status.
        """
        if not os.path.exists(self.camera_device):
            if verbose:
                self.get_logger().error(f"Camera device {self.camera_device} does not exist")
                self.get_logger().info("Available video devices:")
                # List available video devices
                for i in range(10):
                    dev_path = f"/dev/video{i}"
                    if os.path.exists(dev_path):
                        self.get_logger().info(f"  - {dev_path}")
            return False
        
        # Check if device is a character device
        try:
            mode = os.stat(self.camera_device).st_mode
            if not stat.S_ISCHR(mode):
                if verbose:
                    self.get_logger().error(f"{self.camera_device} is not a character device")
                return False
        except OSError as e:
            if verbose:
                self.get_logger().error(f"Cannot access {self.camera_device}: {e}")
            return False
        
        # Check read permissions
        if not os.access(self.camera_device, os.R_OK):
            if verbose:
                self.get_logger().error(f"No read permission for {self.camera_device}")
                self.get_logger().info("Try running with sudo or add user to video group: sudo usermod -a -G video $USER")
            return False
        
        return True

    def init_cap(self):
        """Initialize camera capture with GStreamer, fallback to direct OpenCV if needed"""
        # First check if device exists and is accessible
        if not self.check_device():
            self.cap = None
            return
        
        # Try GStreamer pipeline first (preferred for Jetson)
        gst_pipeline = (
            f"v4l2src device={self.camera_device} ! "
            "image/jpeg,width=1280,height=720,framerate=30/1 ! "
            "jpegparse ! "
            "appsink drop=true emit-signals=true sync=false"
        )
        
        self.get_logger().info(f"Attempting to open camera with GStreamer pipeline...")
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().warn(f"GStreamer pipeline failed, trying direct OpenCV VideoCapture...")
            self.cap.release()
            
            # Fallback to direct OpenCV VideoCapture
            # Try device index first (if /dev/video0, try index 0)
            try:
                device_index = int(self.camera_device.replace('/dev/video', ''))
                self.cap = cv2.VideoCapture(device_index)
            except ValueError:
                # If not a standard /dev/videoN format, try as string
                self.cap = cv2.VideoCapture(self.camera_device)
            
            if not self.cap.isOpened():
                self.get_logger().error(f"Failed to open camera on {self.camera_device} with both GStreamer and direct OpenCV")
                self.get_logger().error("Please verify:")
                self.get_logger().error(f"  1. Device {self.camera_device} exists and is accessible")
                self.get_logger().error("  2. Camera is properly connected")
                self.get_logger().error("  3. User has permissions (check with: ls -l /dev/video*)")
                self.get_logger().error("  4. No other process is using the camera")
                self.cap = None
                return
            else:
                self.get_logger().info(f"Camera on {self.camera_device} opened successfully using direct OpenCV")
                self.was_connected = True
        else:
            self.get_logger().info(f"Camera on {self.camera_device} opened successfully using GStreamer")
            self.was_connected = True

        # Optimize OpenCV settings for Jetson
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        except:
            pass

    def publish_frame(self):
        if not self.active:
            return
        
        # Check if camera is available
        if self.cap is None:
            return

        # Verify capture is still opened
        try:
            if not self.cap.isOpened():
                if self.was_connected:
                    self.get_logger().warn(f"Camera capture is no longer opened on {self.camera_device} - camera disconnected")
                try:
                    self.cap.release()
                except:
                    pass
                self.cap = None
                self.was_connected = False
                return
        except Exception as e:
            if self.was_connected:
                self.get_logger().warn(f"Error checking camera status: {e}, camera may have disconnected")
            try:
                if self.cap is not None:
                    self.cap.release()
            except:
                pass
            self.cap = None
            self.was_connected = False
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_publish_time < (1.0 / self.target_fps):
            return

        # Attempt to read frame with exception handling
        try:
            ret, frame = self.cap.read()
        except Exception as e:
            if self.was_connected:
                self.get_logger().warn(f"Exception while reading frame: {e}, camera may have disconnected")
            ret = False
            frame = None
        
        if not ret or frame is None:
            if self.was_connected:
                self.get_logger().warn(f"Frame read failed on {self.camera_device}, camera may have disconnected")
            # Release the capture object
            try:
                if self.cap is not None:
                    self.cap.release()
            except:
                pass
            self.cap = None
            self.was_connected = False
            # Reconnection will be handled by attempt_reconnect timer
            return

        if frame.shape[0] == 1:  # Raw JPEG data
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            msg.data = frame.flatten().tobytes()
            self.jpg_pub.publish(msg)

            self.last_publish_time = current_time
        else:
            self.get_logger().error(f"Unexpected frame shape: {frame.shape}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
