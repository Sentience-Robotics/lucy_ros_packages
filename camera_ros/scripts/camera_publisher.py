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
        
        self.init_cap()

        self.last_publish_time = 0.0
        self.frame_skip_counter = 0

        self.timer = self.create_timer(0.01, self.publish_frame)  # 100Hz timer
        self.get_logger().info(f"Camera publisher node started using device {self.camera_device} at {self.target_fps} FPS")
    

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

    def init_cap(self):
        # Create GStreamer pipeline with parameterized device
        gst_pipeline = (
            f"v4l2src device={self.camera_device} ! "
            "image/jpeg,width=1280,height=720,framerate=30/1 ! "
            "jpegparse ! "
            "appsink drop=true emit-signals=true sync=false"
        )
        
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera on {self.camera_device} with GStreamer pipeline")
            return
        else:
            self.get_logger().info(f"Camera on {self.camera_device} opened successfully")

        # Optimize OpenCV settings for Jetson
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        except:
            pass

    def publish_frame(self):
        if not self.active:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_publish_time < (1.0 / self.target_fps):
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().error(f"Frame read failed on {self.camera_device}, reconnecting camera...")
            self.cap.release()
            self.init_cap()
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
