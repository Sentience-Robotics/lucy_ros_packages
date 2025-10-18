#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Int32
from camera_ros.srv import GetInt
from cv_bridge import CvBridge
import cv2
import signal
import sys

FPS = 10.0
CAMERA_RESOLUTION = (640, 480)
CAMERA_ENDPOINT = '/dev/video0'

# Trying to get hardware acceleration with GStreamer (NVIDIA Jetson) - not working yet
# GST_PIPELINE = (
#     "v4l2src device=/dev/video0 ! "
#     "image/jpeg,width=1280,height=720,framerate=30/1 ! "
#     "nvjpegdec ! "
#     "nvvidconv ! "
#     "video/x-raw,format=BGRx ! "
#     "videoconvert ! "
#     "video/x-raw,format=BGR ! "
#     "appsink drop=true"
# )

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('sensor_interface')
        # self.pub = self.create_publisher(Image, 'camera/mobius/raw', 10)
        self.jpg_pub = self.create_publisher(CompressedImage, 'camera/mobius/jpg', 10)
        self.bridge = CvBridge()
        self.active = False
        self.init_cap()
        self.timer = self.create_timer(1/FPS, self.publish_frame)
        self.get_logger().info("Camera publisher node started.")
        
        # Pausing camera when no more ros bridge clients
        self.create_subscription(Int32, '/client_count', self.client_count_callback, 10)
        self.client_count = 0
        self.create_service(GetInt, 'get_client_count', self.get_client_count)

    def get_client_count(self, request, response):
        response.value = self.client_count
        return response

    def client_count_callback(self, msg):
        self.client_count = msg.data

        if self.client_count == 0:
            self.active = False
        elif self.client_count > 0:
            self.active = True

        self.get_logger().info(f"Client count: {self.client_count}, camera active: {self.active}")

    def set_active(self, request, response):
        self.active = request.data
        response.success = True
        response.message = f"Camera {'enabled' if self.active else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def init_cap(self):
        # self.cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)

        self.cap = cv2.VideoCapture(CAMERA_ENDPOINT)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_RESOLUTION[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_RESOLUTION[1])
        self.cap.set(cv2.CAP_PROP_FPS, FPS)
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera at {CAMERA_ENDPOINT}")
        else:
            self.get_logger().info(f"Camera opened successfully at {CAMERA_ENDPOINT}")

    def publish_frame(self):
        if not self.active:
            return
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Frame empty, reconnecting camera...")
            self.cap.release()
            self.init_cap()
            return

        # Raw image
        # msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        # self.pub.publish(msg)

        # Encode as JPEG
        ret, jpg = cv2.imencode('.jpg', frame)
        if not ret:
            self.get_logger().warn("Failed to encode frame as JPEG.")
            return

        msg = CompressedImage()
        msg.format = 'jpeg'
        msg.data = jpg.tobytes()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.jpg_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
