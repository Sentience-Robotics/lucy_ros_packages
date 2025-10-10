#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_srvs.srv import SetBool
from std_msgs.msg import Int32
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
        self.enable_srv = self.create_service(SetBool, 'camera/mobius/enable', self.set_active)
        self.get_logger().info("Camera publisher node started.")
        
        # Pausing camera when no more ros bridge clients
        self.cli = self.create_client(SetBool, 'camera/mobius/enable')
        while not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('Waiting for camera/mobius/enable service...')
        self.create_subscription(Int32, '/client_count', self.client_count_callback, 10)

    def client_count_callback(self, msg):
        count = msg.data

        if count == 0:
            self.get_logger().info('No clients detected — pausing camera.')
            self.pause_camera()

    def pause_camera(self):
        req = SetBool.Request()
        req.data = False
        future = self.cli.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Camera paused successfully: {response.message}")
            else:
                self.get_logger().warn(f"Camera pause failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

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
        self.jpg_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()

    rclpy.spin(node)

if __name__ == '__main__':
    main()

