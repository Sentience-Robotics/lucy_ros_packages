# Camera ROS Unit Tests

This directory contains unit tests for the `camera_ros` package.

## Test Files

- `test_camera_publisher.py` - Tests for CameraPublisher node
- `test_camera_stream_controller.py` - Tests for CameraStreamController node

## Running Tests

### Run all tests
```bash
cd ~/lucy_ws
colcon build
source install/setup.zsh
colcon test --packages-select camera_ros
colcon test-result --verbose
```

### Run specific test file
```bash
python3 -m pytest src/lucy_ros_packages/camera_ros/test/test_camera_publisher.py -v
```

### Run specific test
```bash
python3 -m pytest src/lucy_ros_packages/camera_ros/test/test_camera_publisher.py::TestCameraPublisher::test_camera_detection_webcamproduct -v
```

## Test Coverage

The tests cover:
- Camera detection by webcamproduct name
- Fallback to default device
- Service-based streaming control (start/stop)
- Client count callback functionality
- Topic naming (ext_camera/jpg)
- Service responses

### Generate Coverage Report

**Note:** For best results, run tests through `colcon test` first, which properly sets up the ROS2 environment.

**Option 1: Using colcon test (recommended)**
```bash
cd ~/lucy_ws

colcon build
source install/setup.zsh

colcon test --packages-select camera_ros
colcon test-result --verbose
```

## Notes

- Tests use mocking to avoid requiring actual camera hardware
- cv2.VideoCapture and subprocess.run are mocked
- rclpy is initialized/shutdown for each test via fixture
- Coverage reports are generated automatically when using pytest directly

