# Camera ROS unit tests

Unit tests for the `camera_ros` package.

## Test files

- `test_camera_publisher.py` — `CameraPublisher`
- `test_camera_stream_controller.py` — `CameraStreamController`

## Building

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select camera_ros --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

## Quick start

Run the package test suite:

```bash
colcon test --packages-select camera_ros
colcon test-result --verbose
```

## Running a specific test

**Single file:**

```bash
python3 -m pytest src/lucy_ros_packages/camera_ros/test/test_camera_publisher.py -v
```

**Single test case:**

```bash
python3 -m pytest src/lucy_ros_packages/camera_ros/test/test_camera_publisher.py::TestCameraPublisher::test_camera_detection_webcamproduct -v
```

(Adjust `src/...` paths if your workspace uses a flat `src/camera_ros` layout.)

## Coverage

The tests cover camera detection, fallback device, service-based streaming, client-count callbacks, topic names, and service responses. Hardware is mocked.

**Recommended:** run once via `colcon test` so the ROS 2 environment is set up, then optional HTML/XML from the main repo README *Tests and coverage* section or:

```bash
cd ~/lucy_ws
source install/setup.zsh
colcon test --packages-select camera_ros
colcon test-result --verbose
```

## Notes

- `cv2.VideoCapture` and `subprocess.run` are mocked
- `rclpy` is initialized per test via fixtures
- Direct `pytest` runs need `source install/setup.bash` so generated interfaces are on the path
