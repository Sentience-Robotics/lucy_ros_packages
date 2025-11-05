# Zero-Copy MJPEG Camera Publisher for ROS2

A high-performance camera publisher optimized for NVIDIA Jetson AGX Orin that provides zero-copy MJPEG streaming with minimal CPU overhead.

## Features

- 🚀 **Zero-copy MJPEG streaming** - No CPU transcoding overhead
- ⚡ **High performance** - Optimized for NVIDIA Jetson AGX Orin
- 🎯 **Smart frame rate control** - Dynamic FPS adjustment
- 🔄 **Automatic client management** - Activates when clients detected, pauses when none
- 📡 **Client count monitoring** - Listens to `/client_count` topic for automatic control
- 🛠️ **GStreamer pipeline** - Hardware-accelerated video processing

## Requirements

- NVIDIA Jetson AGX Orin (tested on 64GB model)
- ROS2 Humble
- Python 3.10+
- OpenCV with GStreamer support
- GStreamer 1.0+

## Usage

### Prerequisites

**Source the ROS environment:**
```bash
# Navigate to your workspace
cd /home/dev/lucy_ws

# Source ROS Humble
source /opt/ros/humble/setup.zsh

# Source your workspace
source install/setup.zsh
```

### Launch Methods

**Basic launch (camera activates automatically when clients connect):**
```bash
ros2 launch camera_ros camera.launch.py
```

**Launch with custom parameters:**
```bash
ros2 launch camera_ros camera.launch.py fps:=20.0 device:=/dev/video0
```

### Automatic Client Management

The camera automatically activates and deactivates based on client count:

**Client count monitoring:**
- Camera **activates** when `client_count > 0`
- Camera **pauses** when `client_count = 0`
- Monitors `/client_count` topic for automatic control

### Topics

**Published topics:**
- `/camera/mobius/jpg` - CompressedImage (JPEG format, published only when clients are present)

**Subscribed topics:**
- `/client_count` - Int32 (monitors client count for automatic activation/deactivation)

## Configuration

### Camera Settings

Edit `src/camera_publisher.py` to modify:

```python
# Frame rate (1.0 - 30.0 FPS)
FPS = 10.0

# GStreamer pipeline
GST_PIPELINE = (
    "v4l2src device=/dev/video0 ! "
    "image/jpeg,width=1280,height=720,framerate=30/1 ! "
    "jpegparse ! "
    "appsink drop=true emit-signals=true sync=false"
)
```

### Resolution Options

For different resolutions, modify the GStreamer pipeline:

```python
# 640x480 (lower CPU usage)
GST_PIPELINE = "v4l2src device=/dev/video0 ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegparse ! appsink drop=true emit-signals=true sync=false"

# 1920x1080 (higher quality, more CPU)
GST_PIPELINE = "v4l2src device=/dev/video0 ! image/jpeg,width=1920,height=1080,framerate=30/1 ! jpegparse ! appsink drop=true emit-signals=true sync=false"
```

## Performance Optimization

### System-Level Optimizations

1. **Set CPU governor to performance mode:**
   ```bash
   sudo jetson_clocks
   ```

2. **Check camera device:**
   ```bash
   ls /dev/video*
   v4l2-ctl --device=/dev/video0 --list-formats-ext
   ```

3. **Monitor performance:**
   ```bash
   # Monitor CPU usage
   htop
   
   # Monitor Jetson stats
   tegrastats
   
   # Check frame rate
   ros2 topic hz /camera/mobius/jpg
   ```

### Troubleshooting

#### Common Issues

**Camera not found:**
```bash
# Check USB devices
lsusb

# Check video devices
ls /dev/video*

# Check camera capabilities
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video0 --list-formats-ext

# Reload camera driver
sudo modprobe uvcvideo
```

**High CPU usage:**
- Reduce frame rate to 10-15 FPS
- Use lower resolution (640x480)
- Check for background processes
- Ensure CPU is in performance mode
- Use `jetson_clocks` for maximum performance

**ExternalShutdownException error:**
This usually occurs when the node is terminated externally. To prevent this:
```bash
# Use Ctrl+C to stop the node gracefully
# Or use the service to disable camera first
ros2 service call /camera/mobius/enable std_srvs/srv/SetBool "{data: false}"
```

## Architecture

```
Camera (/dev/video0)
    ↓
GStreamer Pipeline (v4l2src → jpegparse → appsink)
    ↓
OpenCV VideoCapture (zero-copy)
    ↓
ROS2 CompressedImage Publisher
    ↓
Client Applications
```

## API Reference

### CameraPublisher Class

**Methods:**
- `set_fps(fps)` - Set frame rate (1.0-30.0)
- `set_active(active)` - Enable/disable camera

**Parameters:**
- `fps` - Target frame rate
- `device` - Camera device path

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test on Jetson AGX Orin
5. Submit a pull request

## License

This project is licensed under the GNU General Public License v3.0 - see the LICENSE file for details.

## Authors

- **Sentience Robotics Team** - [contact@sentience-robotics.fr](mailto:contact@sentience-robotics.fr)

## Support

For issues and questions:
- Check the troubleshooting section
- Open an issue on GitHub
- Contact the development team
