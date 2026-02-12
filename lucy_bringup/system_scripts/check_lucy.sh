#!/usr/bin/zsh
# Copyright 2025 Sentience Robotics Team
# Health check script for Lucy Robot System

SESSION_NAME="lucy"
WORKSPACE="$HOME/lucy_ws"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}🔍 Checking Lucy Robot System Status...${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if tmux session exists
echo -n "tmux session: "
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo -e "${GREEN}✅ Running${NC}"
    TMUX_RUNNING=true
else
    echo -e "${RED}❌ Not running${NC}"
    TMUX_RUNNING=false
    echo ""
    echo "Start with: ~/launch_lucy.sh"
    exit 1
fi

# Source ROS2 workspace
source $WORKSPACE/install/setup.zsh 2>/dev/null || true

echo ""
echo -e "${BLUE}ROS2 Nodes:${NC}"

# Check micro-ROS pico nodes (should be 2)
echo -n "  Pico Nodes (2 expected):  "
PICO_COUNT=$(ros2 node list 2>/dev/null | grep -c "pico_node" || echo "0")
if [ "$PICO_COUNT" -eq 2 ]; then
    echo -e "${GREEN}✅ Active ($PICO_COUNT/2)${NC}"
elif [ "$PICO_COUNT" -eq 1 ]; then
    echo -e "${YELLOW}⚠️  Partial ($PICO_COUNT/2)${NC}"
else
    echo -e "${RED}❌ Not found (0/2)${NC}"
fi

# Check camera node
echo -n "  Camera Publisher:        "
if ros2 node list 2>/dev/null | grep -q "camera_publisher"; then
    echo -e "${GREEN}✅ Active${NC}"
else
    echo -e "${RED}❌ Not found${NC}"
fi

# Check camera stream controller
echo -n "  Camera Stream Controller: "
if ros2 node list 2>/dev/null | grep -q "camera_stream_controller"; then
    echo -e "${GREEN}✅ Active${NC}"
else
    echo -e "${RED}❌ Not found${NC}"
fi

# Check rosbridge
echo -n "  ROSBridge Server:        "
if ros2 node list 2>/dev/null | grep -q "rosbridge"; then
    echo -e "${GREEN}✅ Active${NC}"
else
    echo -e "${RED}❌ Not found${NC}"
fi

# Check realsense node
echo -n "  Realsense Camera:         "
if ros2 node list 2>/dev/null | grep -q "realsense2_camera"; then
    echo -e "${GREEN}✅ Active${NC}"
else
    echo -e "${RED}❌ Not found${NC}"
fi

# Check audio nodes
echo -n "  Audio Capturer:          "
if ros2 node list 2>/dev/null | grep -q "audio_capturer"; then
    echo -e "${GREEN}✅ Active${NC}"
else
    echo -e "${RED}❌ Not found${NC}"
fi

echo -n "  Audio Player:            "
if ros2 node list 2>/dev/null | grep -q "audio_player"; then
    echo -e "${GREEN}✅ Active${NC}"
else
    echo -e "${RED}❌ Not found${NC}"
fi

echo ""
echo -e "${BLUE}Camera Services:${NC}"

# Check camera services
for service in "/camera_publisher/start_streaming" "/camera_publisher/stop_streaming"; do
    echo -n "  $service: "
    if ros2 service list 2>/dev/null | grep -q "$service"; then
        echo -e "${GREEN}✅ Available${NC}"
    else
        echo -e "${YELLOW}⚠️  Not found${NC}"
    fi
done

echo ""
echo -e "${BLUE}Key Topics:${NC}"

# Check important topics
for topic in "/joints/right_arm" "/joints/left_arm" "/trace_publisher" "/ext_camera/jpg" "/mic_audio" "/audio"; do
    echo -n "  $topic: "
    if ros2 topic list 2>/dev/null | grep -q "$topic"; then
        echo -e "${GREEN}✅ Available${NC}"
    else
        echo -e "${YELLOW}⚠️  Not found${NC}"
    fi
done

echo ""
echo -e "${BLUE}Serial Devices:${NC}"

# Check serial devices
for device in "/dev/ttyACM0" "/dev/ttyACM1"; do
    echo -n "  $device: "
    if [ -e "$device" ]; then
        echo -e "${GREEN}✅ Connected${NC}"
    else
        echo -e "${RED}❌ Not found${NC}"
    fi
done

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Commands:${NC}"
echo "  Attach:  tmux attach -t $SESSION_NAME"
echo "  Stop:    ~/stop_lucy.sh"
echo -e "${BLUE}========================================${NC}"

