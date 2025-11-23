#!/usr/bin/zsh
# Copyright 2024 Sentience Robotics Team
# Stop script for Lucy Robot System

SESSION_NAME="lucy"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}🛑 Stopping Lucy Robot System...${NC}"
echo -e "${BLUE}========================================${NC}"

# Check if tmux session exists
if ! tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo -e "${YELLOW}⚠️  Session '$SESSION_NAME' is not running${NC}"
    exit 0
fi

echo -e "${BLUE}📋 Gracefully stopping processes...${NC}"

# Stop ROS nodes (pane 0)
echo -e "${BLUE}  → Stopping ROS2 nodes...${NC}"
tmux send-keys -t $SESSION_NAME:0.0 C-c 2>/dev/null || true
sleep 2

# Stop web interface (pane 1)
echo -e "${BLUE}  → Stopping web interface...${NC}"
tmux send-keys -t $SESSION_NAME:0.1 C-c 2>/dev/null || true
sleep 1

# Kill the tmux session
echo -e "${BLUE}  → Terminating tmux session...${NC}"
tmux kill-session -t $SESSION_NAME 2>/dev/null || true

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}✅ Lucy system stopped successfully!${NC}"
echo -e "${GREEN}========================================${NC}"

