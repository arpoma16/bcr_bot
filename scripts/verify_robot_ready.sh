#!/bin/bash

# Script para verificar que el robot está listo para mapping
# Verifica topics, transformaciones y configuración antes de lanzar SLAM

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "======================================"
echo "Verifying BCR Bot is ready for mapping"
echo "======================================"

# Function to check if a topic exists
check_topic() {
    local topic=$1
    echo -n "Checking topic $topic... "
    if timeout 5s bash -c "ros2 topic list | grep -q \"$topic\""; then
        echo -e "${GREEN}OK${NC}"
        return 0
    else
        echo -e "${RED}NOT FOUND${NC}"
        return 1
    fi
}

# Function to check if a transform exists
check_transform() {
    local from=$1
    local to=$2
    echo -n "Checking transform $from -> $to... "
    if timeout 5s ros2 run tf2_ros tf2_echo $from $to > /dev/null 2>&1; then
        echo -e "${GREEN}OK${NC}"
        return 0
    else
        echo -e "${RED}NOT AVAILABLE${NC}"
        return 1
    fi
}

# Check critical topics
echo ""
echo "=== Checking Topics ==="
check_topic "/bcr_bot/scan"
check_topic "/bcr_bot/odom"
check_topic "/clock"

# Check scan message frame_id
echo ""
echo "=== Checking Scan Message ==="
echo -n "Reading scan frame_id... "
FRAME_ID=$(timeout 3s ros2 topic echo /bcr_bot/scan --once 2>/dev/null | grep "frame_id:" | awk '{print $2}' | tr -d "'\"")
if [ -n "$FRAME_ID" ]; then
    echo -e "${GREEN}$FRAME_ID${NC}"
else
    echo -e "${RED}FAILED TO READ${NC}"
    exit 1
fi

# Check TF tree
echo ""
echo "=== Checking TF Transforms ==="
check_transform "odom" "base_link" || echo -e "${YELLOW}Warning: odom->base_link not available. Check odometry publisher.${NC}"
check_transform "base_link" "$FRAME_ID" || echo -e "${YELLOW}Warning: base_link->$FRAME_ID not available. Check robot_state_publisher.${NC}"

# Display TF tree
echo ""
echo "=== Current TF Tree ==="
timeout 5s ros2 run tf2_tools view_frames 2>/dev/null || echo "Could not generate TF tree"
if [ -f "frames.pdf" ]; then
    echo "TF tree saved to frames.pdf"
fi

# Check scan rate
echo ""
echo "=== Checking Scan Rate ==="
echo -n "Measuring scan topic rate (this may take a few seconds)... "
RATE=$(timeout 10s ros2 topic hz /bcr_bot/scan 2>&1 | grep "average rate:" | awk '{print $3}')
if [ -n "$RATE" ]; then
    echo -e "${GREEN}${RATE} Hz${NC}"
else
    echo -e "${YELLOW}Could not measure${NC}"
fi

echo ""
echo "======================================"
echo -e "${GREEN}Verification complete!${NC}"
echo "======================================"
echo ""
echo "If all checks passed, you can launch mapping with:"
echo "  ros2 launch bcr_bot mapping.launch.py"
echo ""
