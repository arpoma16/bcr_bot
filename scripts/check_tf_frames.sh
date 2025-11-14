#!/bin/bash

echo "Checking TF frames and topics..."
echo ""
echo "=== Available TF frames ==="
timeout 5s ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -20 || echo "Could not find transform"
echo ""
echo "=== TF tree ==="
timeout 5s ros2 run tf2_tools view_frames 2>&1 || echo "Could not generate TF tree"
echo ""
echo "=== Scan topic info ==="
ros2 topic info /bcr_bot/scan
echo ""
echo "=== Scan topic data (first message) ==="
timeout 3s ros2 topic echo /bcr_bot/scan --once
