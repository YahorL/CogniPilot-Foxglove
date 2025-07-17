#!/bin/bash

# Run odometry bridge in the background
python3 odometry_bridge.py &
BRIDGE_PID=$!

# Wait a moment for the bridge to start
sleep 2

# Run foxglove bridge in the background
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
FOXGLOVE_BRIDGE_PID=$!

# Start Foxglove Studio in the background
foxglove-studio --url ws://localhost:8765 --layout ~/rdd2_fg_layout.json &
FOXGLOVE_PID=$!

echo "All processes started:"
echo "- Odometry Bridge (PID: $BRIDGE_PID)"
echo "- Foxglove Bridge (PID: $FOXGLOVE_BRIDGE_PID)"
echo "- Foxglove Studio (PID: $FOXGLOVE_PID)"
echo ""
echo "Press Ctrl+C to stop all processes..."

# Cleanup function
cleanup() {
    echo "Stopping all processes..."
    kill $BRIDGE_PID 2>/dev/null || true
    [ ! -z "$LAUNCH_PID" ] && kill $LAUNCH_PID 2>/dev/null || true
    kill $FOXGLOVE_PID 2>/dev/null || true
    echo "Done."
}

# Set up signal handling
trap cleanup EXIT INT TERM

# Wait for all background processes
wait
