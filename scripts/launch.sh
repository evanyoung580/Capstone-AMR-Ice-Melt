#!/bin/bash

base_station=false

VISION_MAIN="/home/amrmgr/amr/src/python/vision_main.py"
ROBOT_CTRL="/home/amrmgr/amr/build/robot_ctrl"

echo "Starting AMR..."

echo "Starting base station message manager..."
if $base_station; then
    echo "Base station mode enabled (TBD)."
fi

echo "running robot controller in a new terminal..."
qterminal --title "Robot Ctrl" -e bash -c "$ROBOT_CTRL; exec bash" &

echo "Starting OpenCV vision..."
qterminal --title "OpenCV Vision" -e bash -c "python3 $VISION_MAIN; exec bash" &

VISION_PID=$!
ROBOT_CTRL_PID=$(pgrep -f "$ROBOT_CTRL")

trap "echo 'Stopping AMR...'; kill $VISION_PID $OBOT_CTRL_PID" SIGINT

wait

echo "AMR has stopped."
