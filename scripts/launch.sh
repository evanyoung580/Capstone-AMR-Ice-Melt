#!/bin/bash

base_station=false

VISION_MAIN="/home/amrmgr/amr/src/python/vision_main.py"
CMD_MANAGER="/home/amrmgr/amr/src/cpp//build/cmd_manager"
TELEM_MANAGER="/home/amrmgr/amr/src/cpp/build/telem_manager"

echo "Starting AMR..."

echo "Starting base station message manager..."
if $base_station; then
    echo "Base station mode enabled (TBD)."
fi

echo "Opening base command manager in a new terminal..."
qterminal --title "Cmd Manager" -e bash -c "$CMD_MANAGER; exec bash" &

echo "Opening telemetry monitor in a new terminal..."
qterminal --title "Telem Monitor" -e bash -c "$TELEM_MANAGER; exec bash" &

echo "Starting OpenCV vision..."
qterminal --title "OpenCV Vision" -e bash -c "python3 $VISION_MAIN; exec bash" &

VISION_PID=$!
CMD_MANAGER_PID=$(pgrep -f "$CMD_MANAGER")
TELEM_MANAGER_PID=$(pgrep -f "$TELEM_MANAGER")

trap "echo 'Stopping AMR...'; kill $VISION_PID $CMD_MANAGER_PID $TELEM_MANAGER_PID" SIGINT

wait

echo "AMR has stopped."
