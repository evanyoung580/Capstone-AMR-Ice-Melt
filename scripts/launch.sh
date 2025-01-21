#!/bin/bash

base_station=false

VISION_MAIN="/home/amrmgr/amr/src/python/vision_main.py"
MSGMGR="/home/amrmgr/amr/test/cpp/build/local_msgmgr"

echo "Starting AMR..."

echo "Starting base station message manager..."
if $base_station; then
    echo "Base station mode enabled (TBD)."
else
    if [[ ! -p data_monitor ]]; then
        echo "Creating FIFO data_monitor..."
        mkfifo data_monitor
    else
        echo "FIFO data_monitor already exists."
    fi
    echo "Opening data monitor terminal..."
    qterminal -e bash -c "cat data_monitor; exec bash" &
fi

echo "Opening base station manager in a new terminal..."
qterminal -e bash -c "$MSGMGR; exec bash" &

echo "Starting OpenCV vision..."
python3 $VISION_MAIN &

VISION_PID=$!
MSGMGR_PID=$(pgrep -f "$MSGMGR")

trap "echo 'Stopping AMR...'; kill $VISION_PID $MSGMGR_PID" SIGINT

wait
echo "AMR has stopped."
