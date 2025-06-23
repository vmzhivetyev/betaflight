#!/bin/bash

while true; do
    SITL_IP="$(ip route show | grep -i default | awk '{ print $3 }')"
    echo ">>> SITL IP: $SITL_IP"

    echo ">>> Initializing websockify..."
    websockify 127.0.0.1:6761 127.0.0.1:5761 &
    WS_PID=$!
    trap "kill $WS_PID" EXIT

    echo ">>> Starting SITL..."
    ./obj/main/betaflight_SITL.elf "$SITL_IP"

    echo ">>> SITL stopped, stopping websockify..."
    kill $WS_PID
    wait $WS_PID 2>/dev/null
    echo ">>> SITL and websockify stopped, restarting..."
    sleep 2
done
