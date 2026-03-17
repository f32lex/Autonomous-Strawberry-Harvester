#!/bin/bash
# Usage: ./run_microros_agent.sh /dev/ttyACM1
# Example: ./run_microros_agent.sh /dev/ttyACM1

PORT=${1:-/dev/ttyACM1}

echo "Starting micro-ROS agent on $PORT"
echo "Press Ctrl+C to stop."

sudo docker run -it --rm \
    -v /dev:/dev \
    -v /dev/shm:/dev/shm \
    --privileged \
    --net=host \
    microros/micro-ros-agent:humble \
    serial --dev $PORT -v6
