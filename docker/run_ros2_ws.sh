#!/usr/bin/env bash

docker run -it --rm \
  --device=/dev/ttyACM0:/dev/ttyACM0 \
  -v /home/pathfinder1/sos_rover/ros2_ws:/ros2_ws \
  -w /ros2_ws \
  sos-ros-humble-dev \
  bash

