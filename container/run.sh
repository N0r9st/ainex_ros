#!/bin/bash

# Allow X11 access (run on host machine first)
xhost +local:docker

# Start the container with GPU and X11 forwarding
docker run -it \
  --gpus all \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  --net=host \
  --name ros_noetic_gpu \
  ros_noetic_nvidia:latest
