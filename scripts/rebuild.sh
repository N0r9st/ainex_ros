#!/bin/bash

# Stop and remove old container
docker stop ros_noetic_gpu 2>/dev/null
docker rm ros_noetic_gpu 2>/dev/null

# Rebuild with host user args
docker build \
  -f ./pc/container/Dockerfile \
  --build-arg USERNAME=$(whoami) \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  -t ros_noetic_nvidia:latest .

echo "Rebuild complete. Run scripts/run.sh to start."
