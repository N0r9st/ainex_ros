#!/bin/bash

# Stop and remove old container (if exists)
docker stop ros_noetic_gpu 2>/dev/null
docker rm ros_noetic_gpu 2>/dev/null

# Rebuild the Docker image
docker build -t ros_noetic_nvidia:latest .

echo "Rebuild complete. Run ./start_container.sh to start."
