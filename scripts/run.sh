#!/bin/bash

# Allow X11 access
xhost +local:docker

# Get current user/group info
USER_ID=$(id -u)
GROUP_ID=$(id -g)
USERNAME=$(whoami)
WORKDIR=$(pwd)
CONTAINER_NAME="ros_noetic_gpu"
IMAGE_NAME="ros_noetic_nvidia:latest"

# Check if container exists
if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    # Container exists
    if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
        # Container is running
        echo "Container $CONTAINER_NAME is already running. Attaching..."
    else
        # Container exists but stopped, start it
        echo "Container $CONTAINER_NAME exists but is stopped. Starting..."
        docker start $CONTAINER_NAME
    fi
else
    # Container doesn't exist, create and run
    echo "Creating new container $CONTAINER_NAME..."
    docker run -d -it \
        --gpus all \
        --network=host \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="USER_ID=$USER_ID" \
        --env="GROUP_ID=$GROUP_ID" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$HOME/.Xauthority:/home/$USERNAME/.Xauthority:rw" \
        --volume="$WORKDIR:$WORKDIR" \
        --workdir="$WORKDIR" \
        --user="$USER_ID:$GROUP_ID" \
        --name $CONTAINER_NAME \
        $IMAGE_NAME
fi

docker exec -it \
    --user="$USER_ID:$GROUP_ID" \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    $CONTAINER_NAME bash
