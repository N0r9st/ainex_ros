#!/bin/bash

# Remote repository details
REMOTE_IP="192.168.149.1"
REMOTE_USER="ubuntu"
REMOTE_PASS="hiwonder"
REMOTE_REPO="/home/ubuntu"
LOCAL_REPO="./robot"

# Check if sshpass is installed
if ! command -v sshpass &> /dev/null; then
    echo "sshpass is not installed. Installing..."
    sudo apt-get update && sudo apt-get install -y sshpass
fi

echo "Pulling specified folders from remote robot at $REMOTE_IP:$REMOTE_REPO to local $LOCAL_REPO..."

# Pull myproject
echo "Pulling myproject..."
sshpass -p "$REMOTE_PASS" rsync -avz --delete --exclude='__pycache__' \
    "$REMOTE_USER@$REMOTE_IP:$REMOTE_REPO/myproject/" "$LOCAL_REPO/myproject/"

# Pull software
echo "Pulling software..."
sshpass -p "$REMOTE_PASS" rsync -avz --delete --exclude='__pycache__' \
    "$REMOTE_USER@$REMOTE_IP:$REMOTE_REPO/software/" "$LOCAL_REPO/software/"

# Pull ros_ws
echo "Pulling ros_ws..."
sshpass -p "$REMOTE_PASS" rsync -avz --delete --exclude='__pycache__' \
    "$REMOTE_USER@$REMOTE_IP:$REMOTE_REPO/ros_ws/" "$LOCAL_REPO/ros_ws/"

echo "Pull complete." 