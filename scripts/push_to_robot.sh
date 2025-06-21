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

echo "Pushing specified folders from local $LOCAL_REPO to remote robot at $REMOTE_IP:$REMOTE_REPO..."

# Push myproject
echo "Pushing myproject..."
sshpass -p "$REMOTE_PASS" rsync -avz --delete --exclude='__pycache__' \
    "$LOCAL_REPO/myproject/" "$REMOTE_USER@$REMOTE_IP:$REMOTE_REPO/myproject/"

# Push software
echo "Pushing software..."
sshpass -p "$REMOTE_PASS" rsync -avz --delete --exclude='__pycache__' \
    "$LOCAL_REPO/software/" "$REMOTE_USER@$REMOTE_IP:$REMOTE_REPO/software/"

# Push ros_ws
echo "Pushing ros_ws..."
sshpass -p "$REMOTE_PASS" rsync -avz --delete --exclude='__pycache__' \
    "$LOCAL_REPO/ros_ws/" "$REMOTE_USER@$REMOTE_IP:$REMOTE_REPO/ros_ws/"

echo "Push complete." 