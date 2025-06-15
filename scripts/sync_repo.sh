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

# Sync only specified folders from remote to local
echo "Syncing specified folders from remote repository at $REMOTE_IP:$REMOTE_REPO to local $LOCAL_REPO..."

# Sync myproject
echo "Syncing myproject..."
sshpass -p "$REMOTE_PASS" rsync -avz --delete \
    "$REMOTE_USER@$REMOTE_IP:$REMOTE_REPO/myproject/" "$LOCAL_REPO/myproject/"

# Sync software
echo "Syncing software..."
sshpass -p "$REMOTE_PASS" rsync -avz --delete \
    "$REMOTE_USER@$REMOTE_IP:$REMOTE_REPO/software/" "$LOCAL_REPO/software/"

# Sync ros_ws
echo "Syncing ros_ws..."
sshpass -p "$REMOTE_PASS" rsync -avz --delete \
    "$REMOTE_USER@$REMOTE_IP:$REMOTE_REPO/ros_ws/" "$LOCAL_REPO/ros_ws/"

echo "Sync complete."
