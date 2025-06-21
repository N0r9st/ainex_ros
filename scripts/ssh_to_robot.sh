#!/bin/bash

# Robot connection details (same as other scripts)
REMOTE_IP="192.168.149.1"
REMOTE_USER="ubuntu"
REMOTE_PASS="hiwonder"

# Check if sshpass is installed
if ! command -v sshpass &> /dev/null; then
    echo "sshpass is not installed. Installing..."
    sudo apt-get update && sudo apt-get install -y sshpass
fi

echo "Connecting to robot at $REMOTE_IP as $REMOTE_USER..."

# SSH to robot and execute /bin/bash
sshpass -p "$REMOTE_PASS" ssh -o StrictHostKeyChecking=no "$REMOTE_USER@$REMOTE_IP"
