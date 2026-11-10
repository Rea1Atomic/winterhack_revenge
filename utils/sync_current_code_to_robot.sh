#!/bin/sh

# This script is for testing codes in development.
# It will read your robot's IP and remote workspace via
# evironment variables.
# Then it uses rsync to copy the new code to your robot and build on it.
#
# The evironment variables you should provide:
# $REMOTE_IP
# $REMOTE_USERNAME

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname $SCRIPT_DIR)"

echo "====> Sync $REPO_DIR/ros_ws to $REMOTE_USERNAME@$REMOTE_IP:/tmp/winterhack_test"

rsync -avh \
    --exclude="build/" \
    --exclude="install/" \
    --exclude="log/" \
    "$REPO_DIR/ros_ws" \
    "$REMOTE_USERNAME@$REMOTE_IP:/tmp/winterhack_test/"

ssh $REMOTE_USERNAME@$REMOTE_IP "distrobox enter ubuntu-ros -- bash -c 'source /opt/ros/humble/setup.bash;cd /tmp/winterhack_test/ros_ws;colcon build'"
