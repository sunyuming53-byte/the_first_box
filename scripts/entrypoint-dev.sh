#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

echo "RealMan development container ready."
echo "  Workspace:   /ws"
echo "  SDK:         /opt/realman-sdk"
echo "  SSH key:     ~/.ssh/id_rsa  (deploy with: deploy-remote <robot-ip>)"
echo ""

exec zsh "$@"
