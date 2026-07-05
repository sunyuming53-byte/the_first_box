#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

echo "RealMan development container ready."
echo "  Workspace:   /ws"
echo "  SDK:         src/omr_hardware/third_party/realman_arm/third_party/RM_API2 (submodule)"
echo "  SSH key:     ~/.ssh/id_rsa  (deploy with: deploy-remote <robot-ip>)"
echo ""

if [ $# -eq 0 ]; then
    exec zsh
else
    exec "$@"
fi
