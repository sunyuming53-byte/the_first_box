#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
    echo "Workspace sourced from /ws/install"
else
    echo "WARNING: /ws/install/setup.bash not found — no built artifacts"
fi

service ssh start

echo "OMRobot runtime starting..."
echo "  sshd:2022   — remote access"
echo "  bringup.launch.py — full robot stack (arm+dais+m65+camera+foxglove+LIO/Nav2)"
echo ""

exec /usr/bin/supervisord -n -c /etc/supervisor/conf.d/omrobot.conf
