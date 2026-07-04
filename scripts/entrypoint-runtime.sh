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

echo "RealMan runtime starting..."
echo "  sshd:22     — remote access"
echo "  arm_node    — arm control + stop service"
echo "  calib_node  — calibration pipeline services"
echo ""

exec /usr/bin/supervisord -n -c /etc/supervisor/conf.d/realman.conf
