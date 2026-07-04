#!/bin/zsh
source /opt/ros/humble/setup.bash
colcon build --packages-select realman_arm --cmake-args -DCLANG_TIDY=ON 2>&1
