#!/usr/bin/env bash
# Sources the full stack for every interactive/exec'd shell in the image:
# ROS2, this workspace's colcon overlay, the Python venv (torch/scipy/etc),
# and PX4's gz_env.sh (sets GZ_SIM_RESOURCE_PATH -- without it gz-sim can't
# resolve model://tricopter and the vehicle silently never spawns, see
# History.md). Then hands off to whatever command the container was run
# with (default: bash).
set -e

source /opt/ros/"${ROS_DISTRO:-jazzy}"/setup.bash

if [ -f "$HOME/AeroCanyon_Guard/install/setup.bash" ]; then
    source "$HOME/AeroCanyon_Guard/install/setup.bash"
fi

if [ -f "$HOME/AeroCanyon_Guard/.venv/bin/activate" ]; then
    source "$HOME/AeroCanyon_Guard/.venv/bin/activate"
fi

if [ -f "$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh" ]; then
    source "$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh"
fi

export GZ_IP="${GZ_IP:-127.0.0.1}"

exec "$@"
