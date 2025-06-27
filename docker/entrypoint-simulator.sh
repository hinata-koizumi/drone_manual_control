#!/bin/bash
set -e

# ROS 2環境を設定
source /opt/ros/humble/setup.bash

# ワークスペースをソース
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# 引数があればそのまま実行、なければsim_launchを起動
if [ $# -gt 0 ]; then
    exec "$@"
else
    # sim_launchパッケージの起動
    ros2 launch sim_launch gz_sim.launch.py || { echo "sim_launch failed"; exit 1; }
fi 