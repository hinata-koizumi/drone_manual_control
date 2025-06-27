#!/bin/bash
set -e

# ROS 2環境を設定
source /opt/ros/humble/setup.bash

# ワークスペースをソース
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# 引数があればそのまま実行、なければcontrol_interfaceを起動
if [ $# -gt 0 ]; then
    exec "$@"
else
    # control_interfaceパッケージの起動
    ros2 run control_interface control_interface_node || { echo "control_interface failed"; exit 1; }
fi 