#!/bin/bash
set -e

# ROS 2環境を設定
source /opt/ros/humble/setup.bash

# ワークスペースをソース
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# 引数があればそのまま実行、なければmanual_controlパッケージを起動
if [ $# -gt 0 ]; then
    exec "$@"
else
    # manual_controlパッケージの起動（直接Pythonモジュールを実行）
    echo "Starting manual_control package..."
    exec python3 -m manual_control.simple_simulator
fi 