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
    # manual_controlパッケージの起動（直接launchファイルを実行）
    if [ -f /workspace/install/share/manual_control/launch/simple_demo_launch.py ]; then
        echo "simple_demo_launch.pyを直接実行します"
        exec python3 /workspace/install/share/manual_control/launch/simple_demo_launch.py
    else
        echo "simple_demo_launch.pyが見つかりません"
        exit 1
    fi
fi 