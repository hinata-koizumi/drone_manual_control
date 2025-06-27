#!/bin/bash
set -e

# ROS 2環境を設定
source /opt/ros/humble/setup.bash

# ワークスペースをソース
if [ -f /bridge_ws/install/setup.bash ]; then
    source /bridge_ws/install/setup.bash
fi

# 引数があればそのまま実行、なければstate_bridgeを起動
if [ $# -gt 0 ]; then
    exec "$@"
else
    # state_bridge起動（直接実行可能ファイルを呼び出し）
    if [ -f /bridge_ws/install/bin/state_bridge_node ]; then
        echo "state_bridge_nodeを直接実行します"
        exec /bridge_ws/install/bin/state_bridge_node
    else
        echo "state_bridge_nodeが見つかりません"
        exit 1
    fi
fi 