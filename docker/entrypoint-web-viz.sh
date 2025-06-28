#!/bin/bash
set -e

# ROS 2環境を設定
source /opt/ros/humble/setup.bash

# ワークスペースをソース
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# 引数があればそのまま実行、なければWeb視覚化を起動
if [ $# -gt 0 ]; then
    exec "$@"
else
    echo "Starting Web Visualization Server..."
    
    # WebサーバーとWebSocketサーバーを起動
    cd /workspace/web_viz
    python3 server.py
    
    echo "Web visualization available at: http://localhost:8080"
    echo "WebSocket server running on port 8081"
fi 