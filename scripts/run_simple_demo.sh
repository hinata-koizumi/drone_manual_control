#!/bin/bash

# 簡単なドローンシミュレーションデモ実行スクリプト

set -e

echo "🚁 Simple Drone Simulation Demo"
echo "================================"

# 環境変数を設定
export ROS_DOMAIN_ID=0

# 1. パッケージをビルド
echo "🔨 Building manual_control package..."
cd src/manual_control
colcon build --packages-select manual_control

# 2. 環境をソース
echo "📦 Sourcing environment..."
source install/setup.sh

# 3. 簡単なデモを起動
echo "🚀 Starting simple drone simulation..."
echo ""
echo "This will start:"
echo "- Simple drone simulator"
echo "- Action executor node"
echo "- State monitor node"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# launchファイルを実行
ros2 launch manual_control simple_demo.launch.py

echo ""
echo "Demo finished!" 