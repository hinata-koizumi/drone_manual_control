#!/bin/bash

# ドローン手動制御デモ実行スクリプト

set -e

echo "🚁 Drone Manual Control Demo"
echo "============================"

# 1. 環境をビルド
echo "🔨 Building environment..."
docker-compose build

# 2. ブリッジノードを起動
echo "🌉 Starting bridge nodes..."
docker-compose up -d bridge

# 3. 少し待機
echo "⏳ Waiting for bridge to initialize..."
sleep 5

# 4. 手動制御ノードを起動
echo "🎮 Starting manual control..."
docker-compose up -d manual_control

# 5. Web可視化を起動
echo "🌐 Starting web visualization..."
docker-compose up -d web_viz

# 6. 少し待機
echo "⏳ Waiting for system to initialize..."
sleep 10

echo ""
echo "🎉 Demo is running!"
echo ""
echo "🌐 Web Visualization: http://localhost:8080"
echo ""
echo "Available commands:"
echo "- View logs: docker-compose logs -f manual_control"
echo "- View web viz logs: docker-compose logs -f web_viz"
echo "- Stop demo: docker-compose down"
echo "- Restart: docker-compose restart manual_control"
echo ""
echo "Predefined actions:"
echo "- hover: Maintain hover position"
echo "- takeoff: Takeoff sequence"
echo "- landing: Landing sequence"
echo "- waypoint_forward: Move forward 5m"
echo "- circle_flight: Circular flight pattern"
echo "- square_pattern: Square flight pattern"
echo ""
echo "To change action, edit config/action_sequences.yaml and restart manual_control"
echo ""
echo "💡 Tip: Use ./scripts/quick_start.sh for one-click startup!" 