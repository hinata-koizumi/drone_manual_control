#!/bin/bash
set -e

echo "🚀 ドローン手動制御システムを起動中..."

# システムを起動
docker-compose up -d

echo "⏳ システムの起動を待機中..."
sleep 5

# コンテナの状態を確認
echo "📊 コンテナの状態："
docker-compose ps

echo ""
echo "🔍 ログを確認するには："
echo "   docker-compose logs -f"
echo ""
echo "🎮 ドローン制御をテストするには："
echo "   docker exec \$(docker-compose ps -q manual_control) bash -c \"source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 topic pub /drone/control_command geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'\""
echo ""
echo "🛑 システムを停止するには："
echo "   docker-compose down" 