#!/bin/bash
set -e

# 色付きのログ出力関数
log_info() {
    echo -e "\033[1;34mℹ️  $1\033[0m"
}

log_success() {
    echo -e "\033[1;32m✅ $1\033[0m"
}

log_warning() {
    echo -e "\033[1;33m⚠️  $1\033[0m"
}

log_error() {
    echo -e "\033[1;31m❌ $1\033[0m"
}

log_step() {
    echo -e "\033[1;36m📋 $1\033[0m"
}

echo "🚀 ドローン手動制御システム - クイックスタート"
echo ""

# 既存のコンテナを停止
log_step "🛑 既存のコンテナを停止中..."
docker-compose down 2>/dev/null || true

# システムを起動
log_step "🚀 システムを起動中..."
docker-compose up -d

log_success "システム起動完了"
echo ""

log_step "⏳ システムの初期化を待機中..."
sleep 15

# コンテナの状態を確認
log_step "📊 コンテナの状態："
docker-compose ps
echo ""

# ヘルスチェック
log_step "🔍 システムのヘルスチェック中..."

# Web可視化の確認
sleep 3
if curl -s http://localhost:8080 > /dev/null; then
    log_success "Web可視化サーバーが正常に動作しています"
else
    log_warning "Web可視化サーバーへの接続に失敗しました"
fi

# ROS 2トピックの確認
sleep 5
if docker-compose exec -T manual_control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && timeout 5 ros2 topic list | grep -q '/drone/position'" 2>/dev/null; then
    log_success "ドローン位置トピックが正常に動作しています"
else
    log_warning "ドローン位置トピックの確認に失敗しました"
fi

echo ""
log_success "🎉 システムの起動が完了しました！"
echo ""

# 使用可能な機能の表示
echo "📋 利用可能な機能："
echo "   🌐 Web可視化: http://localhost:8080"
echo "   📊 ログ確認: docker-compose logs -f"
echo "   🎮 手動制御: ブラウザの制御ボタンを使用"
echo ""

# 自動でブラウザを開く
log_step "🌐 ブラウザでWeb可視化を開いています..."
open http://localhost:8080
log_success "ブラウザでWeb可視化を開きました"

echo ""
echo "🛑 システムを停止するには："
echo "   docker-compose down"
echo ""
echo "📊 ログを確認するには："
echo "   docker-compose logs -f" 