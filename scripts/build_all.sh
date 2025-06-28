#!/bin/bash
set -e

# 色付きのログ出力関数
log_info() {
    echo -e "\033[1;34mℹ️  $1\033[0m"
}

log_success() {
    echo -e "\033[1;32m✅ $1\033[0m"
}

log_error() {
    echo -e "\033[1;31m❌ $1\033[0m"
}

log_step() {
    echo -e "\033[1;36m📋 $1\033[0m"
}

echo "🚀 ドローン手動制御システム - 段階的ビルド開始"
echo ""

# 1. メッセージパッケージをビルド
log_step "📦 ステップ1: メッセージパッケージをビルド中..."
docker-compose build drone_msgs
if [ $? -eq 0 ]; then
    log_success "drone_msgs ビルド成功"
else
    log_error "drone_msgs ビルド失敗"
    exit 1
fi

# 2. ブリッジパッケージをビルド
log_step "🌉 ステップ2: ブリッジパッケージをビルド中..."
docker-compose build bridge
if [ $? -eq 0 ]; then
    log_success "bridge ビルド成功"
else
    log_error "bridge ビルド失敗"
    exit 1
fi

# 3. 手動制御パッケージをビルド
log_step "🎮 ステップ3: 手動制御パッケージをビルド中..."
docker-compose build manual_control
if [ $? -eq 0 ]; then
    log_success "manual_control ビルド成功"
else
    log_error "manual_control ビルド失敗"
    exit 1
fi

# 4. Web可視化パッケージをビルド
log_step "🌐 ステップ4: Web可視化パッケージをビルド中..."
docker-compose build web_viz
if [ $? -eq 0 ]; then
    log_success "web_viz ビルド成功"
else
    log_error "web_viz ビルド失敗"
    exit 1
fi

log_success "🎉 すべてのパッケージのビルドが完了しました！"
echo ""
echo "📋 次のコマンドでシステムを起動できます："
echo "   docker-compose up -d"
echo ""
echo "🚀 または、完全自動デプロイを使用："
echo "   ./scripts/auto_deploy.sh --start-only"
echo ""
echo "🔧 オプション：シミュレーターを有効にする場合は、"
echo "   docker-compose.yml の simulator セクションのコメントアウトを解除してください" 