#!/bin/bash
set -e

echo "🚀 ドローン手動制御システム - 段階的ビルド開始"

# 1. メッセージパッケージをビルド
echo "📦 ステップ1: メッセージパッケージをビルド中..."
docker-compose build drone_msgs
if [ $? -eq 0 ]; then
    echo "✅ drone_msgs ビルド成功"
else
    echo "❌ drone_msgs ビルド失敗"
    exit 1
fi

# 2. ブリッジパッケージをビルド
echo "🌉 ステップ2: ブリッジパッケージをビルド中..."
docker-compose build bridge
if [ $? -eq 0 ]; then
    echo "✅ bridge ビルド成功"
else
    echo "❌ bridge ビルド失敗"
    exit 1
fi

# 3. 手動制御パッケージをビルド
echo "🎮 ステップ3: 手動制御パッケージをビルド中..."
docker-compose build manual_control
if [ $? -eq 0 ]; then
    echo "✅ manual_control ビルド成功"
else
    echo "❌ manual_control ビルド失敗"
    exit 1
fi

echo "🎉 すべての基本パッケージのビルドが完了しました！"
echo ""
echo "📋 次のコマンドでシステムを起動できます："
echo "   docker-compose up -d"
echo ""
echo "🔧 オプション：シミュレーターを有効にする場合は、"
echo "   docker-compose.yml の simulator セクションのコメントアウトを解除してください" 