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

# ヘルプ表示
show_help() {
    echo "🚀 ドローン手動制御システム - 完全自動デプロイスクリプト"
    echo ""
    echo "使用方法:"
    echo "  $0 [オプション]"
    echo ""
    echo "オプション:"
    echo "  --build-only     ビルドのみ実行（システムは起動しない）"
    echo "  --start-only     既存のビルドを使用してシステムのみ起動"
    echo "  --no-web-viz     Web可視化を無効にして起動"
    echo "  --clean          既存のコンテナとイメージを削除してから開始"
    echo "  --help           このヘルプを表示"
    echo ""
    echo "例:"
    echo "  $0                # 完全自動デプロイ"
    echo "  $0 --build-only   # ビルドのみ"
    echo "  $0 --clean        # クリーンな状態から開始"
}

# 引数解析
BUILD_ONLY=false
START_ONLY=false
NO_WEB_VIZ=false
CLEAN=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --build-only)
            BUILD_ONLY=true
            shift
            ;;
        --start-only)
            START_ONLY=true
            shift
            ;;
        --no-web-viz)
            NO_WEB_VIZ=true
            shift
            ;;
        --clean)
            CLEAN=true
            shift
            ;;
        --help)
            show_help
            exit 0
            ;;
        *)
            log_error "不明なオプション: $1"
            show_help
            exit 1
            ;;
    esac
done

# メイン処理
main() {
    log_info "🚀 ドローン手動制御システム - 完全自動デプロイ開始"
    echo ""

    # クリーンアップ（オプション）
    if [ "$CLEAN" = true ]; then
        log_step "🧹 既存のコンテナとイメージをクリーンアップ中..."
        docker-compose down --remove-orphans
        docker system prune -f
        log_success "クリーンアップ完了"
        echo ""
    fi

    # ビルド処理
    if [ "$START_ONLY" = false ]; then
        log_step "📦 ステップ1: メッセージパッケージをビルド中..."
        docker-compose build drone_msgs
        if [ $? -eq 0 ]; then
            log_success "drone_msgs ビルド成功"
        else
            log_error "drone_msgs ビルド失敗"
            exit 1
        fi

        log_step "🌉 ステップ2: ブリッジパッケージをビルド中..."
        docker-compose build bridge
        if [ $? -eq 0 ]; then
            log_success "bridge ビルド成功"
        else
            log_error "bridge ビルド失敗"
            exit 1
        fi

        log_step "🎮 ステップ3: 手動制御パッケージをビルド中..."
        docker-compose build manual_control
        if [ $? -eq 0 ]; then
            log_success "manual_control ビルド成功"
        else
            log_error "manual_control ビルド失敗"
            exit 1
        fi

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
    fi

    # システム起動処理
    if [ "$BUILD_ONLY" = false ]; then
        log_step "🚀 ステップ5: システムを起動中..."
        
        # Web可視化の無効化（オプション）
        if [ "$NO_WEB_VIZ" = true ]; then
            log_warning "Web可視化を無効にして起動します"
            docker-compose up -d bridge manual_control
        else
            docker-compose up -d
        fi

        log_success "システム起動完了"
        echo ""

        log_step "⏳ システムの初期化を待機中..."
        sleep 10

        # コンテナの状態確認
        log_step "📊 コンテナの状態確認中..."
        docker-compose ps
        echo ""

        # ヘルスチェック
        log_step "🔍 システムのヘルスチェック中..."
        
        # ROS 2トピックの確認
        sleep 5
        if docker-compose exec -T manual_control bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && timeout 5 ros2 topic list | grep -q '/drone/position'" 2>/dev/null; then
            log_success "ドローン位置トピックが正常に動作しています"
        else
            log_warning "ドローン位置トピックの確認に失敗しました"
        fi

        # Web可視化の確認
        if [ "$NO_WEB_VIZ" = false ]; then
            sleep 3
            if curl -s http://localhost:8080 > /dev/null; then
                log_success "Web可視化サーバーが正常に動作しています"
            else
                log_warning "Web可視化サーバーへの接続に失敗しました"
            fi
        fi

        echo ""
        log_success "🎉 システムの自動デプロイが完了しました！"
        echo ""
        
        # 使用可能な機能の表示
        echo "📋 利用可能な機能："
        echo "   🌐 Web可視化: http://localhost:8080"
        echo "   📊 ログ確認: docker-compose logs -f"
        echo "   🎮 手動制御: ブラウザの制御ボタンを使用"
        echo ""
        
        if [ "$NO_WEB_VIZ" = false ]; then
            echo "🌐 Web可視化を開くには："
            echo "   open http://localhost:8080"
            echo ""
        fi
        
        echo "🛑 システムを停止するには："
        echo "   docker-compose down"
        echo ""
        
        # 自動でブラウザを開く（オプション）
        if [ "$NO_WEB_VIZ" = false ]; then
            read -p "🌐 ブラウザでWeb可視化を開きますか？ (y/N): " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                open http://localhost:8080
                log_success "ブラウザでWeb可視化を開きました"
            fi
        fi
    fi
}

# エラーハンドリング
trap 'log_error "スクリプトがエラーで終了しました"; exit 1' ERR

# メイン処理実行
main "$@" 