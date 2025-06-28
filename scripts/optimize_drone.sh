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
    echo "🚀 ドローン性能最適化スクリプト"
    echo ""
    echo "使用方法:"
    echo "  $0 [オプション]"
    echo ""
    echo "オプション:"
    echo "  --simulator    最適化されたシミュレーターを使用"
    echo "  --config       最適化された設定ファイルを使用"
    echo "  --performance  パフォーマンステストを実行"
    echo "  --benchmark    ベンチマークテストを実行"
    echo "  --all          すべての最適化を適用"
    echo "  --help         このヘルプを表示"
    echo ""
    echo "例:"
    echo "  $0 --all        # すべての最適化を適用"
    echo "  $0 --simulator  # 最適化シミュレーターのみ"
    echo "  $0 --benchmark  # ベンチマークテストのみ"
}

# 引数解析
USE_OPTIMIZED_SIM=false
USE_OPTIMIZED_CONFIG=false
RUN_PERFORMANCE_TEST=false
RUN_BENCHMARK=false
APPLY_ALL=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --simulator)
            USE_OPTIMIZED_SIM=true
            shift
            ;;
        --config)
            USE_OPTIMIZED_CONFIG=true
            shift
            ;;
        --performance)
            RUN_PERFORMANCE_TEST=true
            shift
            ;;
        --benchmark)
            RUN_BENCHMARK=true
            shift
            ;;
        --all)
            APPLY_ALL=true
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
    log_info "🚀 ドローン性能最適化開始"
    echo ""

    if [ "$APPLY_ALL" = true ]; then
        USE_OPTIMIZED_SIM=true
        USE_OPTIMIZED_CONFIG=true
        RUN_PERFORMANCE_TEST=true
        RUN_BENCHMARK=true
    fi

    # 最適化されたシミュレーターの適用
    if [ "$USE_OPTIMIZED_SIM" = true ]; then
        log_step "🔧 最適化されたシミュレーターを適用中..."
        
        # 最適化されたシミュレーターをコピー
        if [ -f "src/manual_control/manual_control/optimized_simulator.py" ]; then
            cp src/manual_control/manual_control/optimized_simulator.py \
               src/manual_control/manual_control/simple_simulator.py
            log_success "最適化されたシミュレーターを適用しました"
        else
            log_error "最適化されたシミュレーターファイルが見つかりません"
            exit 1
        fi
    fi

    # 最適化された設定ファイルの適用
    if [ "$USE_OPTIMIZED_CONFIG" = true ]; then
        log_step "⚙️  最適化された設定ファイルを適用中..."
        
        if [ -f "config/optimized_drone_specs.yaml" ]; then
            cp config/optimized_drone_specs.yaml config/drone_specs.yaml
            log_success "最適化された設定ファイルを適用しました"
        else
            log_error "最適化された設定ファイルが見つかりません"
            exit 1
        fi
    fi

    # パフォーマンステスト
    if [ "$RUN_PERFORMANCE_TEST" = true ]; then
        log_step "📊 パフォーマンステストを実行中..."
        
        # システムを起動してパフォーマンスをテスト
        docker-compose down 2>/dev/null || true
        docker-compose build manual_control
        docker-compose up -d bridge manual_control
        
        log_info "⏳ システムの初期化を待機中..."
        sleep 15
        
        # パフォーマンスメトリクスの確認
        log_info "🔍 パフォーマンスメトリクスを確認中..."
        
        # シミュレーションFPSの確認
        if docker-compose logs manual_control | grep -q "Simulation FPS"; then
            fps=$(docker-compose logs manual_control | grep "Simulation FPS" | tail -1 | awk '{print $NF}')
            log_success "シミュレーションFPS: $fps"
        else
            log_warning "FPS情報が見つかりません"
        fi
        
        # メモリ使用量の確認
        memory_usage=$(docker stats --no-stream --format "table {{.MemUsage}}" manual_control | tail -1)
        log_info "メモリ使用量: $memory_usage"
        
        # CPU使用量の確認
        cpu_usage=$(docker stats --no-stream --format "table {{.CPUPerc}}" manual_control | tail -1)
        log_info "CPU使用量: $cpu_usage"
    fi

    # ベンチマークテスト
    if [ "$RUN_BENCHMARK" = true ]; then
        log_step "🏃 ベンチマークテストを実行中..."
        
        # 制御コマンドのベンチマーク
        log_info "🎮 制御レスポンステスト中..."
        
        # 上昇コマンドのテスト
        start_time=$(date +%s.%N)
        docker-compose exec -T manual_control bash -c "
            source /opt/ros/humble/setup.bash && 
            source /workspace/install/setup.bash && 
            timeout 5 ros2 topic pub /drone/control_command geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'"
        end_time=$(date +%s.%N)
        
        response_time=$(echo "$end_time - $start_time" | bc)
        log_success "制御レスポンス時間: ${response_time}s"
        
        # 位置更新のベンチマーク
        log_info "📍 位置更新テスト中..."
        
        # 位置トピックの更新頻度を確認
        position_updates=$(docker-compose exec -T manual_control bash -c "
            source /opt/ros/humble/setup.bash && 
            source /workspace/install/setup.bash && 
            timeout 10 ros2 topic hz /drone/pose" 2>/dev/null | grep "average rate" | tail -1)
        
        if [ ! -z "$position_updates" ]; then
            log_success "位置更新頻度: $position_updates"
        else
            log_warning "位置更新頻度の測定に失敗しました"
        fi
    fi

    echo ""
    log_success "🎉 ドローン性能最適化が完了しました！"
    echo ""
    
    # 最適化結果の表示
    echo "📋 適用された最適化:"
    if [ "$USE_OPTIMIZED_SIM" = true ]; then
        echo "   🔧 最適化されたシミュレーター (200Hz, 高速物理計算)"
    fi
    if [ "$USE_OPTIMIZED_CONFIG" = true ]; then
        echo "   ⚙️  最適化された設定 (高性能PID, 軽量化)"
    fi
    if [ "$RUN_PERFORMANCE_TEST" = true ]; then
        echo "   📊 パフォーマンステスト実行済み"
    fi
    if [ "$RUN_BENCHMARK" = true ]; then
        echo "   🏃 ベンチマークテスト実行済み"
    fi
    
    echo ""
    echo "🚀 最適化されたシステムを起動するには:"
    echo "   ./scripts/quick_start.sh"
    echo ""
    echo "📊 パフォーマンスを確認するには:"
    echo "   docker-compose logs -f manual_control"
}

# エラーハンドリング
trap 'log_error "スクリプトがエラーで終了しました"; exit 1' ERR

# メイン処理実行
main "$@" 