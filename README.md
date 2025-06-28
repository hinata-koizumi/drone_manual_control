# ドローン手動制御環境

> **English**: [README.en.md](README.en.md)

この環境は、ROS 2 Humbleベースのドローン手動制御システムです。Webブラウザからリアルタイムでドローンを制御し、3D可視化でドローンの状態を監視できます。

## 🚀 クイックスタート

### 完全自動デプロイ（推奨）
```bash
# ワンクリックでシステムを起動（ブラウザも自動で開きます）
./scripts/quick_start.sh
```

### 段階的デプロイ
```bash
# 1. すべてのパッケージをビルド
./scripts/build_all.sh

# 2. システムを起動
./scripts/start_system.sh
```

### 完全自動化（オプション付き）
```bash
# 完全自動デプロイ（ビルド + 起動 + ヘルスチェック）
./scripts/auto_deploy.sh

# オプション付きデプロイ
./scripts/auto_deploy.sh --clean        # クリーンな状態から開始
./scripts/auto_deploy.sh --build-only   # ビルドのみ実行
./scripts/auto_deploy.sh --start-only   # 既存ビルドで起動のみ
./scripts/auto_deploy.sh --no-web-viz   # Web可視化を無効にして起動
```

## 🎮 動作確認済み機能

### ドローン制御コマンド
✅ **Take Off**: ドローンが上昇（Z軸正方向）  
✅ **Land**: ドローンが下降（Z軸負方向）  
✅ **Forward**: ドローンが前進（X軸正方向）  
✅ **Backward**: ドローンが後退（X軸負方向）  
✅ **Left**: ドローンが左移動（Y軸正方向）  
✅ **Right**: ドローンが右移動（Y軸負方向）  
✅ **Up**: ドローンが上昇（Z軸正方向、中程度）  
✅ **Down**: ドローンが下降（Z軸負方向、中程度）  
✅ **Stop**: すべての移動を停止  
✅ **Hover**: ホバリング状態の維持  

### リアルタイム表示
✅ **位置情報**: X, Y, Z座標のリアルタイム更新  
✅ **速度情報**: X, Y, Z軸の速度のリアルタイム更新  
✅ **WebSocket通信**: 安定したリアルタイム通信  
✅ **3D可視化**: ブラウザベースのドローン位置表示  

## 専用ドローン機体情報

### Aether-SL

手動制御環境専用に設計された高性能ドローン機体です。

#### **基本仕様**
- **機体タイプ**: ストリームライン型クアッドコプター
- **重量**: 0.65kg
- **サイズ**: 32.5cm × 32.5cm × 7cm
- **最大速度**: 23.6 m/s（85 km/h）
- **最大高度**: 50m
- **飛行時間**: 約25分（シミュレーション）
- **最大航続距離**: 20km

#### **推進システム**
- **メインローター**: 4基（6インチプロペラ、最大11000rpm）
- **制御チャンネル**: 4チャンネル（4ローター）
- **推力定数**: 1.6（高効率設計）

#### **安全機能**
- **ジオフェンス**: 水平20km、垂直50m制限
- **フェイルセーフ**: RC信号喪失時の自動帰還
- **速度制限**: 水平23.6m/s、垂直8m/s
- **加速度制限**: 最大12m/s²

#### **制御パラメータ**
- **姿勢制御**: PID制御（Roll/Pitch: P=4.8, I=0.18, D=0.09）
- **位置制御**: PID制御（XY: P=1.0, I=0.1, D=0.05）
- **手動制御**: 傾斜角制限80%、ヨー角制限80%

#### **フライトモード**
- **STABILIZE**: 姿勢安定化モード
- **ALT_HOLD**: 高度保持モード
- **LOITER**: 位置保持モード
- **AUTO**: 自動飛行モード
- **RTL**: 自動帰還モード
- **MANUAL**: 手動制御モード

詳細仕様は `config/drone_specs.yaml` を参照してください。

## システム構成

### コンテナ構成
- **drone_msgs**: カスタムメッセージ定義
- **bridge**: ROS 2通信ブリッジ
- **manual_control**: ドローンシミュレーターと制御システム
- **web_viz**: Web可視化と制御インターフェース

### 技術スタック
- **ROS 2 Humble**: 基盤となる通信システム
- **Docker Compose**: マルチコンテナ環境
- **WebSocket**: リアルタイムなWeb UI通信
- **Python**: シミュレーションと制御ロジック
- **JavaScript**: Web UIフロントエンド

## 特徴

- **リアルタイム制御**: Webブラウザから即座にドローンを制御
- **物理シミュレーション**: 重力、推力、制御を考慮したリアルisticな動作
- **ROS 2 Humble対応**: 最新のROS 2フレームワークを使用
- **Docker統合**: 再現可能な開発環境
- **モジュラー設計**: 既存のブリッジコンポーネントを再利用
- **Web可視化**: ブラウザベースの3D可視化と手動制御
- **完全自動化**: ワンクリックでシステム構築から起動まで
- **マルチプロセス対応**: 安定したWebSocket通信とROS 2統合

## 移行されたコンポーネント

### 基盤コンポーネント
- `common/` - BridgeBaseクラスとユーティリティ
- `drone_msgs/` - カスタムメッセージ定義
- `px4_msgs/` - PX4メッセージ定義

### ブリッジノード
- `command_bridge/` - 制御コマンド変換
- `state_bridge/` - 状態情報変換

## 新規追加コンポーネント

### 手動制御システム
- `manual_control/` - ドローンシミュレーターと制御システム
  - `simple_simulator.py`: 物理シミュレーション
  - `optimized_simulator.py`: 最適化されたシミュレーション
  - `action_executor.py`: アクション実行
  - `state_monitor.py`: 状態監視

### Web可視化システム
- `web_viz/` - ブラウザベースの3D可視化と制御インターフェース
  - `server.py`: WebSocketサーバーとROS 2統合
  - `index.html`: Web UIフロントエンド

## セットアップ手順

### 1. 環境の初期化
```bash
# 既存環境からコンポーネントをコピー
./scripts/setup_environment.sh
```

### 2. 環境のビルド
```bash
# Dockerコンテナをビルド
docker-compose build
```

### 3. デモの実行
```bash
# 自動デモ実行
./scripts/run_demo.sh
```

### 4. 手動実行（オプション）
```bash
# ブリッジノード起動
docker-compose up -d bridge

# 手動制御ノード起動
docker-compose up -d manual_control
```

## 使用方法

### 基本的な使用方法
1. **環境構築**
```bash
cd drone_manual_control
./scripts/setup_environment.sh
docker-compose build
```

2. **システム起動**
```bash
docker-compose up -d
```

3. **Web UIアクセス**
```bash
# ブラウザで以下にアクセス
http://localhost:8080
```

### Web可視化の使用
1. **システム起動後、ブラウザでアクセス**
```
http://localhost:8080
```

2. **利用可能な機能**
- リアルタイム3Dドローン可視化
- 手動制御ボタン（上昇、下降、前進、後退、左移動、右移動）
- ドローン状態のリアルタイム表示（位置、速度）
- WebSocket接続状態の表示

3. **制御ボタン**
- **Take Off**: ドローンを上昇させる
- **Land**: ドローンを下降させる
- **Forward/Backward**: 前進/後退
- **Left/Right**: 左移動/右移動
- **Up/Down**: 上昇/下降（中程度）
- **Stop**: すべての移動を停止
- **Hover**: ホバリング状態を維持

### ログの確認
```bash
# 手動制御ノードのログ
docker-compose logs -f manual_control

# Web可視化のログ
docker-compose logs -f web_viz

# 全ノードのログ
docker-compose logs -f
```

### 環境の停止
```bash
docker-compose down
```

## 技術的な詳細

### 修正済みの問題
1. **推力計算の修正**: `abs()`関数を削除して、landingコマンドの負のスロットル値を正しく処理
2. **QoS設定の統一**: web_vizとシミュレーター間の通信設定を一致
3. **データ共有の改善**: マルチプロセス間のデータ転送を最適化
4. **水平移動の対応**: `linear.x`と`linear.y`コマンドを正しく処理
5. **リアルタイム更新**: UIの位置と速度データがリアルタイムで更新

### 通信フロー
1. **Web UI** → **WebSocket** → **ROS 2 Control Node** → **TwistStamped** → **Simulator**
2. **Simulator** → **PoseStamped/TwistStamped** → **ROS 2 Control Node** → **WebSocket** → **Web UI**

### 物理シミュレーション
- **重力**: 9.81 m/s²
- **推力**: 可変推力（制御コマンドに応じて）
- **質量**: 0.65 kg
- **制御**: PID制御による姿勢制御
- **衝突検出**: 地面との衝突処理

## 自動化スクリプト

### `scripts/auto_deploy.sh` - 完全自動デプロイ
- ビルドから起動、ヘルスチェックまで完全自動化
- オプション付きで柔軟な運用が可能

### `scripts/quick_start.sh` - クイックスタート
- ワンクリックでシステム起動
- ブラウザも自動で開く

### `scripts/build_all.sh` - 段階的ビルド
- 各パッケージを順次ビルド
- 詳細なログ出力

### `scripts/start_system.sh` - システム起動
- 既存ビルドを使用してシステム起動

## トラブルシューティング

### よくある問題
1. **WebSocket接続エラー**: ポート8080が使用中の場合、他のプロセスを停止
2. **ROS 2通信エラー**: コンテナ間のネットワーク設定を確認
3. **UIが更新されない**: ブラウザのキャッシュをクリア

### デバッグ方法
```bash
# リアルタイムログ確認
docker-compose logs -f

# 特定コンテナのログ確認
docker-compose logs -f manual_control
docker-compose logs -f web_viz

# コンテナ内でのROS 2トピック確認
docker-compose exec manual_control bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
```

## ライセンス

このプロジェクトはMITライセンスの下で公開されています。

# Manual control CI trigger
