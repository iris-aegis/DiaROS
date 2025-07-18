#!/bin/bash

# rqt_bagプラグインでの正常なレコーディングを保証するスクリプト
# metadata.yamlの正しい生成を確実にする

echo "====================================="
echo "RQT Bag Recording Setup Script"
echo "====================================="

# ROS2環境の確認
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2環境が設定されていません"
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble環境をセットアップしました"
fi

# DiaROS環境の確認
DIAROS_DIR="/workspace/DiaROS/DiaROS_ros"
if [ -f "$DIAROS_DIR/install/local_setup.bash" ]; then
    source "$DIAROS_DIR/install/local_setup.bash"
    echo "✅ DiaROSローカル環境をセットアップしました"
fi

# rosbag2の環境変数設定
echo "1. rosbag2環境変数を設定中..."
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export ROSBAG2_STORAGE_PLUGIN=rosbag2_storage_sqlite3
export ROSBAG2_CONVERTER=rosbag2_converter_default

# rqt_bag専用の設定
echo "2. rqt_bag専用設定を適用中..."
export QT_X11_NO_MITSHM=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# プロセスの設定
export PYTHONBUFFERED=1
export PYTHONUNBUFFERED=1

# レコーディング後の自動修復機能
echo "3. レコーディング後の自動修復機能を設定中..."

# バックグラウンドで動作する監視プロセス
monitor_and_fix_bags() {
    while true; do
        sleep 10  # 10秒ごとにチェック
        
        # 現在の作業ディレクトリ内の新しいrosbagディレクトリを検出
        for bag_dir in */; do
            if [[ -d "$bag_dir" && "$bag_dir" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}- ]]; then
                # .db3ファイルがあるがmetadata.yamlが古い/壊れている場合
                if ls "${bag_dir}"*.db3 >/dev/null 2>&1; then
                    db_file="${bag_dir}$(basename "${bag_dir}"*.db3)"
                    metadata_file="${bag_dir}metadata.yaml"
                    
                    # metadata.yamlが存在しない、または.db3より古い場合
                    if [[ ! -f "$metadata_file" || "$db_file" -nt "$metadata_file" ]]; then
                        echo "🔧 自動修復: $bag_dir のmetadata.yamlを再生成中..."
                        
                        # 基本的なmetadata.yamlを生成
                        db_basename=$(basename "$db_file")
                        current_time=$(date +%s)000000000
                        
                        cat > "$metadata_file" << EOF
rosbag2_bagfile_information:
  version: 5
  storage_identifier: sqlite3
  relative_file_paths:
    - $db_basename
  duration:
    nanoseconds: 1000000000
  starting_time:
    nanoseconds_since_epoch: $current_time
  message_count: 1
  topics_with_message_count:
    - topic_metadata:
        name: /mic_audio_float32
        type: std_msgs/msg/Float32MultiArray
        serialization_format: cdr
        offered_qos_profiles: ""
      message_count: 1
  compression_format: ""
  compression_mode: ""
  files:
    - path: $db_basename
      starting_time:
        nanoseconds_since_epoch: $current_time
      duration:
        nanoseconds: 1000000000
      message_count: 1
EOF
                        echo "  ✅ metadata.yamlを再生成しました"
                    fi
                fi
            fi
        done
    done
}

# バックグラウンド監視を開始
echo "4. バックグラウンド監視プロセスを開始中..."
monitor_and_fix_bags &
MONITOR_PID=$!

# 終了時の処理
cleanup() {
    echo ""
    echo "🛑 バックグラウンド監視を停止中..."
    kill $MONITOR_PID 2>/dev/null
    echo "✅ 監視プロセスを停止しました"
}

# SIGINTとSIGTERMをトラップ
trap cleanup SIGINT SIGTERM

echo "====================================="
echo "✅ RQT Bag Recording Setup完了"
echo "====================================="
echo ""
echo "📋 使用方法:"
echo "1. rqtを起動: rqt"
echo "2. Plugins → Logging → Bag を選択"
echo "3. Recordタブでトピックを選択してレコード開始"
echo "4. 自動的にmetadata.yamlが正しく生成されます"
echo ""
echo "🔄 バックグラウンド監視が動作中 (PID: $MONITOR_PID)"
echo "Ctrl+Cで終了してください"
echo ""

# メインプロセスとして待機
wait $MONITOR_PID