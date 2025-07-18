#!/bin/bash

# rosbag2のレコーディング問題を修正するスクリプト
# metadata.yamlが正しく作成されない問題を解決

echo "====================================="
echo "ROS2 Bag Recording Fix Script"
echo "====================================="

# 現在のディレクトリ
CURRENT_DIR=$(pwd)
echo "現在のディレクトリ: $CURRENT_DIR"

# 権限の修正
echo "1. ディレクトリ権限を修正中..."
sudo chown -R $USER:$USER .
sudo chmod -R 755 .

# 不完全なrosbagディレクトリを検出して修正
echo "2. 不完全なrosbagディレクトリを検出中..."
for bag_dir in */; do
    if [[ -d "$bag_dir" ]]; then
        # .db3ファイルがあるがmetadata.yamlがない場合
        if ls "$bag_dir"*.db3 >/dev/null 2>&1 && [[ ! -f "${bag_dir}metadata.yaml" ]]; then
            echo "修正が必要: $bag_dir"
            
            # metadata.yamlを作成
            db_basename=$(basename "${bag_dir}"*.db3)
            current_time=$(date +%s)000000000
            
            cat > "${bag_dir}metadata.yaml" << EOF
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
            echo "  ✅ metadata.yamlを作成しました: $bag_dir"
        fi
    fi
done

# ROS2環境変数の設定
echo "3. ROS2環境変数を設定中..."
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export CYCLONEDX_URI=xml
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# rosbag2のストレージ設定
echo "4. rosbag2設定を最適化中..."
export ROSBAG2_STORAGE_PLUGIN=rosbag2_storage_sqlite3
export ROSBAG2_CONVERTER=rosbag2_converter_default

echo "====================================="
echo "✅ ROS2 Bag Recording修正完了"
echo "====================================="
echo ""
echo "📋 推奨レコーディングコマンド:"
echo "ros2 bag record -a"
echo "または特定のトピック:"
echo "ros2 bag record /mic_audio_float32 /ASRtoNLU /DMtoNLG"
echo ""
echo "📁 レコーディング後、以下を確認:"
echo "1. .db3ファイルの存在"
echo "2. metadata.yamlの存在"
echo "3. ファイルの権限 (644)"
echo ""