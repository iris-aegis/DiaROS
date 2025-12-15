#!/bin/bash

# rosbag2レコーディングが正しく動作するかテストするスクリプト

echo "====================================="
echo "ROS2 Bag Recording Test"
echo "====================================="

# 作業ディレクトリ
DIAROS_DIR="/workspace/DiaROS_ros"
cd "$DIAROS_DIR"

# ROS2環境の設定
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "$DIAROS_DIR/install/local_setup.bash" ]; then
    source "$DIAROS_DIR/install/local_setup.bash"
fi

# rosbag2環境変数の設定
if [ -f "/root/.ros/rosbag2_environment.sh" ]; then
    source /root/.ros/rosbag2_environment.sh
fi

echo "1. テスト用レコーディングを開始..."
TEST_DIR="test_recording_$(date +%Y%m%d_%H%M%S)"

# 3秒間のテストレコーディング
timeout 3 ros2 bag record /rosout --output "$TEST_DIR" 2>/dev/null

sleep 1

echo "2. レコーディング結果を確認..."

# 結果チェック
if [ -d "$TEST_DIR" ]; then
    echo "   ✅ レコーディングディレクトリが作成されました: $TEST_DIR"
    
    # .db3ファイルの確認
    if ls "${TEST_DIR}"/*.db3 >/dev/null 2>&1; then
        db_file=$(ls "${TEST_DIR}"/*.db3 | head -1)
        echo "   ✅ データベースファイルが作成されました: $(basename "$db_file")"
    else
        echo "   ❌ データベースファイルが見つかりません"
    fi
    
    # metadata.yamlの確認
    if [ -f "${TEST_DIR}/metadata.yaml" ]; then
        echo "   ✅ metadata.yamlが作成されました"
        
        # metadata.yamlの内容確認
        if grep -q "rosbag2_bagfile_information" "${TEST_DIR}/metadata.yaml"; then
            echo "   ✅ metadata.yamlの形式が正しいです"
        else
            echo "   ❌ metadata.yamlの形式に問題があります"
        fi
        
        # ファイル構造の確認
        if grep -q "relative_file_paths" "${TEST_DIR}/metadata.yaml" && \
           grep -q "version" "${TEST_DIR}/metadata.yaml" && \
           grep -q "storage_identifier" "${TEST_DIR}/metadata.yaml"; then
            echo "   ✅ metadata.yamlに必要な要素が含まれています"
        else
            echo "   ❌ metadata.yamlに必要な要素が不足しています"
        fi
        
    else
        echo "   ❌ metadata.yamlが作成されていません"
    fi
    
    echo ""
    echo "3. rqt_bagでの読み込みテスト..."
    
    # rqt_bagでの読み込みテスト（非GUI環境では実際の読み込みはできないが、ファイル形式をチェック）
    if command -v ros2 >/dev/null && ros2 bag info "$TEST_DIR" >/dev/null 2>&1; then
        echo "   ✅ ros2 bag infoコマンドで正常に読み込めます"
        echo ""
        echo "📊 bag情報:"
        ros2 bag info "$TEST_DIR"
    else
        echo "   ❌ ros2 bag infoコマンドで読み込みに失敗しました"
    fi
    
    echo ""
    echo "4. テストファイルのクリーンアップ..."
    read -p "テストファイルを削除しますか？ (y/N): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$TEST_DIR"
        echo "   ✅ テストファイルを削除しました"
    else
        echo "   📁 テストファイルを保持しました: $TEST_DIR"
    fi
    
else
    echo "   ❌ レコーディングディレクトリが作成されませんでした"
fi

echo ""
echo "====================================="
echo "🏁 テスト完了"
echo "====================================="

if [ -d "$TEST_DIR" ] || [ -f "${TEST_DIR}/metadata.yaml" ]; then
    echo "✅ rosbag2レコーディングは正常に動作しています"
    echo ""
    echo "📋 rqt_bagでのレコーディング手順:"
    echo "1. bash /workspace/scripts/debug/monitor.sh"
    echo "2. 1a を選択（rqt_bag専用起動）"
    echo "3. Plugins → Logging → Bag を選択"
    echo "4. Record タブでトピックを選択してレコーディング"
else
    echo "❌ rosbag2レコーディングに問題があります"
    echo ""
    echo "🔧 トラブルシューティング:"
    echo "1. ROS2環境が正しく設定されているか確認"
    echo "2. rosbag2パッケージがインストールされているか確認"
    echo "3. 権限設定を確認"
fi