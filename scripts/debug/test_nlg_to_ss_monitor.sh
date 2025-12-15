#!/bin/bash
# NLGtoSSトピック監視スクリプト（環境セットアップ付き）

set -e

echo "=========================================="
echo "🔍 NLGtoSS通信監視（音声認識履歴送受信テスト）"
echo "=========================================="

# ROS2環境セットアップ
echo "🔧 ROS2環境をセットアップ中..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble環境をロード"
else
    echo "❌ ROS2 Humble環境が見つかりません"
    exit 1
fi

# DiaROSローカル環境セットアップ
DIAROS_DIR="/workspace/DiaROS_ros"
if [ -f "$DIAROS_DIR/install/local_setup.bash" ]; then
    source "$DIAROS_DIR/install/local_setup.bash"
    echo "✅ DiaROSローカル環境をロード"
else
    echo "❌ DiaROSビルドが見つかりません: $DIAROS_DIR/install/local_setup.bash"
    exit 1
fi

echo ""
echo "📡 NLGtoSSトピック監視開始..."
echo "=========================================="
echo "確認項目:"
echo "  - 対話生成結果（reply）"
echo "  - 音声認識結果リスト（source_words）の受信"
echo "  - 履歴数とデータ内容"
echo ""
echo "Ctrl+C で終了"
echo "=========================================="

# NLGtoSSトピックの存在確認
if ! ros2 topic list | grep -q "^/NLGtoSS$"; then
    echo "⚠️  /NLGtoSSトピックが見つかりません"
    echo "DiaROSが起動していることを確認してください"
    exit 1
fi

# Python監視スクリプト実行
python3 /workspace/scripts/debug/test_nlg_to_ss.py

echo ""
echo "🏁 NLGtoSS監視完了"