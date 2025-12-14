#!/bin/bash

# DM→NLGトピック通信テストスクリプト

echo "🔍 DM→NLGトピック通信テスト"
echo "=" * 50

# ROS2環境設定
cd /workspace/DiaROS/DiaROS_ros
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash

echo "1. 現在のトピック一覧確認"
echo "利用可能なトピック:"
ros2 topic list | grep -E "(NLU|DM|NLG)" || echo "  DM/NLG関連トピックが見つかりません"

echo ""
echo "2. トピック型確認"
echo "NLUtoDM トピック型:"
ros2 topic info /NLUtoDM || echo "  NLUtoDMトピックが存在しません"

echo "DMtoNLG トピック型:"
ros2 topic info /DMtoNLG || echo "  DMtoNLGトピックが存在しません"

echo ""
echo "3. 現在の通信状況確認（5秒間監視）"
echo "NLUtoDM (ASR→DM) の監視:"
timeout 5 ros2 topic echo /NLUtoDM --once 2>/dev/null || echo "  5秒間でメッセージを受信できませんでした"

echo ""
echo "DMtoNLG (DM→NLG) の監視:"
timeout 5 ros2 topic echo /DMtoNLG --once 2>/dev/null || echo "  5秒間でメッセージを受信できませんでした"

echo ""
echo "4. 手動テストメッセージ送信"
echo "テスト用ASRメッセージを送信します..."

# テスト用ASRメッセージ送信
ros2 topic pub /NLUtoDM interfaces/msg/Iasr "{you: 'こんにちは', is_final: true}" --once

echo "テストメッセージを送信しました。"
echo ""
echo "5. DM応答確認（10秒間監視）"
echo "DMからの応答を監視中..."

# DM応答を監視
timeout 10 ros2 topic echo /DMtoNLG --once 2>/dev/null && echo "✅ DM応答を受信しました" || echo "❌ DM応答を受信できませんでした"

echo ""
echo "6. ノード状態確認"
echo "起動中のノード:"
ros2 node list | grep -E "(dialog|management)" || echo "  DMノードが見つかりません"

echo ""
echo "7. 追加テスト - 連続送信"
echo "複数のテストメッセージを送信します..."

for i in {1..3}; do
    echo "  テスト $i/3: メッセージ送信中..."
    ros2 topic pub /NLUtoDM interfaces/msg/Iasr "{you: 'テストメッセージ$i', is_final: true}" --once
    sleep 1
    
    # DM応答確認
    echo "  DM応答確認中..."
    timeout 3 ros2 topic echo /DMtoNLG --once 2>/dev/null && echo "    ✅ DM応答受信" || echo "    ❌ DM応答なし"
    echo ""
done

echo "=" * 50
echo "テスト完了"
echo ""
echo "📋 結果の解釈:"
echo "- NLUtoDMにメッセージが表示される = ASR→DM通信は正常"
echo "- DMtoNLGにメッセージが表示される = DM→NLG通信は正常"
echo "- DMtoNLGにメッセージが表示されない = DM内部で条件が満たされていない"
echo ""
echo "🔧 対処法:"
echo "- DMデバッグログを確認してresponse_updateの状態をチェック"
echo "- 音声認識結果が変化しているかを確認"
echo "- DM内部の条件判定ロジックを確認"