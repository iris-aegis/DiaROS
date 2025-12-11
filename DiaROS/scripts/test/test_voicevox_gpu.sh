#!/bin/bash
# VOICEVOX GPU動作テストスクリプト

echo "🧪 VOICEVOX GPU動作テスト"
echo "========================"

# 音声合成テスト
echo "音声合成テスト実行中..."
for i in {1..3}; do
    echo "テスト $i/3: 処理中..."

    curl -X POST "localhost:50021/synthesis" \
        -H "Content-Type: application/json" \
        -d "{\"text\":\"テスト番号${i}です。音声合成動作確認中。\",\"speaker\":1}" \
        --output "/tmp/test_${i}.wav" \
        --silent

    if [ $? -eq 0 ]; then
        echo "✅ テスト $i 成功"
    else
        echo "❌ テスト $i 失敗"
    fi

    sleep 1
done

echo ""
echo "✅ テスト完了"
