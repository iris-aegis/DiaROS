#!/bin/bash
# Docker内でPulseAudio経由の音声設定を行うスクリプト

echo "🎤 Docker内音声設定"
echo "=================="

# PulseAudioサーバーへの接続確認
echo "1. PulseAudioサーバーへの接続を確認中..."
if pactl info > /dev/null 2>&1; then
    echo "✅ PulseAudioサーバーに接続されています"
    pactl info | grep -E "Server Name|Server Version|User Name"
else
    echo "❌ PulseAudioサーバーに接続できません"
    echo "ホスト側でPulseAudioが起動していることを確認してください"
    exit 1
fi

# 利用可能なソース（入力デバイス）を表示
echo ""
echo "2. 利用可能な入力デバイス:"
echo "------------------------"
pactl list sources short | grep -v monitor | while read -r line; do
    id=$(echo "$line" | awk '{print $1}')
    name=$(echo "$line" | awk '{print $2}')
    echo "  ID: $id - $name"
done

# PyAudio用の設定
echo ""
echo "3. PyAudio環境設定..."

# ALSAプラグインの確認
if [ ! -f /usr/lib/*/alsa-lib/libasound_module_pcm_pulse.so ]; then
    echo "⚠️  ALSAのPulseAudioプラグインが見つかりません"
    echo "   apt-get install libasound2-plugins でインストールしてください"
fi

# PulseAudioをデフォルトのALSAデバイスとして設定
cat > ~/.asoundrc << 'EOF'
pcm.!default {
    type pulse
}
ctl.!default {
    type pulse
}
EOF
echo "✅ ALSAをPulseAudioにリダイレクトする設定を作成しました"

# システム全体の設定も更新
if [ -w /etc/asound.conf ]; then
    cat > /etc/asound.conf << 'EOF'
pcm.!default {
    type pulse
}
ctl.!default {
    type pulse
}
EOF
    echo "✅ システム全体のALSA設定も更新しました"
fi

# 環境変数の設定
export PULSE_LATENCY_MSEC=30
echo "✅ PULSE_LATENCY_MSEC=30 を設定しました"

# PyAudioのテスト
echo ""
echo "4. PyAudioテスト..."
python3 -c "
import pyaudio
import sys

try:
    p = pyaudio.PyAudio()
    device_count = p.get_device_count()
    print(f'✅ PyAudioが正常に初期化されました')
    print(f'   検出されたデバイス数: {device_count}')
    
    # 入力デバイスを表示
    print('\\n   入力デバイス:')
    for i in range(device_count):
        info = p.get_device_info_by_index(i)
        if info['maxInputChannels'] > 0:
            print(f'     [{i}] {info[\"name\"]} (ch: {info[\"maxInputChannels\"]})')
    
    p.terminate()
except Exception as e:
    print(f'❌ PyAudioエラー: {e}')
    sys.exit(1)
"

echo ""
echo "5. 設定完了"
echo "==========="
echo ""
echo "次のステップ:"
echo "1. python3 /workspace/scripts/test_audio_simple.py でテスト"
echo "2. python3 /workspace/scripts/set_default_mic.py でデバイス設定"
echo ""
echo "トラブルシューティング:"
echo "- 音声デバイスが見つからない場合は、ホスト側でPulseAudioが起動しているか確認"
echo "- macOS: brew services list で pulseaudio が started になっているか確認"