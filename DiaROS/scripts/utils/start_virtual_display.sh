#!/bin/bash
# 仮想ディスプレイ起動スクリプト

echo "🖥️  仮想ディスプレイ起動"
echo "===================="

# Xvfbプロセス確認
if pgrep Xvfb >/dev/null; then
    echo "✅ Xvfbが既に起動しています"
    ps aux | grep Xvfb | grep -v grep
else
    echo "📺 仮想ディスプレイを起動中..."
    # Xvfb起動（バックグラウンド）
    Xvfb :99 -ac -screen 0 1280x1024x16 &
    XVFB_PID=$!
    sleep 2
    
    if pgrep Xvfb >/dev/null; then
        echo "✅ 仮想ディスプレイ起動成功（PID: $XVFB_PID）"
        export DISPLAY=:99
        echo "DISPLAY=:99 に設定しました"
    else
        echo "❌ 仮想ディスプレイ起動失敗"
    fi
fi
