#!/bin/bash
# Docker環境用RQT起動スクリプト

echo "🔧 RQT Docker起動"
echo "=================="

# GUI無効化モード（推奨）
echo "1. GUI無効化モードでRQTを起動..."
export QT_QPA_PLATFORM=offscreen
export DISPLAY=:99

echo "設定確認:"
echo "QT_QPA_PLATFORM=$QT_QPA_PLATFORM"
echo "DISPLAY=$DISPLAY"

# 仮想ディスプレイが無い場合は起動
if ! pgrep Xvfb >/dev/null; then
    echo "仮想ディスプレイを起動中..."
    /workspace/DiaROS/scripts/utils/start_virtual_display.sh
fi

echo ""
echo "2. RQT起動試行..."

# ROS2環境設定
source /opt/ros/foxy/setup.bash
source /workspace/DiaROS/DiaROS_ros/install/local_setup.bash

# RQTをバックグラウンドで起動（出力制限）
rqt --force-discover > /tmp/rqt.log 2>&1 &
RQT_PID=$!

sleep 3

if ps -p $RQT_PID > /dev/null; then
    echo "✅ RQT起動成功（PID: $RQT_PID）"
    echo "ログファイル: /tmp/rqt.log"
else
    echo "❌ RQT起動失敗"
    echo "エラーログ:"
    cat /tmp/rqt.log
fi
