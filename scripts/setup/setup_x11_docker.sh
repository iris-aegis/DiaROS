#!/bin/bash
# Docker環境でのX11/GUI設定スクリプト

echo "🖥️  Docker X11/GUI 設定"
echo "======================="

# 問題診断
echo "1. 現在の状況確認..."
echo "DISPLAY環境変数: $DISPLAY"
echo "QT_QPA_PLATFORM: $QT_QPA_PLATFORM"

# X11ソケット確認
if [ -d "/tmp/.X11-unix" ]; then
    echo "✅ X11ソケットディレクトリが存在します"
    ls -la /tmp/.X11-unix/
else
    echo "❌ X11ソケットディレクトリが存在しません"
fi

# Docker X11設定の修正
echo ""
echo "2. X11設定修正..."

# DISPLAY環境変数の設定
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
    echo "✅ DISPLAY=:0 を設定しました"
else
    echo "現在のDISPLAY: $DISPLAY"
fi

# Qtプラットフォーム設定（GUI無効化対応）
export QT_QPA_PLATFORM=offscreen
echo "✅ QT_QPA_PLATFORM=offscreen を設定（GUI無効化）"

# 代替案：仮想ディスプレイ設定
echo ""
echo "3. 仮想ディスプレイ設定（オプション）..."

# Xvfbの確認・インストール
if ! command -v Xvfb >/dev/null 2>&1; then
    echo "📦 Xvfbをインストール中..."
    apt-get update -qq
    apt-get install -y xvfb x11-utils
else
    echo "✅ Xvfbが利用可能です"
fi

# 仮想ディスプレイ起動スクリプト作成
cat > /workspace/DiaROS/scripts/utils/start_virtual_display.sh << 'EOF'
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
EOF

chmod +x /workspace/DiaROS/scripts/utils/start_virtual_display.sh
echo "✅ 仮想ディスプレイ起動スクリプト作成: /workspace/DiaROS/scripts/utils/start_virtual_display.sh"

# RQT用設定スクリプト作成
cat > /workspace/DiaROS/scripts/launch/launch_rqt_docker.sh << 'EOF'
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
EOF

chmod +x /workspace/DiaROS/scripts/launch/launch_rqt_docker.sh
echo "✅ Docker用RQT起動スクリプト作成: /workspace/DiaROS/scripts/launch/launch_rqt_docker.sh"

echo ""
echo "4. 環境変数の永続化..."

# bashrcに追加
if ! grep -q "QT_QPA_PLATFORM=offscreen" ~/.bashrc 2>/dev/null; then
    echo "export QT_QPA_PLATFORM=offscreen" >> ~/.bashrc
    echo "✅ QT_QPA_PLATFORM設定を~/.bashrcに追加しました"
fi

echo ""
echo "5. 設定完了"
echo "============"
echo ""
echo "📋 次のステップ:"
echo "1. Docker用RQT起動: /workspace/DiaROS/scripts/launch/launch_rqt_docker.sh"
echo "2. 仮想ディスプレイ起動: /workspace/DiaROS/scripts/utils/start_virtual_display.sh"
echo "3. 通常RQT起動: QT_QPA_PLATFORM=offscreen rqt"
echo ""
echo "⚠️  注意事項:"
echo "- Docker環境ではGUIが表示されませんが、RQTは動作します"
echo "- 設定ファイルの読み書きは正常に動作します"
echo "- 必要に応じてホスト側でRQTを起動してください"