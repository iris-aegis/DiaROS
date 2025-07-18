#!/usr/bin/env bash
# =========================================
# DiaROS launcher (WITHOUT speech_input node)
# ros2 bag playでの音声データ再生に対応
# =========================================
set -euo pipefail

echo "=== DiaROS launcher (No Speech Input) ==="

# ---------- 事前設定 ----------
PIXI_DIR="$HOME/_data/_DiaROS_mac/DiaROS_pixi"         # Pixi ルート
PIXI_WS="$PIXI_DIR/diaros_workspace"                   # Pixi 仮想環境 (= poetry venv 相当)
DIAROS_DIR="$PIXI_DIR/DiaROS_imamoto/DiaROS_ros"       # ROS2 ワークスペース

# OS ごとの定義 ------------------------------------
OS=$(uname -s)
case "$OS" in
  Darwin)
    GPU_DEVICE="mps"
    VOICEVOX_APP="$HOME/_data/tools/VOICEVOX/VOICEVOX.app"
    start_voicevox() {
      if curl -fs http://localhost:50021/version >/dev/null 2>&1; then
        echo "✅ VOICEVOX already running"
      else
        if [ -d "$VOICEVOX_APP" ]; then
          echo "▶ Launching VOICEVOX.app ..."
          open -a "$VOICEVOX_APP"
          # 起動待機
          echo "⏳ Waiting for VOICEVOX to start..."
          for i in {1..30}; do
            if curl -fs http://localhost:50021/version >/dev/null 2>&1; then
              echo "✅ VOICEVOX started successfully"
              return 0
            fi
            echo "  Attempt $i/30..."
            sleep 2
          done
          echo "❌ VOICEVOX failed to start within 60 seconds"
          exit 1
        else
          echo "❌ VOICEVOX.app not found at $VOICEVOX_APP"
          exit 1
        fi
      fi
    }
    ;;

  Linux)
    GPU_DEVICE="cuda"   # お好みで `cpu` など
    VOICEVOX_DIR="$HOME/_data/tools/VOICEVOX/linux-x64"
    start_voicevox() {
      if curl -fs http://localhost:50021/version >/dev/null 2>&1; then
        echo "✅ VOICEVOX already running"
      else
        if [ -x "$VOICEVOX_DIR/run" ]; then
          echo "▶ Launching VOICEVOX ..."
          nohup "$VOICEVOX_DIR/run" > /tmp/voicevox.log 2>&1 & 
          VOICEVOX_PID=$!
          echo "VOICEVOX PID: $VOICEVOX_PID"
          
          echo "⏳ Waiting for VOICEVOX to start..."
          for i in {1..30}; do
            if curl -fs http://localhost:50021/version >/dev/null 2>&1; then
              echo "✅ VOICEVOX started successfully"
              return 0
            fi
            echo "  Attempt $i/30..."
            sleep 2
          done
          echo "❌ VOICEVOX failed to start within 60 seconds"
          exit 1
        else
          echo "❌ run binary not found under $VOICEVOX_DIR"
          exit 1
        fi
      fi
    }
    ;;

  *)
    echo "❌ Unsupported OS: $OS"
    exit 1
    ;;
esac

# ---------- ディレクトリ検証 ----------
echo "🔍 Verifying directories..."
for p in "$PIXI_WS" "$DIAROS_DIR"; do
  if [ ! -d "$p" ]; then
    echo "❌ Directory not found: $p"
    exit 1
  else
    echo "✅ Found: $p"
  fi
done

# ---------- VOICEVOX 起動 ----------
echo "🎤 Starting VOICEVOX..."
start_voicevox

# ---------- 一時スクリプト作成 ----------
TEMP_SCRIPT=$(mktemp)
cat > "$TEMP_SCRIPT" << 'SCRIPT_EOF'
#!/bin/bash
set -ex

echo "=== Environment Check ==="
echo "Current directory: $(pwd)"
echo "Python version: $(python3 --version 2>/dev/null || echo 'Python not found')"

# ROS2コマンドチェック
if command -v ros2 >/dev/null 2>&1; then
    echo "✅ ros2 found at: $(which ros2)"
else
    echo "❌ ros2 not found"
    echo "PATH: $PATH"
    exit 1
fi

echo "=== Changing to DiaROS directory ==="
cd "$DIAROS_DIR_VAR"
echo "Now in: $(pwd)"

echo "=== Checking DiaROS installation ==="
if [ ! -d "install/" ]; then
    echo "❌ install/ directory not found"
    exit 1
fi

if [ ! -d "install/diaros_package/" ]; then
    echo "❌ diaros_package not found"
    exit 1
fi

echo "=== Setting up ROS2 environment ==="
export DIAROS_DEVICE="$GPU_DEVICE_VAR"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export AMENT_PREFIX_PATH="$PWD/install/diaros_package:$PWD/install/interfaces:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="$PWD/install/diaros_package/lib/python3.9/site-packages:$PWD/install/interfaces/lib/python3.9/site-packages:${PYTHONPATH:-}"

case "$OS_VAR" in
    Darwin)
        export DYLD_LIBRARY_PATH="$PWD/install/interfaces/lib:${DYLD_LIBRARY_PATH:-}"
        ;;
    Linux)
        export LD_LIBRARY_PATH="$PWD/install/interfaces/lib:${LD_LIBRARY_PATH:-}"
        ;;
esac

echo "Environment variables set:"
echo "  DIAROS_DEVICE=$DIAROS_DEVICE"
echo "  AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"

echo "=== Checking launch file ==="
LAUNCH_FILE=$(find $PWD/install -name 'sdsmod.launch.py' 2>/dev/null | head -1)
if [ -z "$LAUNCH_FILE" ]; then
    echo "❌ sdsmod.launch.py not found"
    echo "Available launch files:"
    find $PWD/install -name '*.launch.py' 2>/dev/null || echo "No launch files found"
    exit 1
else
    echo "✅ Found launch file: $LAUNCH_FILE"
fi

echo "=== Testing ros2 commands ==="
if ! ros2 --help >/dev/null 2>&1; then
    echo "❌ ros2 help failed"
    exit 1
fi

echo "=== Starting ROS2 launch (WITHOUT speech_input node) ==="
echo "📝 speech_inputノードは起動されません (mic:=false)"
echo "🎵 ros2 bag playでの音声データ再生に対応しています"
echo "Command: ros2 launch diaros_package sdsmod.launch.py mic:=false"
exec ros2 launch diaros_package sdsmod.launch.py mic:=false
SCRIPT_EOF

# 変数を置換
sed -i.bak "s|\$DIAROS_DIR_VAR|$DIAROS_DIR|g" "$TEMP_SCRIPT"
sed -i.bak "s|\$GPU_DEVICE_VAR|$GPU_DEVICE|g" "$TEMP_SCRIPT"
sed -i.bak "s|\$OS_VAR|$OS|g" "$TEMP_SCRIPT"

chmod +x "$TEMP_SCRIPT"

# ---------- Pixi 経由で実行 ----------
echo "🤖 Launching DiaROS (No Speech Input) in Pixi environment ..."
cd "$PIXI_WS"

echo "📂 Current directory: $(pwd)"
echo "🚀 Executing temporary script through pixi..."

# Pixi環境でPythonを明示的に指定して実行（環境変数を引き継ぐ）
OPENAI_API_KEY="${OPENAI_API_KEY:-}" ANTHROPIC_API_KEY="${ANTHROPIC_API_KEY:-}" \
PYTHONEXECUTABLE="$PIXI_WS/.pixi/envs/default/bin/python" \
pixi run bash "$TEMP_SCRIPT"

# クリーンアップ
rm -f "$TEMP_SCRIPT" "$TEMP_SCRIPT.bak"