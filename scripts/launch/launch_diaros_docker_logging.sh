#!/bin/bash
# Docker環境用 DiaROS起動スクリプト（ログファイル保存付き）

# ログディレクトリ設定
LOG_DIR="/workspace/log/console_logs"
mkdir -p "$LOG_DIR"

# タイムスタンプ付きログファイル名
TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
LOG_FILE="$LOG_DIR/diaros_${TIMESTAMP}.log"

echo "=== DiaROS Docker Launcher with Logging ==="
echo "Logs will be saved to: $LOG_FILE"

# 環境設定
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
fi

if [ -f "/workspace/DiaROS_ros/install/local_setup.bash" ]; then
    source /workspace/DiaROS_ros/install/local_setup.bash
else
    echo "Warning: /workspace/DiaROS_ros/install/local_setup.bash not found."
    echo "Make sure to build the workspace first."
fi

# 実行コマンド（teeで標準出力・標準エラー出力をログファイルにも保存）
# PYTHONUNBUFFERED=1を設定してPythonの出力バッファリングを無効化
export PYTHONUNBUFFERED=1
# カラー出力を強制（ログファイルにも色コードが入るが、less -R等で見やすい）
export RCUTILS_COLORIZED_OUTPUT=1 

echo "Starting ros2 launch (DMPC mode: nlg:=false)..."
# DMPCとして起動するため nlg:=false を指定
# 2>&1 で標準エラー出力も標準出力にマージして tee に渡す
ros2 launch diaros_package sdsmod.launch.py nlg:=false 2>&1 | tee "$LOG_FILE"
