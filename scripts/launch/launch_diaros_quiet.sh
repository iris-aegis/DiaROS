#!/bin/bash
# launch_diaros_quiet.sh - rcutilsエラーを抑制した起動スクリプト
# 
# このスクリプトは、rcutilsのtruncatedエラーメッセージを抑制して
# DiaROSを起動します。

# スクリプトのディレクトリを取得
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SCRIPT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "=== DiaROS quiet launcher ==="
echo "🔇 rcutilsエラーメッセージを抑制してDiaROSを起動します"
echo ""

# 既存の起動スクリプトに環境変数を渡して実行
export RCUTILS_LOGGING_SUPPRESSED_FILE="/Users/sayonari/_data/_DiaROS_mac/DiaROS_pixi/DiaROS_imamoto/DiaROS_ros/src"
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {message}'
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_COLORIZED_OUTPUT=0
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

# デフォルトでローカルLLMを使用（APIキーがある場合のみChatGPT使用）
if [ -z "$OPENAI_API_KEY" ] && [ -z "$DIAROS_LLM_MODEL" ]; then
    echo "💡 APIキー未設定のため、ローカルLLMモードで起動します"
    export DIAROS_LLM_MODEL=rinna-small
fi

# 通常の起動スクリプトを実行
exec "$SCRIPT_DIR/launch_diaros.sh"