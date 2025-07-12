#!/bin/bash
# launch_diaros_chatgpt.sh - ChatGPT APIモードでDiaROSを起動
# 
# このスクリプトは、ChatGPT APIを使用してDiaROSを起動します。
# APIキーが必要です。

# スクリプトのディレクトリを取得
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "=== DiaROS ChatGPT launcher ==="
echo "🤖 ChatGPT APIモードでDiaROSを起動します"
echo "💰 API使用料が発生します"
echo ""

# APIキーの確認
if [ -z "$OPENAI_API_KEY" ]; then
    echo "❌ エラー: OPENAI_API_KEY が設定されていません"
    echo ""
    echo "以下のコマンドでAPIキーを設定してください:"
    echo 'export OPENAI_API_KEY="sk-your-api-key"'
    echo ""
    echo "または、ローカルLLMモードで起動:"
    echo "./scripts/launch/launch_diaros_local.sh"
    exit 1
fi

echo "✅ OpenAI APIキー検出: ${OPENAI_API_KEY:0:20}..."
echo ""

# LLMモデル設定をクリア（ChatGPT APIを優先）
unset DIAROS_LLM_MODEL

# 通常の起動スクリプトを実行
exec "$SCRIPT_DIR/launch_diaros.sh"