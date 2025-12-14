#!/bin/bash
# launch_diaros_local.sh - ローカルLLMモードでDiaROSを起動
# 
# このスクリプトは、APIキーを無効化して高速なローカルLLMモデルで
# DiaROSを起動します。

# スクリプトのディレクトリを取得
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "=== DiaROS Local LLM launcher ==="
echo "🏠 ローカルLLMモードでDiaROSを起動します"
echo "⚡ 高速応答（目標: <500ms）"
echo ""

# APIキーを無効化
unset OPENAI_API_KEY
unset ANTHROPIC_API_KEY

# ローカルLLMモデルを設定（rinna-smallを使用）
export DIAROS_LLM_MODEL=rinna-small

echo "📊 設定:"
echo "  - LLMモデル: rinna-small（最速・軽量）"
echo "  - APIキー: 無効化済み"
echo ""
echo "💡 代替オプション:"
echo "  - 最速: export DIAROS_LLM_MODEL=rinna-small"
echo "  - 中品質: export DIAROS_LLM_MODEL=calm-small"
echo "  - 別の高品質: export DIAROS_LLM_MODEL=rinna-neox"
echo "  - 超高速: export DIAROS_LLM_MODEL=phi-3-mini"
echo "  - API使用: export OPENAI_API_KEY='your-api-key'"
echo ""

# 通常の起動スクリプトを実行
exec "$SCRIPT_DIR/launch_diaros.sh"