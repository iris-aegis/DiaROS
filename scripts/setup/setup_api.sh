#!/bin/bash
# setup_api.sh - DiaROS用APIキー設定スクリプト

echo "🔧 DiaROS API設定ツール"
echo "======================="
echo ""

# 現在の設定を確認
echo "📊 現在の設定状態:"
if [ -n "$OPENAI_API_KEY" ]; then
    echo "  ✅ OpenAI API: 設定済み (${OPENAI_API_KEY:0:7}...)"
else
    echo "  ❌ OpenAI API: 未設定"
fi

if [ -n "$ANTHROPIC_API_KEY" ]; then
    echo "  ✅ Anthropic API: 設定済み (${ANTHROPIC_API_KEY:0:7}...)"
else
    echo "  ❌ Anthropic API: 未設定"
fi

echo ""

# APIキーの設定を促す
echo "🔑 APIキーを設定してください:"
echo ""
echo "1. OpenAI API (ChatGPT) - 推奨"
echo "   以下のコマンドを実行してください:"
echo "   export OPENAI_API_KEY=\"sk-your-openai-api-key\""
echo ""
echo "2. Anthropic API (Claude) - オプション"
echo "   export ANTHROPIC_API_KEY=\"sk-ant-your-anthropic-api-key\""
echo ""
echo "3. 永続的に設定する場合:"
echo "   echo 'export OPENAI_API_KEY=\"sk-your-key\"' >> ~/.zshrc"
echo "   source ~/.zshrc"
echo ""
echo "📝 APIキーの取得方法:"
echo "   OpenAI: https://platform.openai.com/api-keys"
echo "   Anthropic: https://console.anthropic.com/settings/keys"
echo ""
echo "💡 ローカルモデル（APIキー不要）:"
echo "   APIキーを設定しない場合、高速なローカルLLMが使用されます"
echo "   - デフォルト: Gemma 2 (google/gemma-2-2b-it)"
echo "   - 軽量版: export DIAROS_LLM_MODEL=rinna-small"
echo ""
echo "   Gemma 2を使用する場合は事前にモデルをダウンロード:"
echo "   ./scripts/setup/download_gemma_model.sh"
echo ""

# API接続テスト機能
if [ "$1" = "test" ]; then
    echo "🧪 API接続テストを実行します..."
    
    if [ -n "$OPENAI_API_KEY" ]; then
        echo -n "  OpenAI API: "
        response=$(curl -s -o /dev/null -w "%{http_code}" \
            -H "Authorization: Bearer $OPENAI_API_KEY" \
            https://api.openai.com/v1/models)
        if [ "$response" = "200" ]; then
            echo "✅ 接続成功"
        else
            echo "❌ 接続失敗 (HTTP $response)"
        fi
    fi
    
    if [ -n "$ANTHROPIC_API_KEY" ]; then
        echo -n "  Anthropic API: "
        # Anthropic APIのテスト（簡易版）
        echo "✅ キー設定済み（詳細テストは実行時に確認）"
    fi
fi