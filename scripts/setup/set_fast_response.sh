#\!/bin/bash
# DiaROS高速応答設定スクリプト

echo "DiaROS高速応答設定を行います..."
echo ""

# APIキーの確認
if [ \! -z "$OPENAI_API_KEY" ]; then
    echo "✅ OpenAI APIキーが設定されています"
    echo "   → 最速の応答（~500-1000ms）が利用可能です"
elif [ \! -z "$ANTHROPIC_API_KEY" ]; then
    echo "✅ Anthropic APIキーが設定されています"
    echo "   → 高速応答（~800-1200ms）が利用可能です"
else
    echo "⚠️  APIキーが設定されていません"
    echo "   → ローカルモデルを使用します（~2000-5000ms）"
    echo ""
    echo "高速応答のためには、以下のいずれかを設定してください："
    echo "  export OPENAI_API_KEY='sk-your-key'"
    echo "  export ANTHROPIC_API_KEY='sk-ant-your-key'"
    echo ""
    echo "または、setup_api.shを実行してください："
    echo "  ./scripts/setup/setup_api.sh"
fi

# ローカルモデルの設定（APIキーがない場合）
if [ -z "$OPENAI_API_KEY" ] && [ -z "$ANTHROPIC_API_KEY" ]; then
    echo ""
    echo "ローカルモデルの選択："
    echo "1) stablelm-2 (Japanese StableLM 2, ~3.2GB) - 推奨"
    echo "2) rinna-small (最速, ~130MB)"
    echo "3) calm-small (バランス, ~400MB)"
    echo "4) rinna-neox (高品質, ~560MB)"
    echo "5) phi-3-mini (Phi-3-mini, 超高速, ~7.6GB)"
    echo "6) elyza-7b (ELYZA-japanese-Llama-2-7b, 高品質日本語, ~13GB)"
    echo "7) gemma-2b (Google Gemma 2, ~5GB)"
    echo ""
    read -p "選択してください [1-7] (デフォルト: 1): " choice
    
    case $choice in
        2)
            export DIAROS_LLM_MODEL="rinna-small"
            echo "✅ rinna-smallモデルを設定しました（最速）"
            ;;
        3)
            export DIAROS_LLM_MODEL="calm-small"
            echo "✅ calm-smallモデルを設定しました"
            ;;
        4)
            export DIAROS_LLM_MODEL="rinna-neox"
            echo "✅ rinna-neoxモデルを設定しました"
            ;;
        5)
            export DIAROS_LLM_MODEL="phi-3-mini"
            echo "✅ Phi-3-miniモデルを設定しました"
            echo "   超高速な応答が期待できます"
            ;;
        6)
            export DIAROS_LLM_MODEL="elyza-7b"
            echo "✅ ELYZA-japanese-Llama-2-7bモデルを設定しました"
            echo "   高品質な日本語応答が期待できます"
            ;;
        7)
            export DIAROS_LLM_MODEL="gemma-2b"
            echo "✅ gemma-2bモデルを設定しました"
            ;;
        *)
            export DIAROS_LLM_MODEL="stablelm-2"
            echo "✅ Japanese StableLM 2モデルを設定しました（推奨）"
            echo "   高速で高品質な応答が期待できます"
            ;;
    esac
fi

# デバイス設定（macOS）
if [[ "$OSTYPE" == "darwin"* ]]; then
    export DIAROS_DEVICE="mps"
    echo ""
    echo "✅ Apple Silicon GPU (MPS)を使用します"
fi

echo ""
echo "設定が完了しました！"
echo ""
echo "DiaROSを起動するには："
echo "  ros2 launch diaros_package sdsmod.launch.py"