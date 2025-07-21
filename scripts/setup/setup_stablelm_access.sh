#!/bin/bash
# setup_stablelm_access.sh - Japanese StableLM 2へのアクセス設定
#
# このスクリプトは、Japanese StableLM 2モデルへのアクセス設定を行います。

echo "=== Japanese StableLM 2 アクセス設定 ==="
echo ""
echo "Japanese StableLM 2は制限付きモデルです。"
echo "使用するには以下の手順が必要です："
echo ""
echo "1. HuggingFaceにログイン"
echo "   huggingface-cli login"
echo ""
echo "2. 以下のURLでアクセス許可をリクエスト"
echo "   https://huggingface.co/stabilityai/japanese-stablelm-2-instruct-1_6b"
echo ""
echo "3. アクセスが承認されたら、環境変数を設定"
echo "   export HF_TOKEN=your_token_here"
echo ""
echo "4. DiaROSでStableLM 2を使用"
echo "   export DIAROS_LLM_MODEL=stablelm-2"
echo ""
echo "現在の設定状況:"

# HuggingFaceトークンの確認
if [ ! -z "$HF_TOKEN" ] || [ ! -z "$HUGGINGFACE_TOKEN" ]; then
    echo "✅ HuggingFaceトークンが設定されています"
else
    # CLIログインの確認
    if command -v huggingface-cli &> /dev/null; then
        if huggingface-cli whoami &> /dev/null; then
            echo "✅ HuggingFace CLIでログイン済み"
        else
            echo "❌ HuggingFaceにログインしていません"
        fi
    else
        echo "❌ huggingface-cliがインストールされていません"
        echo "   pip install huggingface-hub でインストールしてください"
    fi
fi

echo ""
echo "代替モデル（アクセス制限なし）:"
echo "  - rinna-small: 最速・軽量"
echo "  - calm-small: バランス型"
echo "  - phi-3-mini: 超高速（要HFトークン）"