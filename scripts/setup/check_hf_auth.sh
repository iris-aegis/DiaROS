#!/bin/bash
# HuggingFace認証状態をチェック

echo "=== HuggingFace認証状態確認 ==="
echo ""

# Pixi環境のチェック
if [ -z "$CONDA_PREFIX" ]; then
    echo "❌ エラー: Pixi環境で実行してください"
    echo ""
    echo "実行方法:"
    echo "  cd ~/_data/_DiaROS_mac/DiaROS_pixi/diaros_workspace"
    echo "  pixi shell"
    echo "  ../DiaROS_imamoto/scripts/setup/check_hf_auth.sh"
    exit 1
fi

echo "✅ Pixi環境検出: $CONDA_PREFIX"
echo ""

# HuggingFaceトークンの確認
if [ -n "$HF_TOKEN" ] || [ -n "$HUGGINGFACE_TOKEN" ]; then
    echo "✅ HuggingFaceトークンが環境変数に設定されています"
    if [ -n "$HF_TOKEN" ]; then
        echo "   HF_TOKEN: ${HF_TOKEN:0:10}..."
    else
        echo "   HUGGINGFACE_TOKEN: ${HUGGINGFACE_TOKEN:0:10}..."
    fi
else
    echo "⚠️  HuggingFaceトークンが環境変数に設定されていません"
fi
echo ""

# huggingface-cliでのログイン確認
echo "HuggingFace CLIログイン状態:"
huggingface-cli whoami 2>&1 || echo "❌ 未ログイン"
echo ""

# 次のステップの案内
if ! huggingface-cli whoami >/dev/null 2>&1; then
    echo "=== 次のステップ ==="
    echo ""
    echo "1. HuggingFaceでGemma 2へのアクセス許可を取得:"
    echo "   https://huggingface.co/google/gemma-2-2b-it"
    echo "   → 「Agree and access repository」をクリック"
    echo ""
    echo "2. HuggingFace CLIでログイン:"
    echo "   huggingface-cli login"
    echo ""
    echo "3. モデルをダウンロード:"
    echo "   python3 ../DiaROS_imamoto/scripts/setup/download_gemma_model.py"
    echo ""
fi