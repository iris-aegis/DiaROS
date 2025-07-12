#!/bin/bash

# setup_chatgpt_api.sh - OpenAI APIキー設定スクリプト

echo "=========================================="
echo "DiaROS - OpenAI API設定スクリプト"
echo "=========================================="
echo ""
echo "このスクリプトはOpenAI APIキーを設定します。"
echo "APIキーは以下のサイトから取得できます："
echo "https://platform.openai.com/api-keys"
echo ""

# APIキーの入力
echo -n "OpenAI APIキーを入力してください (sk-...): "
read -r api_key

# 入力確認
if [ -z "$api_key" ]; then
    echo "エラー: APIキーが入力されていません。"
    exit 1
fi

# APIキーの形式チェック
if [[ ! "$api_key" =~ ^sk- ]]; then
    echo "警告: APIキーは通常 'sk-' で始まります。"
    echo -n "このまま続行しますか？ (y/N): "
    read -r confirm
    if [ "$confirm" != "y" ] && [ "$confirm" != "Y" ]; then
        echo "設定をキャンセルしました。"
        exit 0
    fi
fi

# 環境変数の設定
export OPENAI_API_KEY="$api_key"

# シェル設定ファイルの検出
if [ -n "$ZSH_VERSION" ]; then
    SHELL_CONFIG="$HOME/.zshrc"
elif [ -n "$BASH_VERSION" ]; then
    SHELL_CONFIG="$HOME/.bashrc"
else
    SHELL_CONFIG="$HOME/.profile"
fi

# 既存の設定を確認
if grep -q "OPENAI_API_KEY" "$SHELL_CONFIG" 2>/dev/null; then
    echo ""
    echo "既存のOPENAI_API_KEY設定が見つかりました。"
    echo -n "上書きしますか？ (y/N): "
    read -r overwrite
    if [ "$overwrite" = "y" ] || [ "$overwrite" = "Y" ]; then
        # 既存の行を削除
        if [[ "$OSTYPE" == "darwin"* ]]; then
            sed -i '' '/export OPENAI_API_KEY=/d' "$SHELL_CONFIG"
        else
            sed -i '/export OPENAI_API_KEY=/d' "$SHELL_CONFIG"
        fi
    else
        echo "設定をキャンセルしました。"
        exit 0
    fi
fi

# 設定ファイルに追加
echo "" >> "$SHELL_CONFIG"
echo "# OpenAI API Key for DiaROS" >> "$SHELL_CONFIG"
echo "export OPENAI_API_KEY='$api_key'" >> "$SHELL_CONFIG"

echo ""
echo "✅ APIキーが設定されました！"
echo ""
echo "設定を有効にするには、以下のコマンドを実行してください："
echo "  source $SHELL_CONFIG"
echo ""
echo "または、新しいターミナルを開いてください。"
echo ""
echo "設定の確認："
echo "  echo \$OPENAI_API_KEY"
echo ""
echo "DiaROSを起動する前に、設定が反映されていることを確認してください。"