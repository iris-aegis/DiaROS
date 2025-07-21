#!/bin/bash
# Gemma 2モデルダウンロードスクリプト
# Pixi環境で実行されることを前提とする

# スクリプトのディレクトリを取得
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "=== Gemma 2モデルダウンロードツール ==="
echo ""

# Pixi環境のチェック
if [ -z "$CONDA_PREFIX" ]; then
    echo "❌ エラー: このスクリプトはPixi環境で実行する必要があります"
    echo ""
    echo "以下のコマンドを実行してください:"
    echo "  cd ~/_data/_DiaROS_mac/DiaROS_pixi/diaros_workspace"
    echo "  pixi shell"
    echo "  $0"
    exit 1
fi

echo "✅ Pixi環境を検出しました: $CONDA_PREFIX"
echo ""

# Pythonスクリプトを実行
echo "モデルダウンロードを開始します..."
echo ""

# Pixi環境のPythonを明示的に使用
"$CONDA_PREFIX/bin/python3" "$SCRIPT_DIR/download_gemma_model.py"

# 終了コードを保持
exit_code=$?

if [ $exit_code -eq 0 ]; then
    echo ""
    echo "✅ セットアップ完了！"
    echo ""
    echo "DiaROSを起動できます："
    echo "  $PROJECT_ROOT/scripts/launch/launch_diaros_local.sh"
else
    echo ""
    echo "❌ ダウンロードに失敗しました"
    echo "上記のエラーメッセージを確認してください"
    exit $exit_code
fi