#!/bin/bash

# 音声対話システム用モデル事前ダウンロードスクリプト
# システム起動前にすべての必要なモデルをダウンロード

set -e  # エラー時に停止

echo "🚀 DiaROS音声対話システム - モデル事前ダウンロードツール"
echo "=============================================================="

# スクリプトの場所を取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Python環境の確認
check_python_env() {
    if command -v pixi &> /dev/null; then
        if [[ -n "$CONDA_PREFIX" ]]; then
            echo "✅ Pixi環境を検出しました"
            return 0
        else
            echo "⚠️  Pixi環境で実行してください"
            echo "実行方法:"
            echo "  cd $PROJECT_ROOT"
            echo "  pixi shell"
            echo "  ./scripts/setup/predownload_all_models.sh"
            exit 1
        fi
    elif [[ -n "$VIRTUAL_ENV" ]]; then
        echo "✅ Python仮想環境を検出しました"
        return 0
    elif command -v python3 &> /dev/null; then
        echo "ℹ️  システムPythonを使用します"
        return 0
    else
        echo "❌ Pythonが見つかりません"
        exit 1
    fi
}

# 必要なパッケージの確認とインストール
check_dependencies() {
    echo ""
    echo "📦 依存関係をチェック中..."
    
    python3 -c "
import sys
required_packages = [
    'transformers',
    'torch', 
    'huggingface_hub',
    'bitsandbytes'
]

missing_packages = []
for package in required_packages:
    try:
        __import__(package)
        print(f'  ✅ {package}')
    except ImportError:
        print(f'  ❌ {package} (未インストール)')
        missing_packages.append(package)

if missing_packages:
    print(f'\\n⚠️  不足パッケージ: {missing_packages}')
    print('\\n自動インストールを実行しますか？ [y/N]')
    sys.exit(1)
else:
    print('\\n✅ すべての依存関係が満たされています')
" || {
    echo ""
    read -p "自動インストールを実行しますか？ [y/N]: " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "📦 パッケージをインストール中..."
        
        # DiaROS Pythonパッケージのインストール
        if [[ -d "$PROJECT_ROOT/DiaROS_py" ]]; then
            cd "$PROJECT_ROOT/DiaROS_py"
            python3 -m pip install . --user
            echo "✅ DiaROS Pythonパッケージをインストールしました"
        fi
        
        # 追加の必要パッケージ（4ビット量子化用）
        python3 -m pip install --user transformers torch huggingface_hub bitsandbytes
        echo "✅ 依存パッケージ（4ビット量子化用）をインストールしました"
    else
        echo "インストールをキャンセルしました"
        exit 1
    fi
}
}

# メイン実行
main() {
    # 環境チェック
    check_python_env
    check_dependencies
    
    echo ""
    echo "⏰ Gemma 3 モデル（4ビット量子化）のダウンロードを開始します..."
    echo "このプロセスには15-45分かかる場合があります"
    echo ""
    
    # Pythonスクリプトを実行
    python3 "$SCRIPT_DIR/predownload_all_models.py"
    
    exit_code=$?
    
    if [[ $exit_code -eq 0 ]]; then
        echo ""
        echo "🎉 Gemma 3 モデル（4ビット量子化）の事前ダウンロードが完了しました！"
        echo ""
        echo "次のステップ:"
        echo "1. 音声対話システムを起動:"
        echo "   ./scripts/launch/launch_diaros_local.sh"
        echo ""
        echo "2. または ChatGPT APIモード:"
        echo "   ./scripts/launch/launch_diaros_chatgpt.sh"
        echo ""
        echo "これで起動時間が大幅に短縮されます！"
    else
        echo ""
        echo "❌ ダウンロードが完了しませんでした"
        echo "エラーメッセージを確認して再実行してください"
        exit 1
    fi
}

# エラーハンドリング
trap 'echo ""; echo "❌ スクリプトが中断されました"; exit 1' INT TERM

# メイン実行
main "$@"