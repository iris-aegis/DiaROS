#!/bin/bash
# test_api_nlg.sh - Pixi環境でAPIとNLGテストを実行

# スクリプトのディレクトリを取得
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SCRIPT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "🧪 DiaROS API/NLGテストツール"
echo "================================"
echo ""

# OS検出
OS_TYPE=$(uname -s)

if [[ "$OS_TYPE" == "Darwin" ]]; then
    # macOSの場合、Pixi環境で実行
    PARENT_DIR="$(cd "$SCRIPT_ROOT/.." && pwd)"
    PIXI_DIR="$PARENT_DIR/diaros_workspace"
    
    if [ ! -d "$PIXI_DIR" ]; then
        echo "❌ Pixi環境が見つかりません: $PIXI_DIR"
        exit 1
    fi
    
    echo "🔧 Pixi環境でテストを実行します..."
    
    # Pixi環境内で実行するための一時スクリプトを作成
    TEMP_SCRIPT=$(mktemp)
    cat > "$TEMP_SCRIPT" << EOF
#!/bin/bash
# ROS2環境変数の設定
export ROS_DISTRO=humble
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
export DIAROS_DEVICE=mps

# DiaROSパッケージのパス設定
DIAROS_ROOT="$SCRIPT_ROOT"
DIAROS_ROS_DIR="\$DIAROS_ROOT/DiaROS_ros"

# PYTHONPATHの設定
export PYTHONPATH="\$DIAROS_ROOT/DiaROS_py:\$PYTHONPATH"

# APIキーの確認
echo "📊 現在のAPI設定:"
if [ -n "\$OPENAI_API_KEY" ]; then
    echo "  ✅ OpenAI API: 設定済み (\${OPENAI_API_KEY:0:7}...)"
else
    echo "  ❌ OpenAI API: 未設定"
fi

if [ -n "\$ANTHROPIC_API_KEY" ]; then
    echo "  ✅ Anthropic API: 設定済み (\${ANTHROPIC_API_KEY:0:7}...)"
else
    echo "  ❌ Anthropic API: 未設定"
fi

echo ""

# テストスクリプトの実行
python3 "\$DIAROS_ROOT/scripts/test/test_api_nlg.py"
EOF
    
    chmod +x "$TEMP_SCRIPT"
    
    # Pixi環境で実行（環境変数を引き継ぐ）
    cd "$PIXI_DIR"
    # 現在の環境変数を引き継ぐ
    OPENAI_API_KEY="${OPENAI_API_KEY:-}" ANTHROPIC_API_KEY="${ANTHROPIC_API_KEY:-}" pixi run bash "$TEMP_SCRIPT"
    
    # 一時ファイルを削除
    rm -f "$TEMP_SCRIPT"
    
else
    # Linuxの場合、通常の環境で実行
    echo "🐧 Linux環境でテストを実行します..."
    
    # ROS2環境の設定
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    # DiaROSパスの設定
    export PYTHONPATH="$SCRIPT_ROOT/DiaROS_py:$PYTHONPATH"
    
    # APIキーの確認
    echo "📊 現在のAPI設定:"
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
    
    # テストスクリプトの実行
    python3 "$SCRIPT_ROOT/scripts/test/test_api_nlg.py"
fi