#!/bin/bash
# test_diaros_response.sh - DiaROS応答テストツール実行スクリプト
# ROS2環境を自動設定してテストツールを実行します

# スクリプトのディレクトリを取得
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "🧪 DiaROS応答テストツール"
echo "========================="
echo ""

# OS検出
OS_TYPE=$(uname -s)

if [[ "$OS_TYPE" == "Darwin" ]]; then
    # macOSの場合、Pixi環境で実行
    PIXI_DIR="$HOME/DiaROS_pixi/diaros_workspace"
    
    if [ ! -d "$PIXI_DIR" ]; then
        echo "❌ Pixi環境が見つかりません: $PIXI_DIR"
        exit 1
    fi
    
    echo "🔧 Pixi環境でテストツールを起動します..."
    
    # Pixi環境内で実行するための一時スクリプトを作成
    TEMP_SCRIPT=$(mktemp)
    cat > "$TEMP_SCRIPT" << 'EOF'
#!/bin/bash
# ROS2環境変数の設定
export ROS_DISTRO=humble
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
export ROS_LOCALHOST_ONLY=1
export RCUTILS_LOGGING_SEVERITY_THRESHOLD='ERROR'
export RCUTILS_COLORIZED_OUTPUT='0'

# DiaROSパッケージのパス設定
DIAROS_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DIAROS_ROS_DIR="$DIAROS_ROOT/DiaROS_ros"

# インストール済みパッケージの設定
if [ -f "$DIAROS_ROS_DIR/install/local_setup.bash" ]; then
    source "$DIAROS_ROS_DIR/install/local_setup.bash"
fi

export AMENT_PREFIX_PATH="$DIAROS_ROS_DIR/install/diaros_package:$DIAROS_ROS_DIR/install/interfaces:$AMENT_PREFIX_PATH"
export PYTHONPATH="$DIAROS_ROS_DIR/install/diaros_package/lib/python3.9/site-packages:$DIAROS_ROS_DIR/install/interfaces/lib/python3.9/site-packages:$PYTHONPATH"
export PYTHONPATH="$DIAROS_ROOT/DiaROS_py:$PYTHONPATH"

# テストツールの実行
python3 "$DIAROS_ROOT/scripts/test_diaros_response.py"
EOF
    
    chmod +x "$TEMP_SCRIPT"
    
    # Pixi環境で実行
    cd "$PIXI_DIR"
    pixi run bash -c "cd '$SCRIPT_DIR/..' && '$TEMP_SCRIPT'"
    
    # 一時ファイルを削除
    rm -f "$TEMP_SCRIPT"
    
else
    # Linuxの場合、通常のROS2環境で実行
    echo "🐧 Linux環境でテストツールを起動します..."
    
    # ROS2環境の設定
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "❌ ROS2 Humbleが見つかりません"
        exit 1
    fi
    
    # DiaROSパッケージの設定
    DIAROS_ROS_DIR="$SCRIPT_DIR/../DiaROS_ros"
    if [ -f "$DIAROS_ROS_DIR/install/local_setup.bash" ]; then
        source "$DIAROS_ROS_DIR/install/local_setup.bash"
    fi
    
    # 環境変数の設定
    export RCUTILS_LOGGING_SEVERITY_THRESHOLD='ERROR'
    export RCUTILS_COLORIZED_OUTPUT='0'
    export PYTHONPATH="$SCRIPT_DIR/../DiaROS_py:$PYTHONPATH"
    
    # テストツールの実行
    python3 "$SCRIPT_DIR/test_diaros_response.py"
fi