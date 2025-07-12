#!/bin/bash
# debug_diaros_flow.sh - DiaROS対話フローデバッグツール実行スクリプト
# ROS2環境を自動設定してデバッグツールを実行します

# スクリプトのディレクトリを取得
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "🐛 DiaROS対話フローデバッガー"
echo "============================="
echo ""

# OS検出
OS_TYPE=$(uname -s)

if [[ "$OS_TYPE" == "Darwin" ]]; then
    # macOSの場合、Pixi環境で実行
    # スクリプトの2階層上から相対パスでdiaros_workspaceを探す
    SCRIPT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
    
    # 現在のプロジェクトが DiaROS_imamoto の場合、その親ディレクトリに diaros_workspace があるはず
    PARENT_DIR="$(cd "$SCRIPT_ROOT/.." && pwd)"
    PIXI_DIR="$PARENT_DIR/diaros_workspace"
    
    if [ ! -d "$PIXI_DIR" ]; then
        echo "❌ Pixi環境が見つかりません: $PIXI_DIR"
        echo "🔍 現在のディレクトリ: $SCRIPT_ROOT"
        echo "🔍 親ディレクトリ: $PARENT_DIR"
        exit 1
    fi
    
    echo "🔧 Pixi環境でデバッガーを起動します..."
    
    # Pixi環境内で実行するための一時スクリプトを作成
    TEMP_SCRIPT=$(mktemp)
    cat > "$TEMP_SCRIPT" << 'EOF'
#!/bin/bash
# ROS2環境変数の設定
export ROS_DISTRO=humble
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
export RCUTILS_LOGGING_SEVERITY_THRESHOLD='ERROR'
export RCUTILS_COLORIZED_OUTPUT='0'

# DiaROSパッケージのパス設定
# TEMP_SCRIPT内では、pixi run bash -c から実行されるため、
# $0 が一時スクリプトを指すことに注意
DIAROS_ROOT="__DIAROS_ROOT__"
DIAROS_ROS_DIR="$DIAROS_ROOT/DiaROS_ros"

# インストール済みパッケージの設定
if [ -f "$DIAROS_ROS_DIR/install/local_setup.bash" ]; then
    source "$DIAROS_ROS_DIR/install/local_setup.bash"
fi

export AMENT_PREFIX_PATH="$DIAROS_ROS_DIR/install/diaros_package:$DIAROS_ROS_DIR/install/interfaces:$AMENT_PREFIX_PATH"
export PYTHONPATH="$DIAROS_ROS_DIR/install/diaros_package/lib/python3.9/site-packages:$DIAROS_ROS_DIR/install/interfaces/lib/python3.9/site-packages:$PYTHONPATH"
export PYTHONPATH="$DIAROS_ROOT/DiaROS_py:$PYTHONPATH"

# デバッグツールの実行
python3 "$DIAROS_ROOT/scripts/debug/debug_diaros_flow.py"
EOF
    
    chmod +x "$TEMP_SCRIPT"
    
    # DIAROS_ROOTを一時スクリプトに置換
    sed -i.bak "s|__DIAROS_ROOT__|$SCRIPT_ROOT|g" "$TEMP_SCRIPT"
    rm -f "$TEMP_SCRIPT.bak"
    
    # Pixi環境で実行
    cd "$PIXI_DIR"
    pixi run bash "$TEMP_SCRIPT"
    
    # 一時ファイルを削除
    rm -f "$TEMP_SCRIPT"
    
else
    # Linuxの場合、通常のROS2環境で実行
    echo "🐧 Linux環境でデバッガーを起動します..."
    
    # ROS2環境の設定
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "❌ ROS2 Humbleが見つかりません"
        exit 1
    fi
    
    # DiaROSパッケージの設定
    DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
    DIAROS_ROS_DIR="$DIAROS_ROOT/DiaROS_ros"
    if [ -f "$DIAROS_ROS_DIR/install/local_setup.bash" ]; then
        source "$DIAROS_ROS_DIR/install/local_setup.bash"
    fi
    
    # 環境変数の設定
    export ROS_DOMAIN_ID=0
    export ROS_LOCALHOST_ONLY=1
    export RCUTILS_LOGGING_SEVERITY_THRESHOLD='ERROR'
    export RCUTILS_COLORIZED_OUTPUT='0'
    export PYTHONPATH="$DIAROS_ROOT/DiaROS_py:$PYTHONPATH"
    
    # デバッグツールの実行
    python3 "$DIAROS_ROOT/scripts/debug/debug_diaros_flow.py"
fi