#!/bin/bash
# setup_ros2_env.sh - DiaROS用ROS2環境設定スクリプト
# このスクリプトをsourceすることで、ROS2環境が設定されます

echo "🚀 DiaROS ROS2環境設定スクリプト"
echo "=================================="

# スクリプトのディレクトリを取得
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# OS検出
OS_TYPE=$(uname -s)

# Pixi環境のチェックと設定
if [[ "$OS_TYPE" == "Darwin" ]]; then
    echo "📱 macOS環境を検出しました"
    
    # Pixi環境のパスを設定
    PIXI_DIR="$HOME/DiaROS_pixi/diaros_workspace"
    
    if [ ! -d "$PIXI_DIR" ]; then
        echo "❌ Pixi環境が見つかりません: $PIXI_DIR"
        echo "Pixiセットアップガイドを参照してください"
        return 1
    fi
    
    # Pixi環境のROS2設定
    echo "🔧 Pixi環境のROS2を設定中..."
    
    # Pixi環境の.pixi/envディレクトリを探す
    if [ -d "$PIXI_DIR/.pixi/envs/default" ]; then
        CONDA_PREFIX="$PIXI_DIR/.pixi/envs/default"
    elif [ -d "$PIXI_DIR/.pixi/env" ]; then
        CONDA_PREFIX="$PIXI_DIR/.pixi/env"
    else
        echo "❌ Pixi環境のenvディレクトリが見つかりません"
        return 1
    fi
    
    # ROS2環境変数の設定
    export ROS_DISTRO=humble
    export ROS_VERSION=2
    export ROS_PYTHON_VERSION=3
    export ROS_LOCALHOST_ONLY=1
    
    # ROS2のsetup.bashをsource
    if [ -f "$CONDA_PREFIX/setup.bash" ]; then
        source "$CONDA_PREFIX/setup.bash"
    else
        echo "⚠️  ROS2 setup.bashが見つかりません"
    fi
    
    # Python環境の設定
    export PYTHONPATH="$CONDA_PREFIX/lib/python3.9/site-packages:$PYTHONPATH"
    
else
    echo "🐧 Linux環境を検出しました"
    
    # 標準的なROS2インストールパスをチェック
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "❌ ROS2 Humbleが見つかりません"
        echo "ROS2をインストールしてください"
        return 1
    fi
fi

# DiaROSビルド済みパッケージの設定
DIAROS_ROS_DIR="$PROJECT_ROOT/DiaROS_ros"

if [ -d "$DIAROS_ROS_DIR/install" ]; then
    echo "📦 DiaROSパッケージを設定中..."
    
    # local_setup.bashが存在する場合はそれを使用
    if [ -f "$DIAROS_ROS_DIR/install/local_setup.bash" ]; then
        source "$DIAROS_ROS_DIR/install/local_setup.bash"
    else
        # 個別にパッケージを設定
        for pkg in interfaces diaros_package; do
            if [ -f "$DIAROS_ROS_DIR/install/$pkg/share/$pkg/local_setup.bash" ]; then
                source "$DIAROS_ROS_DIR/install/$pkg/share/$pkg/local_setup.bash"
            fi
        done
    fi
    
    # AMENT_PREFIX_PATHとPYTHONPATHの設定
    export AMENT_PREFIX_PATH="$DIAROS_ROS_DIR/install/diaros_package:$DIAROS_ROS_DIR/install/interfaces:$AMENT_PREFIX_PATH"
    
    # Pythonバージョンに応じたパス設定
    if [[ "$OS_TYPE" == "Darwin" ]]; then
        export PYTHONPATH="$DIAROS_ROS_DIR/install/diaros_package/lib/python3.9/site-packages:$DIAROS_ROS_DIR/install/interfaces/lib/python3.9/site-packages:$PYTHONPATH"
    else
        export PYTHONPATH="$DIAROS_ROS_DIR/install/diaros_package/lib/python3.10/site-packages:$DIAROS_ROS_DIR/install/interfaces/lib/python3.10/site-packages:$PYTHONPATH"
    fi
else
    echo "⚠️  DiaROSパッケージがビルドされていません"
    echo "以下のコマンドでビルドしてください："
    echo "  cd $DIAROS_ROS_DIR"
    echo "  colcon build"
fi

# DiaROS Pythonモジュールのパス設定
export PYTHONPATH="$PROJECT_ROOT/DiaROS_py:$PYTHONPATH"

# rcutilsエラーメッセージの抑制
export RCUTILS_LOGGING_SEVERITY_THRESHOLD='ERROR'
export RCUTILS_COLORIZED_OUTPUT='0'

# Apple Silicon GPU使用設定（macOSの場合）
if [[ "$OS_TYPE" == "Darwin" ]]; then
    export DIAROS_DEVICE=mps
fi

# 設定完了メッセージ
echo ""
echo "✅ ROS2環境設定が完了しました！"
echo ""
echo "確認コマンド:"
echo "  ros2 --version      # ROS2バージョン確認"
echo "  ros2 node list      # 起動中のノード確認"
echo "  ros2 topic list     # トピック一覧"
echo ""
echo "デバッグツール:"
echo "  python3 $SCRIPT_DIR/test_diaros_response.py"
echo "  python3 $SCRIPT_DIR/debug_diaros_flow.py"
echo ""

# 関数として環境チェックを提供
check_ros2_env() {
    echo "🔍 ROS2環境チェック"
    echo "===================="
    echo "ROS_DISTRO: $ROS_DISTRO"
    echo "ROS_VERSION: $ROS_VERSION"
    echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"
    echo "PYTHONPATH: $PYTHONPATH"
    
    if command -v ros2 &> /dev/null; then
        echo "✅ ros2コマンドが利用可能です"
        ros2 --version
    else
        echo "❌ ros2コマンドが見つかりません"
    fi
}