#!/bin/bash
# =========================================
# DiaROS简潔起動スクリプト (speech_inputノード除外)
# Docker環境およびシンプルな環境向け (VOICEVOX自動起動対応)
# =========================================

set -e

echo "=== DiaROS起動 (speech_inputノード除外) ==="

# VOICEVOX自動起動関数
start_voicevox() {
    echo "🎤 VOICEVOX状態を確認中..."

    # VOICEVOXが既に動作しているかチェック
    if curl -fs http://localhost:50021/version >/dev/null 2>&1; then
        VOICEVOX_VERSION=$(curl -s http://localhost:50021/version 2>/dev/null)
        echo "✅ VOICEVOXは既に動作中です (バージョン: $VOICEVOX_VERSION)"
        return 0
    fi

    # VOICEVOXの実行ファイルパス
    VOICEVOX_RUN="/opt/voicevox_engine/linux-nvidia/run"

    if [ -x "$VOICEVOX_RUN" ]; then
        echo "▶ VOICEVOXを起動中..."

        # バックグラウンドでVOICEVOXを起動
        nohup "$VOICEVOX_RUN" > /tmp/voicevox.log 2>&1 &
        VOICEVOX_PID=$!
        echo "  VOICEVOX PID: $VOICEVOX_PID"

        # 起動待機
        echo "⏳ VOICEVOXの起動を待機中..."
        for i in {1..30}; do
            if curl -fs http://localhost:50021/version >/dev/null 2>&1; then
                VOICEVOX_VERSION=$(curl -s http://localhost:50021/version 2>/dev/null)
                echo "✅ VOICEVOXが正常に起動しました (バージョン: $VOICEVOX_VERSION)"
                return 0
            fi
            echo "  試行 $i/30..."
            sleep 2
        done

        echo ""
        echo "============================================================"
        echo -e "\033[91m❌ VOICEVOX STARTUP FAILED\033[0m"
        echo "============================================================"
        echo "VOICEVOXの起動が60秒以内に完了しませんでした"
        echo "💡 ログを確認してください: /tmp/voicevox.log"
        echo "============================================================"
        exit 1
    else
        echo ""
        echo "============================================================"
        echo -e "\033[91m❌ VOICEVOX NOT FOUND\033[0m"
        echo "============================================================"
        echo "VOICEVOX実行ファイルが見つかりません: $VOICEVOX_RUN"
        echo "💡 VOICEVOXがインストールされていない可能性があります"
        echo "============================================================"
        exit 1
    fi
}

# 作業ディレクトリ
DIAROS_DIR="/workspace/DiaROS/DiaROS_ros"

# ディレクトリ存在確認
if [ ! -d "$DIAROS_DIR" ]; then
    echo "❌ DiaROSディレクトリが見つかりません: $DIAROS_DIR"
    exit 1
fi

cd "$DIAROS_DIR"
echo "📁 作業ディレクトリ: $(pwd)"

# VOICEVOX起動
start_voicevox

# ROS2環境セットアップ
echo "🔧 ROS2環境をセットアップ中..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble環境をロード"
else
    echo "❌ ROS2 Humble環境が見つかりません"
    exit 1
fi

# ★指定順序での完全ビルドプロセス
echo "🔧 DiaROSの完全ビルドを実行中..."

# 1. Pythonモジュールの再インストール
echo "📦 Step 1/6: Pythonモジュールを再インストール中..."
cd /workspace/DiaROS/DiaROS_py
pip install . --upgrade
echo "✅ Pythonモジュールの再インストール完了"

# 2. DiaROS_rosディレクトリに移動
echo "📦 Step 2/6: DiaROS_rosディレクトリに移動中..."
cd /workspace/DiaROS/DiaROS_ros
echo "✅ 作業ディレクトリ: $(pwd)"

# 3. ROS2環境セットアップ
echo "📦 Step 3/6: ROS2環境をセットアップ中..."
source /opt/ros/humble/setup.bash
echo "✅ ROS2 Humble環境をロード"

# 4. interfacesパッケージのビルド
echo "📦 Step 4/6: interfacesパッケージをビルド中..."
colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces
. ./install/local_setup.bash
echo "✅ interfacesパッケージのビルド完了"

# 5. diaros_packageのビルド
echo "📦 Step 5/6: diaros_packageをビルド中..."
colcon build --packages-select diaros_package
. ./install/local_setup.bash
echo "✅ diaros_packageのビルド完了"

echo "🎉 Step 6/6: 完全ビルドプロセス完了"

# DiaROSローカル環境再セットアップ
if [ -f "$DIAROS_DIR/install/local_setup.bash" ]; then
    source "$DIAROS_DIR/install/local_setup.bash"
    echo "✅ DiaROSローカル環境をロード"
else
    echo "❌ DiaROSビルドが見つかりません: $DIAROS_DIR/install/local_setup.bash"
    exit 1
fi

# NumPy互換性確認（aubio対応）
echo "🔧 NumPy互換性の確認..."
if python3 -c "import numpy" 2>/dev/null; then
    python3 -c "import numpy; print(f'NumPy version: {numpy.__version__}')"
    if python3 -c "import numpy; exit(0 if numpy.__version__.startswith('2.') else 1)" 2>/dev/null; then
        echo "⚠️  NumPy 2.xが検出されました。aubio互換性のためNumPy 1.xにダウングレードします..."
        pip3 install --force-reinstall "numpy==1.24.3"
        echo "✅ NumPy 1.24.3にダウングレードしました"
    fi
else
    echo "⚠️  NumPyが見つかりません"
fi

# 環境変数設定
export ROS_DOMAIN_ID=0
export DIAROS_DEVICE="${DIAROS_DEVICE:-cpu}"
# NLGのプロンプトディレクトリを明示的に設定
export DIAROS_PROMPTS_DIR="/workspace/DiaROS/DiaROS_py/diaros/prompts"

echo "📋 設定確認:"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  DIAROS_DEVICE: $DIAROS_DEVICE"
echo "  DIAROS_PROMPTS_DIR: $DIAROS_PROMPTS_DIR"
echo "  speech_inputノード: 除外 (mic:=false)"

echo ""
echo "🚀 DiaROSを起動します (speech_inputノード除外)..."
echo "📝 以下のノードが起動されます:"
echo "  - acoustic_analysis"
echo "  - automatic_speech_recognition"
echo "  - natural_language_understanding"
echo "  - dialog_management"
echo "  - natural_language_generation ⭐ (ローカルPC上)"
echo "  - speech_synthesis"
echo "  - turn_taking"
echo "  - back_channel"
echo ""
echo "🎵 ros2 bag playで音声データを再生してください"
echo ""

# DiaROS起動 (speech_inputノード除外)
exec ros2 launch diaros_package sdsmod.launch.py mic:=false