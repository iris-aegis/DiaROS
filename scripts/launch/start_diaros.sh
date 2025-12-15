#!/bin/bash
# DiaROS起動スクリプト

# 色付きの出力
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}🚀 DiaROS起動準備${NC}"
echo "=================================="
echo ""
echo -e "${YELLOW}💡 使用方法:${NC}"
echo "  通常起動: bash start_diaros.sh"
echo "  強制ビルド: FORCE_BUILD=true bash start_diaros.sh"
echo ""

# power_calibration.wavファイルの存在確認
if [ ! -f "/workspace/power_calibration.wav" ]; then
    echo -e "${YELLOW}⚠️  power_calibration.wavファイルが見つかりません。コピーします...${NC}"
    if [ -f "/workspace/DiaROS_py/power_calibration.wav" ]; then
        cp /workspace/DiaROS_py/power_calibration.wav /workspace/
        echo -e "${GREEN}✅ ファイルをコピーしました${NC}"
    else
        echo -e "${YELLOW}⚠️  ソースファイルが見つかりません。スキップします${NC}"
    fi
fi

# HuggingFaceトークンの確認
check_hf_token() {
    # 複数の方法でHuggingFaceトークンを確認
    
    # 1. 環境変数HF_TOKENをチェック
    if [ -n "$HF_TOKEN" ]; then
        echo -e "${GREEN}✅ HuggingFaceトークン(HF_TOKEN)が設定されています${NC}"
        return 0
    fi
    
    # 2. 環境変数HUGGING_FACE_HUB_TOKENをチェック
    if [ -n "$HUGGING_FACE_HUB_TOKEN" ]; then
        echo -e "${GREEN}✅ HuggingFaceトークン(HUGGING_FACE_HUB_TOKEN)が設定されています${NC}"
        export HF_TOKEN="$HUGGING_FACE_HUB_TOKEN"
        return 0
    fi
    
    # 3. huggingface-cli whoamiでログイン状態をチェック
    if command -v huggingface-cli >/dev/null 2>&1; then
        if huggingface-cli whoami >/dev/null 2>&1; then
            echo -e "${GREEN}✅ HuggingFace CLIでログイン済みです${NC}"
            # ログイン済みの場合、トークンを環境変数にエクスポート
            TOKEN=$(python3 -c "from huggingface_hub import HfApi; print(HfApi().token)" 2>/dev/null || echo "")
            if [ -n "$TOKEN" ] && [ "$TOKEN" != "None" ]; then
                export HF_TOKEN="$TOKEN"
                echo -e "${GREEN}✅ トークンを環境変数に設定しました${NC}"
            fi
            return 0
        fi
    fi
    
    # 4. ~/.cache/huggingface/token ファイルをチェック
    if [ -f "$HOME/.cache/huggingface/token" ]; then
        TOKEN=$(cat "$HOME/.cache/huggingface/token" 2>/dev/null)
        if [ -n "$TOKEN" ]; then
            export HF_TOKEN="$TOKEN"
            echo -e "${GREEN}✅ HuggingFaceトークンファイルから読み込みました${NC}"
            return 0
        fi
    fi
    
    return 1
}

if ! check_hf_token; then
    echo -e "${YELLOW}⚠️  HuggingFaceトークンが設定されていません${NC}"
    echo "ターンテイキング機能を使用する場合は、以下のいずれかの方法でトークンを設定してください："
    echo "1. export HF_TOKEN=your_token"
    echo "2. export HUGGING_FACE_HUB_TOKEN=your_token"
    echo "3. huggingface-cli login"
    echo ""
    
    # huggingface-cliが利用可能かチェック
    if command -v huggingface-cli >/dev/null 2>&1; then
        read -p "今すぐHuggingFace CLIでログインしますか？ (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            echo -e "${YELLOW}🔑 HuggingFace CLIでログイン中...${NC}"
            huggingface-cli login
            echo ""
            # ログイン後に再チェック
            if check_hf_token; then
                echo -e "${GREEN}✅ ログインが完了しました${NC}"
            else
                echo -e "${YELLOW}⚠️  ログインが完了しましたが、トークンの取得に失敗しました${NC}"
                echo -e "${YELLOW}💡 手動でトークンを設定することをお勧めします: export HF_TOKEN=your_token${NC}"
            fi
        else
            echo -e "${YELLOW}⚠️  ターンテイキング機能は使用できません${NC}"
        fi
    else
        echo -e "${YELLOW}⚠️  huggingface-cliが見つかりません${NC}"
        echo "pip install huggingface_hub でインストールしてください"
        echo "または手動でトークンを設定: export HF_TOKEN=your_token"
    fi
fi

# 音声デバイスの設定確認
if [ -f "/workspace/config/audio_device.conf" ]; then
    source /workspace/config/audio_device.conf
    echo -e "${GREEN}✅ 音声デバイス設定を読み込みました: AUDIO_DEVICE_INDEX=$AUDIO_DEVICE_INDEX${NC}"
fi

echo ""
echo -e "${GREEN}🎤 VOICEVOX起動確認${NC}"
echo "=================================="

# VOICEVOX起動確認・自動起動機能
check_voicevox() {
    if curl -fs http://localhost:50021/version >/dev/null 2>&1; then
        echo -e "${GREEN}✅ VOICEVOX is already running${NC}"
        return 0
    else
        return 1
    fi
}

start_voicevox() {
    # OS判定
    OS=$(uname -s)
    case "$OS" in
        Darwin)
            # macOS用VOICEVOX起動
            VOICEVOX_APP="$HOME/_data/tools/VOICEVOX/VOICEVOX.app"
            if [ -d "$VOICEVOX_APP" ]; then
                echo -e "${YELLOW}▶ Launching VOICEVOX.app for macOS...${NC}"
                open -a "$VOICEVOX_APP"
            else
                # Homebrewでインストールされた場合の代替パス
                if [ -d "/Applications/VOICEVOX.app" ]; then
                    echo -e "${YELLOW}▶ Launching VOICEVOX.app from Applications...${NC}"
                    open -a "/Applications/VOICEVOX.app"
                else
                    echo -e "${RED}❌ VOICEVOX.app not found${NC}"
                    echo "Please install VOICEVOX from https://voicevox.hiroshiba.jp/"
                    return 1
                fi
            fi
            ;;
        Linux)
            # Linux用VOICEVOX起動 - GPU最適化モードで起動
            VOICEVOX_PATHS=(
                "/opt/voicevox_engine/linux-nvidia/run"
                "$HOME/_data/tools/VOICEVOX/linux-x64/run"
                "/opt/voicevox/run"
                "/usr/local/bin/voicevox"
            )
            
            VOICEVOX_STARTED=false
            for VOICEVOX_PATH in "${VOICEVOX_PATHS[@]}"; do
                if [ -x "$VOICEVOX_PATH" ]; then
                    echo -e "${YELLOW}▶ Launching VOICEVOX in GPU mode from: $VOICEVOX_PATH${NC}"
                    nohup "$VOICEVOX_PATH" --host 127.0.0.1 --port 50021 --use_gpu > /tmp/voicevox_gpu.log 2>&1 &
                    VOICEVOX_PID=$!
                    echo "VOICEVOX PID: $VOICEVOX_PID (GPU optimized)"
                    VOICEVOX_STARTED=true
                    break
                fi
            done
            
            # システムコマンドとしてvoicevoxが利用可能かチェック
            if [ "$VOICEVOX_STARTED" = false ] && command -v voicevox >/dev/null 2>&1; then
                echo -e "${YELLOW}▶ Launching VOICEVOX via system command in GPU mode...${NC}"
                nohup voicevox --host 127.0.0.1 --port 50021 --use_gpu > /tmp/voicevox_gpu.log 2>&1 &
                VOICEVOX_PID=$!
                echo "VOICEVOX PID: $VOICEVOX_PID (GPU optimized)"
                VOICEVOX_STARTED=true
            fi
            
            if [ "$VOICEVOX_STARTED" = false ]; then
                echo -e "${RED}❌ VOICEVOX not found in any of the following locations:${NC}"
                for path in "${VOICEVOX_PATHS[@]}"; do
                    echo "  - $path"
                done
                echo "Please install VOICEVOX:"
                echo "1. Download from https://voicevox.hiroshiba.jp/"
                echo "2. Or install via package manager"
                return 1
            fi
            ;;
        *)
            echo -e "${RED}❌ Unsupported OS: $OS${NC}"
            return 1
            ;;
    esac

    # 起動待機
    echo -e "${YELLOW}⏳ Waiting for VOICEVOX to start...${NC}"
    for i in {1..30}; do
        if curl -fs http://localhost:50021/version >/dev/null 2>&1; then
            echo -e "${GREEN}✅ VOICEVOX started successfully${NC}"
            return 0
        fi
        echo "  Attempt $i/30..."
        sleep 2
    done
    
    echo -e "${RED}❌ VOICEVOX failed to start within 60 seconds${NC}"
    echo -e "${YELLOW}💡 Please try starting VOICEVOX manually${NC}"
    return 1
}

# VOICEVOXチェック・起動
if ! check_voicevox; then
    echo -e "${YELLOW}⚠️  VOICEVOX is not running. Starting automatically...${NC}"
    if start_voicevox; then
        echo -e "${GREEN}✅ VOICEVOX startup completed${NC}"
    else
        echo -e "${YELLOW}⚠️  VOICEVOX automatic startup failed${NC}"
        echo -e "${YELLOW}💡 DiaROS will continue without VOICEVOX (TTS may not work)${NC}"
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo -e "${RED}❌ Exiting...${NC}"
            exit 1
        fi
    fi
fi

echo ""
echo -e "${GREEN}🎯 ROS2パッケージビルドの確認${NC}"
echo "=================================="

# 変更検出のための関数
check_ros_changes() {
    local src_dir="/workspace/DiaROS_ros/src"
    local install_dir="/workspace/DiaROS_ros/install"
    
    # installディレクトリが存在しない場合は初回ビルド
    if [ ! -d "$install_dir" ]; then
        return 1  # 変更あり（初回ビルド）
    fi
    
    # srcディレクトリ内のPythonファイルとinstallディレクトリの更新時刻を比較
    local newest_src=$(find "$src_dir" -name "*.py" -type f -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f1)
    local install_time=$(stat -c %Y "$install_dir" 2>/dev/null || echo "0")
    
    if [ -n "$newest_src" ] && [ "${newest_src%.*}" -gt "$install_time" ]; then
        return 1  # 変更あり
    fi
    
    return 0  # 変更なし
}

# ビルド実行（SDSモジュール編集時は常に実行）
echo -e "${YELLOW}🔨 SDSモジュール編集を反映するため、完全ビルドを実行します...${NC}"

# 1. Pythonモジュールの再インストール
echo -e "${YELLOW}📦 Step 1/3: Pythonモジュールを再インストール中...${NC}"
cd /workspace/DiaROS_py
pip install . --upgrade
echo -e "${GREEN}✅ Pythonモジュールの再インストール完了${NC}"

# 2. ROS2環境セットアップとinterfacesビルド
echo -e "${YELLOW}📦 Step 2/3: interfacesパッケージをビルド中...${NC}"
cd /workspace/DiaROS_ros
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces
. ./install/local_setup.bash
echo -e "${GREEN}✅ interfacesパッケージのビルド完了${NC}"

# 3. diaros_packageのビルド
echo -e "${YELLOW}📦 Step 3/3: diaros_packageをビルド中...${NC}"
colcon build --packages-select diaros_package
. ./install/local_setup.bash
echo -e "${GREEN}✅ diaros_packageのビルド完了${NC}"

echo -e "${GREEN}✅ 完全ビルドプロセス完了${NC}"

echo ""
echo -e "${GREEN}🎯 DiaROSを起動します...${NC}"
echo "=================================="

# ROS2環境の設定
cd /workspace/DiaROS_ros
source /opt/ros/humble/setup.bash

# 既存のビルドがある場合はセットアップスクリプトを実行
if [ -f "./install/local_setup.bash" ]; then
    . ./install/local_setup.bash
    echo -e "${GREEN}✅ ROS2環境を設定しました${NC}"
else
    echo -e "${YELLOW}⚠️  install/local_setup.bashが見つかりません。先にビルドが必要です${NC}"
fi

# NumPy 1.xを強制（aubio互換性のため）
echo -e "${YELLOW}🔧 NumPy互換性の確認...${NC}"
if python3 -c "import numpy" 2>/dev/null; then
    python3 -c "import numpy; print(f'NumPy version: {numpy.__version__}')"
    if python3 -c "import numpy; exit(0 if numpy.__version__.startswith('2.') else 1)" 2>/dev/null; then
        echo -e "${YELLOW}⚠️  NumPy 2.xが検出されました。aubio互換性のためNumPy 1.xにダウングレードします...${NC}"
        pip3 install --force-reinstall "numpy==1.24.3"
        echo -e "${GREEN}✅ NumPy 1.24.3にダウングレードしました${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  NumPyが見つかりません${NC}"
fi

# ROS_DOMAIN_ID固定設定
export ROS_DOMAIN_ID=0
echo -e "${GREEN}📋 ROS設定:${NC}"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""

# 分散実行対応：nlg:=false オプション
NLG_PARAM=""
if [ "$1" = "--nlg-distributed" ] || [ "$1" = "nlg:=false" ]; then
    NLG_PARAM="nlg:=false"
    echo -e "${YELLOW}⚙️  分散実行モード: NLGノードを除外します${NC}"
    echo -e "${YELLOW}📝 NLGPC側で以下を実行してください:${NC}"
    echo "   ros2 run diaros_package ros2_natural_language_generation"
    echo ""
fi

# DiaROSの起動
echo -e "${GREEN}🚀 DiaROSを起動中...${NC}"
ros2 launch diaros_package sdsmod.launch.py $NLG_PARAM