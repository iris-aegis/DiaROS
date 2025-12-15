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
            # Linux用VOICEVOX起動 - 複数パスを順番に確認
            VOICEVOX_PATHS=(
                "/opt/voicevox_engine/linux-nvidia/run"
                "$HOME/_data/tools/VOICEVOX/linux-x64/run"
                "/opt/voicevox/run"
                "/usr/local/bin/voicevox"
            )
            
            VOICEVOX_STARTED=false
            for VOICEVOX_PATH in "${VOICEVOX_PATHS[@]}"; do
                if [ -x "$VOICEVOX_PATH" ]; then
                    echo -e "${YELLOW}▶ Launching VOICEVOX from: $VOICEVOX_PATH${NC}"
                    nohup "$VOICEVOX_PATH" > /tmp/voicevox.log 2>&1 &
                    VOICEVOX_PID=$!
                    echo "VOICEVOX PID: $VOICEVOX_PID"
                    VOICEVOX_STARTED=true
                    break
                fi
            done
            
            # システムコマンドとしてvoicevoxが利用可能かチェック
            if [ "$VOICEVOX_STARTED" = false ] && command -v voicevox >/dev/null 2>&1; then
                echo -e "${YELLOW}▶ Launching VOICEVOX via system command...${NC}"
                nohup voicevox > /tmp/voicevox.log 2>&1 &
                VOICEVOX_PID=$!
                echo "VOICEVOX PID: $VOICEVOX_PID"
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

# Ollama起動確認・自動起動機能
echo ""
echo -e "${GREEN}🤖 Ollama起動確認${NC}"
echo "=================================="

check_ollama() {
    if curl -fs http://localhost:11434/api/version >/dev/null 2>&1; then
        echo -e "${GREEN}✅ Ollama is already running${NC}"
        return 0
    else
        return 1
    fi
}

if ! check_ollama; then
    echo -e "${YELLOW}⚠️  Ollama is not running. Starting Ollama...${NC}"
    
    # Ollama GPU設定の最適化
    export OLLAMA_GPU_OVERHEAD=0
    export OLLAMA_NUM_PARALLEL=2  # 並列推論サポート
    export OLLAMA_MAX_LOADED_MODELS=2  # 最大2モデル同時ロード
    export OLLAMA_KEEP_ALIVE=10m  # モデルをメモリに10分間保持
    export OLLAMA_HOST=0.0.0.0:11434
    export OLLAMA_ORIGINS="*"
    
    # GPU使用の強制設定
    export CUDA_VISIBLE_DEVICES=0  # GPU 0を使用
    export OLLAMA_LLM_LIBRARY=cuda
    
    echo -e "${GREEN}🖥️  GPU最適化設定を適用:${NC}"
    echo "  - OLLAMA_NUM_PARALLEL=2 (並列推論)"
    echo "  - OLLAMA_MAX_LOADED_MODELS=2 (同時ロード)"
    echo "  - OLLAMA_KEEP_ALIVE=10m (メモリ保持)"
    echo "  - CUDA_VISIBLE_DEVICES=0 (GPU強制使用)"
    
    ollama serve &
    OLLAMA_PID=$!
    echo "Ollama PID: $OLLAMA_PID"
    
    # 起動待機
    echo -e "${YELLOW}⏳ Waiting for Ollama to start...${NC}"
    for i in {1..30}; do
        if curl -fs http://localhost:11434/api/version >/dev/null 2>&1; then
            echo -e "${GREEN}✅ Ollama started successfully${NC}"
            break
        fi
        echo "  Attempt $i/30..."
        sleep 2
    done

    # 最終確認
    if ! curl -fs http://localhost:11434/api/version >/dev/null 2>&1; then
        echo -e "${RED}❌ Ollama failed to start within 60 seconds${NC}"
        echo -e "${YELLOW}💡 Please try starting Ollama manually: ollama serve${NC}"
    fi
else
    echo -e "${GREEN}✅ Ollama is already running${NC}"
    
    # 既に実行中でもGPU設定を適用
    export OLLAMA_GPU_OVERHEAD=0
    export OLLAMA_NUM_PARALLEL=2
    export OLLAMA_MAX_LOADED_MODELS=2
    export OLLAMA_KEEP_ALIVE=10m
    export CUDA_VISIBLE_DEVICES=0
    export OLLAMA_LLM_LIBRARY=cuda
fi

echo ""
echo -e "${GREEN}🎯 ビルドとインストール${NC}"
echo "=================================="

# Pythonモジュールのインストール（常に実行）
echo -e "${YELLOW}🔍 Pythonモジュールの更新...${NC}"
cd /workspace/DiaROS_py
pip install . --upgrade
echo -e "${GREEN}✅ モジュールのインストールが完了しました${NC}"

# ROS2パッケージのビルド（常に実行）
echo -e "${YELLOW}🔨 ROS2パッケージをビルドします...${NC}"
cd /workspace/DiaROS_ros
source /opt/ros/humble/setup.bash

# interfacesパッケージのビルド
echo -e "${YELLOW}📦 interfacesパッケージをビルド中...${NC}"
colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces
. ./install/local_setup.bash

# diaros_packageのビルド
echo -e "${YELLOW}📦 diaros_packageをビルド中...${NC}"
colcon build --packages-select diaros_package
. ./install/local_setup.bash

echo -e "${GREEN}✅ ROS2パッケージのビルドが完了しました${NC}"

echo ""
echo -e "${GREEN}🎯 DiaROSを起動します...${NC}"
echo "=================================="

# ROS2環境の設定
cd /workspace/DiaROS_ros
source /opt/ros/humble/setup.bash

# セットアップスクリプトを実行
. ./install/local_setup.bash
echo -e "${GREEN}✅ ROS2環境を設定しました${NC}"

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

# DiaROSの起動
echo -e "${GREEN}🚀 DiaROSを起動中...${NC}"
ros2 launch diaros_package sdsmod.launch.py