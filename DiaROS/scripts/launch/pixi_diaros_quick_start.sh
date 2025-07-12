#!/bin/bash
# Pixi環境でDiaROSを素早く起動するためのスクリプト

set -e

# 色付き出力
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}🚀 DiaROS Quick Start Script for Pixi Environment${NC}"
echo "=================================================="

# ディレクトリの確認
PIXI_DIR="$HOME/DiaROS_pixi"
if [ ! -d "$PIXI_DIR/diaros_workspace" ]; then
    echo -e "${RED}❌ Pixi workspace not found at $PIXI_DIR/diaros_workspace${NC}"
    echo "Please run setup_pixi_ros2.sh first"
    exit 1
fi

if [ ! -d "$PIXI_DIR/DiaROS_imamoto" ]; then
    echo -e "${RED}❌ DiaROS not found at $PIXI_DIR/DiaROS_imamoto${NC}"
    echo "Please clone DiaROS first"
    exit 1
fi

# VOICEVOXの確認
echo -e "\n${YELLOW}Checking VOICEVOX...${NC}"
if curl -s http://localhost:50021/version > /dev/null 2>&1; then
    echo -e "${GREEN}✅ VOICEVOX is running${NC}"
else
    echo -e "${YELLOW}⚠️  VOICEVOX is not running${NC}"
    echo "Please start VOICEVOX in another terminal:"
    echo "  cd ~/Downloads/macos-x64 && ./run"
    read -p "Press Enter when VOICEVOX is running..."
fi

# スクリプトの作成
cat > "$PIXI_DIR/launch_diaros.sh" << 'EOF'
#!/bin/bash
# DiaROSを起動するスクリプト

# 色付き出力
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}Starting DiaROS in Pixi environment...${NC}"

# ディレクトリ設定
PIXI_DIR="$HOME/DiaROS_pixi"
cd "$PIXI_DIR/diaros_workspace"

# Pixi環境で実行
pixi shell << 'PIXI_SCRIPT'
# DiaROSディレクトリに移動
cd ~/_data/_DiaROS_mac/DiaROS_pixi/DiaROS_imamoto/DiaROS_ros

# 環境変数の設定
echo "Setting up environment variables..."
export DIAROS_DEVICE=mps
export ROS_DISTRO=humble
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3

# AMENTとPYTHONPATHの設定
export AMENT_PREFIX_PATH=$PWD/install/diaros_package:$PWD/install/interfaces:$AMENT_PREFIX_PATH
export PYTHONPATH=$PWD/install/diaros_package/lib/python3.9/site-packages:$PWD/install/interfaces/lib/python3.9/site-packages:$PYTHONPATH

# 動的ライブラリパスの設定
export DYLD_LIBRARY_PATH=$PWD/install/interfaces/lib:$DYLD_LIBRARY_PATH

# デバイス情報の表示
echo ""
echo "Device configuration:"
echo "  DIAROS_DEVICE: $DIAROS_DEVICE (Apple Silicon GPU)"
python -c "import torch; print(f'  MPS available: {torch.backends.mps.is_available()}')"
echo ""

# HuggingFaceトークンの確認
if [ -z "$HF_TOKEN" ]; then
    echo "⚠️  HF_TOKEN is not set. Some models may not be accessible."
    echo "   Set it with: export HF_TOKEN=your_token_here"
    echo "   Or login with: huggingface-cli login"
fi

# power_calibration.wavファイルの確認
if [ ! -f "power_calibration.wav" ]; then
    echo "⚠️  power_calibration.wav not found in current directory."
    if [ -f "../DiaROS_py/diaros/power_calibration.wav" ]; then
        echo "   Copying from DiaROS_py..."
        cp ../DiaROS_py/diaros/power_calibration.wav .
    else
        echo "   Please ensure power_calibration.wav is available."
    fi
fi

# 必須Pythonパッケージの確認
echo ""
echo "Checking required Python packages..."
python -c "import playsound" 2>/dev/null || echo "⚠️  playsound not installed. Run: pip install playsound"
python -c "import pydub" 2>/dev/null || echo "⚠️  pydub not installed. Run: pip install pydub"
python -c "import PyObjC" 2>/dev/null || echo "⚠️  PyObjC not installed. Run: pip install PyObjC"

# DiaROSの起動
echo "Launching DiaROS..."
ros2 launch diaros_package sdsmod.launch.py
PIXI_SCRIPT
EOF

chmod +x "$PIXI_DIR/launch_diaros.sh"

echo -e "\n${GREEN}✅ Quick start script created!${NC}"
echo -e "\nTo launch DiaROS, run:"
echo -e "${YELLOW}$PIXI_DIR/launch_diaros.sh${NC}"
echo -e "\nMake sure:"
echo -e "1. VOICEVOX is running"
echo -e "2. HF_TOKEN is set (if needed)"
echo -e "3. Microphone permissions are granted"