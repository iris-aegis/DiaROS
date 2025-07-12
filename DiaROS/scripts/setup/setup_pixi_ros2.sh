#!/bin/bash
# Pixiを使ってROS2 HumbleをM1/M2/M3 Macにセットアップするスクリプト

set -e

# 色付き出力
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}🚀 Pixi ROS2 Setup for Apple Silicon Mac${NC}"
echo "========================================"

# 1. Pixiがインストールされているか確認
echo -e "\n${YELLOW}1. Pixiのインストール確認${NC}"
if command -v pixi &> /dev/null; then
    echo -e "${GREEN}✅ Pixiが既にインストールされています: $(pixi --version)${NC}"
else
    echo -e "${YELLOW}Pixiをインストールします...${NC}"
    curl -fsSL https://pixi.sh/install.sh | bash
    
    # パスを追加
    if [[ "$SHELL" == *"zsh"* ]]; then
        echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.zshrc
        source ~/.zshrc
    else
        echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.bashrc
        source ~/.bashrc
    fi
    
    echo -e "${GREEN}✅ Pixiのインストールが完了しました${NC}"
fi

# 2. プロジェクトディレクトリの作成
echo -e "\n${YELLOW}2. DiaROS Pixiプロジェクトの作成${NC}"
PIXI_DIR="$HOME/DiaROS_pixi"
if [ -d "$PIXI_DIR/diaros_workspace" ]; then
    echo -e "${YELLOW}既存のプロジェクトが見つかりました${NC}"
    read -p "削除して再作成しますか？ (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$PIXI_DIR/diaros_workspace"
    else
        echo -e "${GREEN}既存のプロジェクトを使用します${NC}"
        cd "$PIXI_DIR/diaros_workspace"
        exit 0
    fi
fi

mkdir -p "$PIXI_DIR"
cd "$PIXI_DIR"

# 3. Pixiプロジェクトの初期化
echo -e "\n${YELLOW}3. Pixiプロジェクトの初期化${NC}"
pixi init diaros_workspace -c robostack-humble -c conda-forge
cd diaros_workspace

# 4. ROS2 Humbleのインストール
echo -e "\n${YELLOW}4. ROS2 Humbleのインストール（時間がかかります）${NC}"
echo "インストールするパッケージ:"
echo "  - ros-humble-desktop"
echo "  - colcon-common-extensions"
echo "  - python=3.9 (ROS2 Humbleの要件)"
echo ""
read -p "続行しますか？ (Y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Nn]$ ]]; then
    pixi add python=3.9 ros-humble-desktop colcon-common-extensions pip
    echo -e "${GREEN}✅ ROS2 Humbleのインストールが完了しました${NC}"
fi

# 5. DiaROSのクローン
echo -e "\n${YELLOW}5. DiaROSリポジトリのクローン${NC}"
cd "$PIXI_DIR"
if [ ! -d "DiaROS_imamoto" ]; then
    git clone https://github.com/sayonari/DiaROS_imamoto.git
    echo -e "${GREEN}✅ DiaROSのクローンが完了しました${NC}"
else
    echo -e "${GREEN}✅ DiaROSは既にクローンされています${NC}"
fi

# 6. 実行スクリプトの作成
echo -e "\n${YELLOW}6. 実行スクリプトの作成${NC}"
cat > "$PIXI_DIR/run_diaros_pixi.sh" << 'EOF'
#!/bin/bash
# DiaROSをPixi環境で実行するスクリプト

cd "$(dirname "$0")/diaros_workspace"

echo "🚀 Pixi環境でDiaROSを起動します"
echo "================================"
echo ""
echo "使用方法:"
echo "  1. このスクリプトを実行するとPixi環境に入ります"
echo "  2. 別ターミナルでVOICEVOXを起動してください"
echo "  3. 以下のコマンドでDiaROSをビルド・実行できます:"
echo ""
echo "  # ビルド"
echo "  cd ~/_data/_DiaROS_mac/DiaROS_pixi/DiaROS_imamoto/DiaROS_ros"
echo "  colcon build --packages-select interfaces"
echo "  source install/local_setup.bash"
echo "  colcon build --packages-select diaros_package"
echo ""
echo "  # 実行"
echo "  export DIAROS_DEVICE=mps"
echo "  ros2 launch diaros_package sdsmod.launch.py"
echo ""
echo "Pixi環境に入ります..."
echo ""

exec pixi shell
EOF

chmod +x "$PIXI_DIR/run_diaros_pixi.sh"

# 7. 完了メッセージ
echo -e "\n${GREEN}🎉 セットアップが完了しました！${NC}"
echo -e "\n次のステップ:"
echo -e "1. ${YELLOW}cd $PIXI_DIR/diaros_workspace${NC}"
echo -e "2. ${YELLOW}pixi shell${NC} でPixi環境に入る"
echo -e "3. Python依存パッケージをインストール:"
echo -e "   ${YELLOW}pip install torch transformers pyaudio numpy==1.24.3${NC}"
echo -e "   ${YELLOW}# aubioのインストール（コンパイラエラーが出る場合）:${NC}"
echo -e "   ${YELLOW}brew install aubio${NC}"
echo -e "   ${YELLOW}export CFLAGS=\"-Wno-error=incompatible-function-pointer-types\"${NC}"
echo -e "   ${YELLOW}pip install aubio --no-cache-dir${NC}"
echo -e "4. DiaROSをビルド・実行"
echo -e "\nまたは、以下を実行:"
echo -e "${YELLOW}$PIXI_DIR/run_diaros_pixi.sh${NC}"