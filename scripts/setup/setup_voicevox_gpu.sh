#!/bin/bash
# VOICEVOX GPU対応セットアップスクリプト（Docker環境用）

echo "🎯 VOICEVOX GPU設定セットアップ"
echo "============================="

# CUDA環境確認
echo "1. CUDA環境確認..."
if command -v nvcc >/dev/null 2>&1; then
    CUDA_VERSION=$(nvcc --version | grep "release" | sed 's/.*release \([0-9.]*\).*/\1/')
    echo "✅ CUDA $CUDA_VERSION が利用可能です"
else
    echo "❌ CUDAが見つかりません"
    exit 1
fi

# GPU確認
echo ""
echo "2. GPU確認..."
if nvidia-smi >/dev/null 2>&1; then
    GPU_INFO=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)
    echo "✅ GPU検出: $GPU_INFO"
else
    echo "❌ GPUが検出されません"
    exit 1
fi

# VOICEVOX Engine設定
echo ""
echo "3. VOICEVOX Engine GPU設定..."

# VOICEVOX Engine実行可能ファイルの検索
VOICEVOX_PATHS=(
    "/opt/voicevox_engine/linux-nvidia/run"
    "/opt/voicevox_engine/linux-gpu/run"
    "/opt/voicevox_engine/run"
    "/usr/local/bin/voicevox_engine"
    "/usr/bin/voicevox_engine"
)

VOICEVOX_BIN=""
for path in "${VOICEVOX_PATHS[@]}"; do
    if [ -x "$path" ]; then
        VOICEVOX_BIN="$path"
        echo "✅ VOICEVOX Engine発見: $path"
        break
    fi
done

if [ -z "$VOICEVOX_BIN" ]; then
    echo "❌ VOICEVOX Engineが見つかりません"
    echo "次のパスを確認してください:"
    printf '%s\n' "${VOICEVOX_PATHS[@]}"
    exit 1
fi

# GPU設定用環境変数
echo ""
echo "4. GPU用環境変数設定..."
export CUDA_VISIBLE_DEVICES=0
export NVIDIA_VISIBLE_DEVICES=0
export VOICEVOX_USE_GPU=1

echo "✅ 環境変数設定完了:"
echo "   CUDA_VISIBLE_DEVICES=$CUDA_VISIBLE_DEVICES"
echo "   NVIDIA_VISIBLE_DEVICES=$NVIDIA_VISIBLE_DEVICES"
echo "   VOICEVOX_USE_GPU=$VOICEVOX_USE_GPU"

# GPU起動用ラッパースクリプト作成
WRAPPER_SCRIPT="/workspace/scripts/launch/launch_voicevox_gpu.sh"
echo ""
echo "5. GPU起動スクリプト作成..."

mkdir -p "$(dirname "$WRAPPER_SCRIPT")"

cat > "$WRAPPER_SCRIPT" << EOF
#!/bin/bash
# VOICEVOX GPU起動ラッパースクリプト

echo "🚀 VOICEVOX GPU起動"
echo "=================="

# GPU環境変数設定
export CUDA_VISIBLE_DEVICES=0
export NVIDIA_VISIBLE_DEVICES=0
export VOICEVOX_USE_GPU=1

echo "VOICEVOX Engine（GPU加速）を起動中..."

# GPU強制設定（ONNX Runtime用）
export ONNX_PROVIDERS="CUDAExecutionProvider,CPUExecutionProvider"
export CUDA_DEVICE_ORDER=PCI_BUS_ID
export OMP_NUM_THREADS=4

# VOICEVOX Engine起動（GPU強制、--device削除）
exec $VOICEVOX_BIN \\
    --host 0.0.0.0 \\
    --port 50021 \\
    --use_gpu
EOF

chmod +x "$WRAPPER_SCRIPT"
echo "✅ GPU起動スクリプト作成: $WRAPPER_SCRIPT"

# テスト用スクリプト作成
TEST_SCRIPT="/workspace/scripts/test/test_voicevox_gpu.sh"
echo ""
echo "6. GPU動作テストスクリプト作成..."

mkdir -p "$(dirname "$TEST_SCRIPT")"

cat > "$TEST_SCRIPT" << 'EOF'
#!/bin/bash
# VOICEVOX GPU動作テストスクリプト

echo "🧪 VOICEVOX GPU動作テスト"
echo "========================"

# 音声合成テスト
echo "音声合成テスト実行中..."
for i in {1..3}; do
    echo "テスト $i/3: 処理中..."

    curl -X POST "localhost:50021/synthesis" \
        -H "Content-Type: application/json" \
        -d "{\"text\":\"テスト番号${i}です。音声合成動作確認中。\",\"speaker\":1}" \
        --output "/tmp/test_${i}.wav" \
        --silent

    if [ $? -eq 0 ]; then
        echo "✅ テスト $i 成功"
    else
        echo "❌ テスト $i 失敗"
    fi

    sleep 1
done

echo ""
echo "✅ テスト完了"
EOF

chmod +x "$TEST_SCRIPT"
echo "✅ GPUテストスクリプト作成: $TEST_SCRIPT"

echo ""
echo "7. セットアップ完了"
echo "=================="
echo ""
echo "📋 次のステップ:"
echo "1. GPU起動: $WRAPPER_SCRIPT"
echo "2. GPU動作テスト: $TEST_SCRIPT"
echo "3. システム起動: ros2 launch diaros_package sdsmod.launch.py"
echo ""
echo "⚠️  注意事項:"
echo "- Docker起動時に --gpus all オプションが必要です"