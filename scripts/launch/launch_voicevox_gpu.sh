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

echo "起動パラメータ:"
echo "ONNX_PROVIDERS=$ONNX_PROVIDERS"
echo "CUDA_VISIBLE_DEVICES=$CUDA_VISIBLE_DEVICES"
echo "CUDA_DEVICE_ORDER=$CUDA_DEVICE_ORDER"
echo ""

# VOICEVOX Engine起動（GPU強制、--device削除）
exec /opt/voicevox_engine/linux-nvidia/run \
    --host 0.0.0.0 \
    --port 50021 \
    --use_gpu
