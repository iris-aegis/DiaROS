#!/usr/bin/env python3
"""
Gemmaモデル（gemma3:4b等）をOllamaを使用せずに
Transformersライブラリを用いて直接推論するテストコード

要件:
- transformers
- torch
- accelerate (推奨)

実行方法:
python3 test_gemma_native_inference.py
"""

import torch
import time
from transformers import AutoTokenizer, AutoModelForCausalLM
import sys
import traceback
import transformers.modeling_utils

# ==========================================
# 設定
# ==========================================

MODEL_ID = "google/gemma-3-4b-it" 
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
MAX_NEW_TOKENS = 256
TEMPERATURE = 0.7
TOP_P = 0.9
PROMPT_TEXT = "こんにちは、今日の天気について教えてください。"

# ==========================================
# モンキーパッチ (バグ回避)
# ==========================================

# transformersのバグ回避: ALL_PARALLEL_STYLESが未定義、または不足している場合がある
# Gemma 3は colwise / rowwise を使用する可能性があるため追加
if not hasattr(transformers.modeling_utils, "ALL_PARALLEL_STYLES") or transformers.modeling_utils.ALL_PARALLEL_STYLES is None:
    print("[WARN] Patching ALL_PARALLEL_STYLES (init)...")
    transformers.modeling_utils.ALL_PARALLEL_STYLES = []

# 既存のリストに必要なスタイルを追加
required_styles = ["sequence_parallel", "pipeline_parallel", "tensor_parallel", "colwise", "rowwise"]
for style in required_styles:
    if style not in transformers.modeling_utils.ALL_PARALLEL_STYLES:
        print(f"[WARN] Adding '{style}' to ALL_PARALLEL_STYLES")
        transformers.modeling_utils.ALL_PARALLEL_STYLES.append(style)

# ==========================================
# 処理
# ==========================================

def main():
    print(f"=== Gemma Native Inference Test (Patched v2) ===")
    print(f"Device: {DEVICE}")
    print(f"Model ID: {MODEL_ID}")
    
    # 1. トークナイザーのロード
    print("\n[INFO] Loading tokenizer...")
    start_time = time.time()
    try:
        tokenizer = AutoTokenizer.from_pretrained(MODEL_ID)
    except Exception as e:
        print(f"[ERROR] Failed to load tokenizer: {e}")
        traceback.print_exc()
        sys.exit(1)
    print(f"[INFO] Tokenizer loaded in {time.time() - start_time:.2f}s")

    # 2. モデルのロード
    print("\n[INFO] Loading model...")
    start_time = time.time()
    try:
        model = AutoModelForCausalLM.from_pretrained(
            MODEL_ID,
            torch_dtype=torch.bfloat16 if DEVICE == "cuda" else torch.float32,
            low_cpu_mem_usage=True,
            attn_implementation="eager"
        )
        model.to(DEVICE)
    except Exception as e:
        print(f"[ERROR] Failed to load model: {e}")
        print("\n--- Traceback ---")
        traceback.print_exc()
        print("-----------------")
        sys.exit(1)
    print(f"[INFO] Model loaded in {time.time() - start_time:.2f}s")

    # 3. 入力処理
    print(f"\n[INFO] Processing input...")
    messages = [
        {"role": "user", "content": PROMPT_TEXT},
    ]
    
    input_ids = tokenizer.apply_chat_template(
        messages,
        add_generation_prompt=True,
        return_tensors="pt"
    ).to(model.device)

    print(f"Input prompt: {PROMPT_TEXT}")
    print(f"Input tokens: {input_ids.shape[1]}")

    # 4. 生成実行
    print("\n[INFO] Generating response...")
    start_time = time.time()
    
    terminators = [
        tokenizer.eos_token_id,
        tokenizer.convert_tokens_to_ids("<end_of_turn>")
    ]

    with torch.no_grad():
        outputs = model.generate(
            input_ids,
            max_new_tokens=MAX_NEW_TOKENS,
            temperature=TEMPERATURE,
            top_p=TOP_P,
            do_sample=True,
            eos_token_id=terminators,
            pad_token_id=tokenizer.eos_token_id
        )
    
    generation_time = time.time() - start_time
    
    # 5. 出力デコード
    response = outputs[0][input_ids.shape[-1]:]
    decoded_response = tokenizer.decode(response, skip_special_tokens=True)
    
    print("\n" + "="*30)
    print("RESPONSE")
    print("="*30)
    print(decoded_response)
    print("="*30)
    
    # 統計
    new_tokens = len(response)
    tokens_per_sec = new_tokens / generation_time
    
    print(f"\n[STATS]")
    print(f"Generation time: {generation_time:.2f}s")
    print(f"Generated tokens: {new_tokens}")
    print(f"Speed: {tokens_per_sec:.2f} tokens/sec")

if __name__ == "__main__":
    main()
