#!/usr/bin/env python3
"""
Gemmaモデル（gemma3:4b等）を用いた
生成中断(Interrupt)と切り替え(Switching)のオーバーヘッド計測テスト (Streamer版)

修正点:
StoppingCriteriaが効かないため、TextIteratorStreamerを用いて
メインスレッド側でトークン受信ループを回し、時間経過で強制的に
「次の処理（Second Stage）」へ移行する方式に変更。
"""

import torch
import time
import sys
import traceback
import threading
from transformers import (
    AutoTokenizer,
    AutoModelForCausalLM,
    TextIteratorStreamer
)
import transformers.modeling_utils

# ==========================================
# 設定
# ==========================================

MODEL_ID = "google/gemma-3-4b-it"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# 割り込み設定
INTERRUPT_TIMING_SEC = 0.5  # 500msで中断

# プロンプト
PROMPT_1_LONG = "日本の歴史について、古代から現代まで詳細に、少なくとも2000文字以上で小説風に語ってください。"
PROMPT_2_SHORT = "あ、すみません。今の時間は何時ですか？"

# ==========================================
# モンキーパッチ
# ==========================================
if not hasattr(transformers.modeling_utils, "ALL_PARALLEL_STYLES") or transformers.modeling_utils.ALL_PARALLEL_STYLES is None:
    transformers.modeling_utils.ALL_PARALLEL_STYLES = []

required_styles = ["sequence_parallel", "pipeline_parallel", "tensor_parallel", "colwise", "rowwise"]
for style in required_styles:
    if style not in transformers.modeling_utils.ALL_PARALLEL_STYLES:
        transformers.modeling_utils.ALL_PARALLEL_STYLES.append(style)

def main():
    print(f"=== Gemma Interrupt Overhead Test (Streamer Based) ===")
    print(f"Device: {DEVICE}")
    print(f"Model: {MODEL_ID}")
    print(f"Interrupt Target: {INTERRUPT_TIMING_SEC * 1000} ms")

    # 1. ロード
    try:
        print("\n[INFO] Loading Tokenizer...")
        tokenizer = AutoTokenizer.from_pretrained(MODEL_ID)
        
        print("[INFO] Loading Model...")
        model = AutoModelForCausalLM.from_pretrained(
            MODEL_ID,
            torch_dtype=torch.bfloat16 if DEVICE == "cuda" else torch.float32,
            low_cpu_mem_usage=True,
            attn_implementation="eager"
        )
        model.to(DEVICE)
    except Exception as e:
        print(f"[ERROR] Load failed: {e}")
        traceback.print_exc()
        sys.exit(1)

    # 2. ウォームアップ
    print("\n[INFO] Warming up...")
    input_ids_warmup = tokenizer.encode("Warmup", return_tensors="pt").to(DEVICE)
    model.generate(input_ids_warmup, max_new_tokens=10)
    print("[INFO] Warmup done.")

    # ==========================================
    # テスト実行
    # ==========================================
    print("\n" + "="*50)
    print(" STARTING LATENCY TEST")
    print("="*50)

    # --- Stage 1: Long Generation (Background Thread) ---
    messages1 = [{"role": "user", "content": PROMPT_1_LONG}]
    input_ids1 = tokenizer.apply_chat_template(messages1, add_generation_prompt=True, return_tensors="pt").to(DEVICE)
    
    streamer1 = TextIteratorStreamer(tokenizer, skip_prompt=True, skip_special_tokens=True)
    generation_kwargs1 = dict(
        input_ids=input_ids1,
        streamer=streamer1,
        max_new_tokens=512,
        do_sample=True,
        temperature=0.7,
    )
    
    # 時間計測開始
    t_start = time.time()
    t_target_interrupt = t_start + INTERRUPT_TIMING_SEC
    
    print(f"[Stage 1] Start generating... (Target Interrupt at +{INTERRUPT_TIMING_SEC*1000:.1f}ms)")
    
    # 別スレッドで生成開始
    thread1 = threading.Thread(target=model.generate, kwargs=generation_kwargs1)
    thread1.start()
    
    # メインスレッドで監視
    # 500ms経過するまでトークンを受け取る（あるいは待機する）
    interrupted = False
    for new_text in streamer1:
        current_time = time.time()
        # 経過時間をチェック
        if current_time >= t_target_interrupt:
            print(f"[Stage 1] Interrupt triggered! Time elapsed: {(current_time - t_start)*1000:.2f}ms")
            interrupted = True
            break # ここでループを抜ける＝「中断」とみなして次へ進む
        
        # まだ時間内なら何もしない（出力を表示してもよい）
        # sys.stdout.write(new_text)
        # sys.stdout.flush()

    if not interrupted:
        print("[WARN] Stage 1 finished naturally before interrupt time. (Model too fast?)")

    # ループを抜けた時刻＝「中断」を決定した時刻
    t_interrupt_decision = time.time()
    
    # 注: thread1 はまだ裏で走っている可能性があります。
    # 厳密なリソース解放のためにはプロセス終了が必要ですが、
    # Pythonのスレッドでは強制killができないため、ここでは
    # 「アプリケーションが次の処理（Stage 2）に移る」までの時間を計測します。
    # ただし、GPUリソースが競合するため、Stage 2の開始が遅れる可能性があります。
    # それも含めて「オーバーヘッド」として計測します。

    # --- Stage 2: Second Generation (Switching) ---
    print(f"[Stage 2] Requesting new generation immediately...")
    
    messages2 = [{"role": "user", "content": PROMPT_2_SHORT}]
    input_ids2 = tokenizer.apply_chat_template(messages2, add_generation_prompt=True, return_tensors="pt").to(DEVICE)

    streamer2 = TextIteratorStreamer(tokenizer, skip_prompt=True, skip_special_tokens=True)
    generation_kwargs2 = dict(
        input_ids=input_ids2,
        streamer=streamer2,
        max_new_tokens=128,
        do_sample=True,
        temperature=0.7,
    )
    
    t_stage2_call = time.time()
    
    # Stage 2開始
    # GPUがStage 1で占有されている場合、ここでの待ちが発生するはず
    thread2 = threading.Thread(target=model.generate, kwargs=generation_kwargs2)
    thread2.start()
    
    first_token_text = ""
    t_stage2_first_token = None
    
    print(f"[Stage 2] Waiting for first token...")
    for new_text in streamer2:
        if t_stage2_first_token is None:
            t_stage2_first_token = time.time()
            first_token_text = new_text
            print(f"[Stage 2] First token received: '{new_text.strip()}'")
        break

    # ==========================================
    # 結果計算
    # ==========================================
    
    latency_decision_lag = (t_interrupt_decision - t_target_interrupt) * 1000
    latency_switch_overhead = (t_stage2_call - t_interrupt_decision) * 1000
    latency_stage2_prefill = (t_stage2_first_token - t_stage2_call) * 1000
    latency_total_switch = (t_stage2_first_token - t_target_interrupt) * 1000
    
    print("\n" + "="*50)
    print(" RESULT REPORT (Streamer Based)")
    print("="*50)
    print(f"1. Interrupt Target Time : 0.0 ms (Reference)")
    print(f"2. Decision/Break Time   : +{latency_decision_lag:.2f} ms (Late by {(t_interrupt_decision - t_target_interrupt)*1000:.2f} ms)")
    print(f"3. Stage 2 Call Time     : +{(t_stage2_call - t_target_interrupt)*1000:.2f} ms")
    print(f"4. First Token Time      : +{(t_stage2_first_token - t_target_interrupt)*1000:.2f} ms")
    print("-" * 30)
    print(f"★ Total Switching Latency: {latency_total_switch:.2f} ms")
    print("-" * 30)
    print("Breakdown:")
    print(f" - Decision Lag (Wait for next token check) : {latency_decision_lag:.2f} ms")
    print(f" - Python Overhead                          : {latency_switch_overhead:.2f} ms")
    print(f" - Stage 2 Pre-fill (Wait for GPU free)     : {latency_stage2_prefill:.2f} ms")
    print("   (Note: 'Stage 2 Pre-fill' includes waiting time if Stage 1 is still occupying GPU)")

if __name__ == "__main__":
    main()
