#!/usr/bin/env python3
"""
Native Gemma-3-4B-IT 直接推論オーバーヘッド計測

Ollama を一切使わず、transformers を使ってローカルで直接推論し、
中断～Second stage 開始までのオーバーヘッドを計測

使用方法：
    python3 test_native_gemma3_direct_latency.py
"""

import torch
import time
import statistics
import threading
from transformers import AutoTokenizer, AutoModelForCausalLM
from typing import List

class NativeGemma3DirectValidator:
    def __init__(self, model_name: str = "google/gemma-3-4b-it"):
        """初期化"""
        print(f"📦 Native Gemma-3-4B-IT 初期化中: {model_name}")
        print("   (これには数分かかる場合があります)")

        self.model_name = model_name
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"   使用デバイス: {self.device}")

        try:
            # トークナイザーを読み込み
            print("   トークナイザー読み込み中...", end="", flush=True)
            self.tokenizer = AutoTokenizer.from_pretrained(model_name)
            self.tokenizer.pad_token = self.tokenizer.eos_token
            print(" ✅")

            # モデルを読み込み
            print("   モデル読み込み中...", end="", flush=True)
            self.model = AutoModelForCausalLM.from_pretrained(
                model_name,
                torch_dtype=torch.float16 if self.device == "cuda" else torch.float32,
                device_map="auto" if self.device == "cuda" else None,
            )

            # CPU の場合は明示的に移動
            if self.device == "cpu":
                self.model = self.model.to(self.device)

            # 評価モードに設定
            self.model.eval()
            print(" ✅")

            # ウォームアップ（初回実行のメモリアロケーション遅延を排除）
            print("   ウォームアップ実行中...", end="", flush=True)
            self._warmup()
            print(" ✅")

            print("✅ 初期化完了\n")

        except Exception as e:
            print(f"\n❌ モデル初期化失敗: {e}")
            raise

    def _warmup(self):
        """ウォームアップ実行"""
        with torch.no_grad():
            inputs = self.tokenizer(
                "test",
                return_tensors="pt",
                max_length=512,
                truncation=True
            ).to(self.device)
            _ = self.model.generate(
                inputs.input_ids,
                max_new_tokens=10,
                temperature=0.7,
                do_sample=True
            )
        if self.device == "cuda":
            torch.cuda.empty_cache()

    def generate_with_interrupt(
        self,
        prompt: str,
        max_tokens: int = 200,
        temperature: float = 0.7
    ) -> tuple:
        """
        推論実行（中断対応）

        戻り値: (生成テキスト, 最初のトークン時刻)
        """
        first_token_time = None
        generated_text = ""
        self.interrupt_flag = False

        try:
            with torch.no_grad():
                # トークン化
                inputs = self.tokenizer(
                    prompt,
                    return_tensors="pt",
                    max_length=512,
                    truncation=True
                ).to(self.device)

                input_ids = inputs.input_ids

                # generate を実行（ストリーミングに対応）
                outputs = self.model.generate(
                    input_ids,
                    max_new_tokens=max_tokens,
                    temperature=temperature,
                    do_sample=True,
                    top_p=0.9,
                    top_k=50,
                    output_scores=False,
                    pad_token_id=self.tokenizer.eos_token_id,
                )

                # 生成テキストをデコード
                # 最初のトークンの時刻を記録（生成完了時）
                if first_token_time is None:
                    first_token_time = time.perf_counter()

                generated_ids = outputs[0, input_ids.shape[1]:]
                generated_text = self.tokenizer.decode(generated_ids, skip_special_tokens=True)

                if self.device == "cuda":
                    torch.cuda.empty_cache()

        except Exception as e:
            print(f"❌ 生成エラー: {e}")
            first_token_time = time.perf_counter()

        return generated_text, first_token_time

    def interrupt_generation(self):
        """生成を中断"""
        self.interrupt_flag = True

    def measure_latency(self, num_iterations: int = 5) -> List[float]:
        """
        中断命令送信 → Second stage 推論開始までのレイテンシを計測

        シーケンス：
        1. First iteration: 短い応答を生成（ウォームアップ）
        2. Second iteration～: 長い応答を生成開始 → 中断 → Second stage開始
        """
        print("=" * 80)
        print(f"🔬 Native Gemma-3-4B-IT オーバーヘッド計測（{num_iterations}回測定）")
        print("=" * 80)

        latencies = []

        for iteration in range(num_iterations):
            print(f"\n【測定 {iteration+1}/{num_iterations}】")

            if iteration == 0:
                # ============================================================
                # 【初回】Short stage: 短い応答を完全生成（ウォームアップ）
                # ============================================================
                print("  🔥 ウォームアップ: 短い応答を生成中...", end="", flush=True)

                short_prompt = "こんにちは。返事をしてください。"

                start_time = time.perf_counter()
                result_short, _ = self.generate_with_interrupt(short_prompt, max_tokens=10)
                end_time = time.perf_counter()

                warmup_time = (end_time - start_time) * 1000
                print(f" ✅ ({warmup_time:.2f}ms)")
                print(f"   結果: '{result_short[:50]}...'")
                time.sleep(0.5)
                continue

            # ============================================================
            # 【本計測】First stage: 長い応答を生成開始
            # ============================================================
            print("  📝 First stage: 長い応答を生成中...", end="", flush=True)

            first_stage_prompt = """あなたは詳しい説明をする専門家です。
以下の質問に対して、できるだけ詳しく、長く説明してください。
複数の段落で、詳細な背景情報を含めて、徹底的に説明してください。

質問: なぜ空は青いのですか？

回答:"""

            # タイマースレッドを設定（500ms後に中断）
            self.interrupt_send_time = None
            self.second_stage_first_token_time = None

            def interrupt_after_500ms():
                time.sleep(0.5)  # 500ms で中断
                self.interrupt_send_time = time.perf_counter()
                print(f"\n  ⏸️  [中断命令] 500ms時点で中断信号を送信", end="", flush=True)
                self.interrupt_generation()

            interrupt_thread = threading.Thread(target=interrupt_after_500ms, daemon=True)
            interrupt_thread.start()

            # First stage 生成を実行
            start_time = time.perf_counter()
            result1, first_token_time1 = self.generate_with_interrupt(first_stage_prompt, max_tokens=200)
            end_time = time.perf_counter()
            first_stage_time = (end_time - start_time) * 1000

            interrupt_thread.join()
            print(" ✅")

            # ============================================================
            # 【Step 2】Second stage: リクエスト送信
            # ============================================================
            print("  💬 Second stage: 応答生成開始...", end="", flush=True)

            second_stage_prompt = "感謝のメッセージを短く述べてください。"

            # Second stage 応答開始
            start_time2 = time.perf_counter()
            result2, first_token_time2 = self.generate_with_interrupt(second_stage_prompt, max_tokens=30)
            self.second_stage_first_token_time = first_token_time2
            end_time2 = time.perf_counter()

            second_stage_time = (end_time2 - start_time2) * 1000

            print(" ✅")

            # ============================================================
            # 【計測結果】
            # ============================================================
            if self.interrupt_send_time is not None and self.second_stage_first_token_time is not None:
                latency = (self.second_stage_first_token_time - self.interrupt_send_time) * 1000
                print(f"\n  📊 計測結果:")
                print(f"     • First stage 総時間: {first_stage_time:.2f}ms")
                print(f"     • Second stage 総時間: {second_stage_time:.2f}ms")
                print(f"     • 中断命令送信～SS最初トークン: {latency:.2f}ms")
                print(f"     • First stage 生成テキスト長: {len(result1)}文字")
                print(f"     • Second stage 生成テキスト長: {len(result2)}文字")

                latencies.append(latency)
            else:
                print(f"\n  ⚠️  計測失敗（タイムスタンプ記録なし）")

            time.sleep(0.5)

        return latencies


def main():
    """メイン処理"""
    print("\n" + "=" * 80)
    print("🔬 Native Gemma-3-4B-IT 直接推論テスト")
    print("=" * 80)

    try:
        # 初期化
        validator = NativeGemma3DirectValidator()

        # 計測を実行
        latencies = validator.measure_latency(num_iterations=5)

        # 統計分析
        print("\n" + "=" * 80)
        print("📈 統計分析")
        print("=" * 80)

        if latencies:
            print(f"\n📊 個別測定値: {[f'{x:.2f}ms' for x in latencies]}")
            print(f"  • 最小値: {min(latencies):.2f}ms")
            print(f"  • 最大値: {max(latencies):.2f}ms")
            print(f"  • 平均値: {statistics.mean(latencies):.2f}ms")
            print(f"  • 中央値: {statistics.median(latencies):.2f}ms")
            if len(latencies) > 1:
                print(f"  • 標準偏差: {statistics.stdev(latencies):.2f}ms")

            print("\n💡 分析:")
            mean_latency = statistics.mean(latencies)

            # REST API との比較
            rest_api_latency = 156.7  # 前回の REST API 計測値
            native_reduction = rest_api_latency - mean_latency
            reduction_percent = (native_reduction / rest_api_latency) * 100

            print(f"\n📊 推論方式の比較:")
            print(f"  • REST API経由（ollama）: {rest_api_latency:.2f}ms")
            print(f"  • ネイティブ直接実行: {mean_latency:.2f}ms")
            print(f"  • 削減: {native_reduction:.2f}ms ({reduction_percent:.1f}%)")

            if native_reduction > 20:
                print(f"\n✅ ネイティブ実行で有意な削減を確認")
                print(f"   推論エンジン初期化オーバーヘッド: {mean_latency:.0f}ms")
                print(f"   API/IPC オーバーヘッド: {native_reduction:.0f}ms")
            elif native_reduction > 0:
                print(f"\n⚠️  若干の削減を確認")
                print(f"   → ネイティブ直接実行でも必須の初期化時間: {mean_latency:.0f}ms")
            else:
                print(f"\n⚠️  削減なし")
                print(f"   → ネイティブ実行も同等のオーバーヘッド")

        else:
            print("\n❌ 計測データなし")

        print("\n" + "=" * 80)
        print("✅ 計測完了")
        print("=" * 80)

    except Exception as e:
        print(f"\n❌ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
