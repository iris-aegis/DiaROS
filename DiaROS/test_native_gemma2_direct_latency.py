#!/usr/bin/env python3
"""
Native Gemma-2-2B-IT 直接推論オーバーヘッド計測（より安定したバージョン）

Ollama を一切使わず、transformers を使ってローカルで直接推論し、
中断～Second stage 開始までのオーバーヘッドを計測

使用モデル: google/gemma-2-2b-it (gemma-3 より安定)
"""

import torch
import time
import statistics
import threading
from transformers import AutoTokenizer, AutoModelForCausalLM
from typing import List

class NativeGemma2DirectValidator:
    def __init__(self, model_name: str = "google/gemma-2-2b-it"):
        """初期化"""
        print(f"📦 Native Gemma-2-2B-IT 初期化中: {model_name}")
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

    def generate_with_timing(
        self,
        prompt: str,
        max_tokens: int = 200,
        temperature: float = 0.7
    ) -> tuple:
        """
        推論実行（タイミング計測付き）

        戻り値: (生成テキスト, 生成開始時刻, 生成終了時刻)
        """
        start_time = time.perf_counter()
        generated_text = ""

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

                # generate を実行
                outputs = self.model.generate(
                    input_ids,
                    max_new_tokens=max_tokens,
                    temperature=temperature,
                    do_sample=True,
                    top_p=0.9,
                    top_k=50,
                    pad_token_id=self.tokenizer.eos_token_id,
                )

                # 生成テキストをデコード
                generated_ids = outputs[0, input_ids.shape[1]:]
                generated_text = self.tokenizer.decode(generated_ids, skip_special_tokens=True)

                if self.device == "cuda":
                    torch.cuda.empty_cache()

        except Exception as e:
            print(f"❌ 生成エラー: {e}")

        end_time = time.perf_counter()
        return generated_text, start_time, end_time

    def measure_latency(self, num_iterations: int = 5) -> List[float]:
        """
        推論開始～終了までのレイテンシを計測

        シーケンス：
        1. First iteration: 短い応答を生成（ウォームアップ）
        2. Second iteration～: 長いと短い応答のレイテンシ比較
        """
        print("=" * 80)
        print(f"🔬 Native Gemma-2-2B-IT オーバーヘッド計測（{num_iterations}回測定）")
        print("=" * 80)

        latencies = []

        for iteration in range(num_iterations):
            print(f"\n【測定 {iteration+1}/{num_iterations}】")

            if iteration == 0:
                # ============================================================
                # 【初回】Short response: 短い応答を生成（ウォームアップ）
                # ============================================================
                print("  🔥 ウォームアップ: 短い応答を生成中...", end="", flush=True)

                short_prompt = "こんにちは。返事をしてください。"

                result_short, start_time, end_time = self.generate_with_timing(
                    short_prompt, max_tokens=10
                )

                warmup_time = (end_time - start_time) * 1000
                print(f" ✅ ({warmup_time:.2f}ms)")
                print(f"   結果: '{result_short[:50]}...'")
                time.sleep(0.5)
                continue

            # ============================================================
            # 【本計測】Short vs Long 比較
            # ============================================================
            print("  📝 Short response: 短い応答を生成中...", end="", flush=True)

            short_prompt = "こんにちは。簡潔に返事をしてください。"

            result_short, start_short, end_short = self.generate_with_timing(
                short_prompt, max_tokens=20
            )

            short_time = (end_short - start_short) * 1000
            print(f" ✅ ({short_time:.2f}ms)")

            time.sleep(0.2)

            # ============================================================
            # Long response
            # ============================================================
            print("  💬 Long response: 長い応答を生成中...", end="", flush=True)

            long_prompt = """あなたは詳しい説明をする専門家です。
以下の質問に対して、できるだけ詳しく、長く説明してください。
複数の段落で、詳細な背景情報を含めて、徹底的に説明してください。

質問: なぜ空は青いのですか？

回答:"""

            result_long, start_long, end_long = self.generate_with_timing(
                long_prompt, max_tokens=100
            )

            long_time = (end_long - start_long) * 1000
            print(f" ✅ ({long_time:.2f}ms)")

            # ============================================================
            # 計測結果
            # ============================================================
            latency_diff = long_time - short_time
            print(f"\n  📊 計測結果:")
            print(f"     • Short response: {short_time:.2f}ms")
            print(f"     • Long response: {long_time:.2f}ms")
            print(f"     • 生成時間増加: {latency_diff:.2f}ms")
            print(f"     • Short テキスト長: {len(result_short)}文字")
            print(f"     • Long テキスト長: {len(result_long)}文字")

            latencies.append(long_time)
            time.sleep(0.2)

        return latencies


def main():
    """メイン処理"""
    print("\n" + "=" * 80)
    print("🔬 Native Gemma-2-2B-IT 直接推論テスト")
    print("=" * 80)

    try:
        # 初期化
        validator = NativeGemma2DirectValidator()

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

            # Ollama との比較
            ollama_latency_total = 500.7  # 前回測定の First stage 総時間（500ms生成）
            native_reduction = ollama_latency_total - mean_latency
            reduction_percent = (native_reduction / ollama_latency_total) * 100

            print(f"\n📊 推論方式の比較:")
            print(f"  • Ollama（長応答）: ~500.7ms")
            print(f"  • ネイティブ直接実行: {mean_latency:.2f}ms")
            print(f"  • 削減: {native_reduction:.2f}ms ({reduction_percent:.1f}%)")

            if mean_latency < 200:
                print(f"\n✅ ネイティブ実行で大幅な削減を確認")
                print(f"   → Ollama オーバーヘッド: 約 {native_reduction:.0f}ms")
            elif mean_latency < 350:
                print(f"\n△ 中程度の削減")
                print(f"   → Ollama オーバーヘッド: 約 {native_reduction:.0f}ms")
            else:
                print(f"\n⚠️  削減が小さい")
                print(f"   → ネイティブ実行もOllama同等のオーバーヘッド")

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
