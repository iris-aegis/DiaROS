#!/usr/bin/env python3
"""
Ollama Python SDK を使用したダイレクト推論オーバーヘッド計測

特徴：
- REST API を経由しない直接推論
- ollama.generate() で Python から直接モデル実行
- HTTP リクエスト/レスポンスのオーバーヘッドを削減
"""

import ollama
import time
import statistics
import threading
from typing import List

class DirectOllamaLatencyValidator:
    def __init__(self, model_name: str = "gemma3:4b"):
        """初期化"""
        print(f"📦 Ollama モデル初期化中: {model_name}")

        self.model_name = model_name
        self.interrupt_send_time = None
        self.second_stage_first_token_time = None
        self.response_closed = False
        self.generated_tokens = []

        try:
            # モデルが利用可能かチェック
            print("   モデル確認中...", end="", flush=True)
            # 簡単なテストで確認
            response = ollama.generate(
                model=model_name,
                prompt="test",
                stream=False,
            )
            print(" ✅")
            print("✅ 初期化完了\n")
        except Exception as e:
            print(f"\n❌ モデル初期化失敗: {e}")
            raise

    def generate_with_interrupt(self, prompt: str, max_tokens: int = 200) -> tuple:
        """
        推論実行（トークンごとにストリーミング）

        戻り値: (生成テキスト, 最初のトークン時刻)
        """
        self.generated_tokens = []
        self.response_closed = False
        first_token_time = None

        try:
            for chunk in ollama.generate(
                model=self.model_name,
                prompt=prompt,
                stream=True,
            ):
                # 【重要】各トークン受信時にチェック
                if self.response_closed:
                    break

                token_text = chunk.get('response', '')

                # 最初のトークンの時刻を記録
                if first_token_time is None and token_text:
                    first_token_time = time.perf_counter()

                if token_text:
                    self.generated_tokens.append(token_text)

                if chunk.get('done', False):
                    break

        except Exception as e:
            print(f"❌ 生成エラー: {e}")

        generated_text = ''.join(self.generated_tokens)
        return generated_text, first_token_time

    def interrupt_generation(self):
        """生成を中断"""
        self.response_closed = True

    def measure_latency(self, num_iterations: int = 5) -> List[float]:
        """
        中断命令送信 → Second stage 最初のトークン受信までのレイテンシを計測

        シーケンス：
        1. First iteration: 短い応答を生成（完全生成）
        2. Second iteration～: 長い応答を生成開始 → 500msで中断 → Second stage開始
        """
        print("=" * 80)
        print(f"🔬 ダイレクト推論オーバーヘッド計測（{num_iterations}回測定）")
        print("=" * 80)

        latencies = []

        for iteration in range(num_iterations):
            print(f"\n【測定 {iteration+1}/{num_iterations}】")

            if iteration == 0:
                # ============================================================
                # 【初回】Short stage: 短い応答を完全生成（ウォームアップ）
                # ============================================================
                print("  🔥 ウォームアップ: 短い応答を完全生成中...", end="", flush=True)

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
    print("🔬 Ollama Python SDK ダイレクト推論")
    print("=" * 80)

    try:
        # 初期化
        validator = DirectOllamaLatencyValidator()

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
            rest_api_latency = 156.7  # 前回のREST API計測値
            reduction = rest_api_latency - mean_latency
            reduction_percent = (reduction / rest_api_latency) * 100

            print(f"\n📊 推論方式の比較:")
            print(f"  • REST API経由（500ms中断）: {rest_api_latency:.2f}ms")
            print(f"  • ダイレクト推論（ollama SDK）: {mean_latency:.2f}ms")
            print(f"  • 削減: {reduction:.2f}ms ({reduction_percent:.1f}%)")

            if reduction > 10:
                print(f"\n✅ ダイレクト推論でオーバーヘッドが削減されました")
                print(f"   推定API初期化時間: {reduction:.0f}ms")
            else:
                print(f"\n⚠️  削減効果が小さい")
                print(f"   → ollama SDK も内部で HTTP を使用している可能性")
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
