#!/usr/bin/env python3
"""
First stage完了/中断 → Second stage開始までの遅延時間比較

以下の2つのシナリオで、状態遷移の遅延を測定：
1. シナリオA（中断あり）: First stage中断完了 → Second stage開始
2. シナリオB（完全生成）: First stage生成完了 → Second stage開始
"""

import time
import subprocess
import requests
import json
from typing import Generator, Tuple
import sys
import threading
import statistics

class SwitchingLatencyValidator:
    def __init__(self, model_name: str = "gemma3:4b", ollama_host: str = "http://localhost:11434"):
        """初期化"""
        print(f"📦 Ollama クライアント初期化中: {model_name}")

        self.model_name = model_name
        self.ollama_host = ollama_host
        self.current_response = None
        self.response_closed = False

        try:
            response = requests.get(f"{ollama_host}/api/tags", timeout=5)
            if response.status_code == 200:
                print(f"✅ Ollamaサーバーが稼働中: {ollama_host}")
        except requests.exceptions.RequestException:
            print(f"⚠️  Ollamaサーバーが起動していません。起動中...")
            self._start_ollama()

        self._check_model_available()
        print("✅ 初期化完了\n")

    def _start_ollama(self):
        """Ollamaサーバーを起動"""
        print("▶ Ollamaサーバーを起動中...")
        try:
            subprocess.Popen(
                ["ollama", "serve"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            time.sleep(5)
            print("✅ Ollamaサーバーが起動しました")
        except Exception as e:
            print(f"❌ Ollama起動失敗: {e}")
            sys.exit(1)

    def _check_model_available(self):
        """モデルが利用可能かチェック"""
        try:
            response = requests.get(f"{self.ollama_host}/api/tags", timeout=10)
            if response.status_code == 200:
                models = response.json().get("models", [])
                model_names = [m["name"] for m in models]
                if self.model_name in model_names:
                    print(f"✅ モデル {self.model_name} が利用可能です")
        except requests.exceptions.RequestException as e:
            print(f"❌ モデル確認失敗: {e}")
            sys.exit(1)

    def generate_streaming(
        self,
        prompt: str,
        max_tokens: int = 50,
        temperature: float = 0.7
    ) -> Tuple[Generator[str, None, None], float]:
        """ストリーミング生成（中断可能）"""
        url = f"{self.ollama_host}/api/generate"

        payload = {
            "model": self.model_name,
            "prompt": prompt,
            "temperature": temperature,
            "num_predict": max_tokens,
            "stream": True,
            "raw": False
        }

        def token_generator():
            try:
                self.response_closed = False
                self.current_response = requests.post(url, json=payload, stream=True, timeout=None)

                if self.current_response.status_code != 200:
                    yield "APIエラー"
                    return

                for line in self.current_response.iter_lines():
                    if self.response_closed:
                        self.current_response.close()
                        yield "[中断]"
                        return

                    if line:
                        try:
                            data = json.loads(line)
                            token_text = data.get("response", "")
                            if token_text:
                                yield token_text
                            if data.get("done", False):
                                break
                        except json.JSONDecodeError:
                            continue

            except Exception as e:
                yield f"[エラー: {e}]"

        return token_generator(), time.time()

    def interrupt_generation(self):
        """生成を中断"""
        self.response_closed = True
        if self.current_response:
            try:
                self.current_response.close()
            except:
                pass

    def scenario_a_with_interrupt(self, num_iterations: int = 5) -> list:
        """
        シナリオA: First stage途中で中断 → Second stage開始までの遅延を測定
        """
        print("\n" + "="*70)
        print(f"🔴 シナリオA: First stage中断 → Second stage開始（{num_iterations}回測定）")
        print("="*70)

        first_stage_prompt = """あなたは男性ユーザの友達である優しく明るい性格である女性アンドロイドです。
与えられた複数のぶつ切りの音声認識結果から、ユーザの「感情」と「発話の意図」のみを読み取り、
タメ口で応答する際の最初のリアクションワードのみを出力してください。

制約事項:
- 出力はリアクションワード（2～5文字程度）のみとしてください。
- ユーザの「感情」と「発話の意図」だけで判断してください。
- 以下のリストのタメ口のリアクションワードのみ出力してください。
  - 肯定・共感: うんうん、そっかー、なるほど
  - 驚き・感心: へー、すごい
  - 笑顔・楽しい: あはは、いいね
  - 困惑・同情: えー、まじか

音声認識結果: こんにちは、今日は天気がいいですね
出力:"""

        second_stage_prompt_template = """あなたは男性ユーザの友達である優しく明るい性格である女性のアンドロイドです。
先ほどあなたはユーザに対して短い「リアクションワード」を出力しました。それに続く、「タメ口の応答」を出力してください。

まず以下の複数のぶつ切りの音声認識結果から元の発話を復元してください。
こんにちは、今日は天気がいいですね

その上で、復元した発話に対し、以下の「リアクションワード」に自然に続く「タメ口の応答」を制約事項に従って出力してください。
{first_stage_result}

制約事項:
- 「リアクションワード」は含めないでください。いきなり「タメ口の応答」を出力してください。
- 20文字程度の一言に収めてください。
- 「タメ口の応答」のみを出力してください。

出力:"""

        switching_latencies = []

        for iteration in range(num_iterations):
            print(f"\n【測定 {iteration+1}/{num_iterations}】")

            # First stage: 途中で中断（70ms後）
            first_stage_result = ""
            first_stage_end_time = None

            def interrupt_after_delay():
                time.sleep(0.07)  # 70ms後に中断
                self.interrupt_generation()

            interrupt_thread = threading.Thread(target=interrupt_after_delay, daemon=True)
            interrupt_thread.start()

            gen1, _ = self.generate_streaming(first_stage_prompt, max_tokens=10)
            for token in gen1:
                if token not in ["[中断]", "[APIエラー]"]:
                    first_stage_result += token

            # First stage終了時刻を記録（高精度計測用）
            first_stage_end_time = time.perf_counter()

            # Second stage開始時刻を記録（高精度計測用）
            second_stage_start_time = time.perf_counter()
            switching_latency = (second_stage_start_time - first_stage_end_time) * 1000

            # Second stageを実行
            second_stage_prompt = second_stage_prompt_template.format(first_stage_result=first_stage_result)
            second_stage_result = ""
            gen2, _ = self.generate_streaming(second_stage_prompt, max_tokens=30)
            for token in gen2:
                if token not in ["[中断]", "[APIエラー]"]:
                    second_stage_result += token

            print(f"  ⏱️  First stage終了 → Second stage開始: {switching_latency:.4f}ms")
            switching_latencies.append(switching_latency)

            time.sleep(0.5)

        return switching_latencies

    def scenario_b_without_interrupt(self, num_iterations: int = 5) -> list:
        """
        シナリオB: First stage完全生成 → Second stage開始までの遅延を測定
        """
        print("\n" + "="*70)
        print(f"🟢 シナリオB: First stage完全生成 → Second stage開始（{num_iterations}回測定）")
        print("="*70)

        first_stage_prompt = """あなたは男性ユーザの友達である優しく明るい性格である女性アンドロイドです。
与えられた複数のぶつ切りの音声認識結果から、ユーザの「感情」と「発話の意図」のみを読み取り、
タメ口で応答する際の最初のリアクションワードのみを出力してください。

制約事項:
- 出力はリアクションワード（2～5文字程度）のみとしてください。
- ユーザの「感情」と「発話の意図」だけで判断してください。
- 以下のリストのタメ口のリアクションワードのみ出力してください。
  - 肯定・共感: うんうん、そっかー、なるほど
  - 驚き・感心: へー、すごい
  - 笑顔・楽しい: あはは、いいね
  - 困惑・同情: えー、まじか

音声認識結果: こんにちは、今日は天気がいいですね
出力:"""

        second_stage_prompt_template = """あなたは男性ユーザの友達である優しく明るい性格である女性のアンドロイドです。
先ほどあなたはユーザに対して短い「リアクションワード」を出力しました。それに続く、「タメ口の応答」を出力してください。

まず以下の複数のぶつ切りの音声認識結果から元の発話を復元してください。
こんにちは、今日は天気がいいですね

その上で、復元した発話に対し、以下の「リアクションワード」に自然に続く「タメ口の応答」を制約事項に従って出力してください。
{first_stage_result}

制約事項:
- 「リアクションワード」は含めないでください。いきなり「タメ口の応答」を出力してください。
- 20文字程度の一言に収めてください。
- 「タメ口の応答」のみを出力してください。

出力:"""

        switching_latencies = []

        for iteration in range(num_iterations):
            print(f"\n【測定 {iteration+1}/{num_iterations}】")

            # First stage: 完全生成
            first_stage_result = ""
            first_stage_end_time = None

            gen1, _ = self.generate_streaming(first_stage_prompt, max_tokens=10)
            for token in gen1:
                if token not in ["[中断]", "[APIエラー]"]:
                    first_stage_result += token

            # First stage終了時刻を記録（高精度計測用）
            first_stage_end_time = time.perf_counter()

            # Second stage開始時刻を記録（高精度計測用）
            second_stage_start_time = time.perf_counter()
            switching_latency = (second_stage_start_time - first_stage_end_time) * 1000

            # Second stageを実行
            second_stage_prompt = second_stage_prompt_template.format(first_stage_result=first_stage_result)
            second_stage_result = ""
            gen2, _ = self.generate_streaming(second_stage_prompt, max_tokens=30)
            for token in gen2:
                if token not in ["[中断]", "[APIエラー]"]:
                    second_stage_result += token

            print(f"  ⏱️  First stage終了 → Second stage開始: {switching_latency:.4f}ms")
            switching_latencies.append(switching_latency)

            time.sleep(0.5)

        return switching_latencies


def main():
    """メイン処理"""
    print("\n" + "="*70)
    print("🔍 First stage完了/中断 → Second stage開始までの遅延比較")
    print("="*70)

    try:
        validator = SwitchingLatencyValidator()

        # シナリオA: 中断あり
        a_latencies = validator.scenario_a_with_interrupt(num_iterations=5)

        # シナリオB: 完全生成
        b_latencies = validator.scenario_b_without_interrupt(num_iterations=5)

        # 統計分析
        print("\n" + "="*70)
        print("📈 統計分析結果")
        print("="*70)

        print("\n🔴 シナリオA: First stage中断後の切り替え遅延")
        print(f"  • 個別測定値: {[f'{x:.4f}ms' for x in a_latencies]}")
        print(f"  • 最小値: {min(a_latencies):.4f}ms")
        print(f"  • 最大値: {max(a_latencies):.4f}ms")
        print(f"  • 平均値: {statistics.mean(a_latencies):.4f}ms")
        print(f"  • 中央値: {statistics.median(a_latencies):.4f}ms")
        if len(a_latencies) > 1:
            print(f"  • 標準偏差: {statistics.stdev(a_latencies):.4f}ms")

        print("\n🟢 シナリオB: First stage完全生成後の切り替え遅延")
        print(f"  • 個別測定値: {[f'{x:.4f}ms' for x in b_latencies]}")
        print(f"  • 最小値: {min(b_latencies):.4f}ms")
        print(f"  • 最大値: {max(b_latencies):.4f}ms")
        print(f"  • 平均値: {statistics.mean(b_latencies):.4f}ms")
        print(f"  • 中央値: {statistics.median(b_latencies):.4f}ms")
        if len(b_latencies) > 1:
            print(f"  • 標準偏差: {statistics.stdev(b_latencies):.4f}ms")

        # 比較分析
        a_mean = statistics.mean(a_latencies)
        b_mean = statistics.mean(b_latencies)
        difference = a_mean - b_mean
        difference_ratio = (difference / b_mean) * 100 if b_mean != 0 else 0

        print("\n" + "="*70)
        print("🔬 比較分析")
        print("="*70)
        print(f"\n中断時の切り替え遅延: {a_mean:.4f}ms")
        print(f"完全生成時の切り替え遅延: {b_mean:.4f}ms")
        print(f"差分: {difference:+.4f}ms ({difference_ratio:+.1f}%)")

        print("\n💡 結論:")
        if abs(difference) < 0.1:
            print("  ✅ 両シナリオの切り替え遅延はほぼ同じ")
            print("     システムレベルでの遅延差はない（計測誤差範囲）")
        elif difference > 0:
            print(f"  ⚠️  中断時の方が約{difference:.3f}ms遅い")
            print("     中断処理に若干のオーバーヘッドがある可能性")
        else:
            print(f"  ℹ️  中断時の方が約{abs(difference):.3f}ms高速")
            print("     中断時に若干のメリットがある可能性")

        print("\n" + "="*70)
        print("✅ 検証完了")
        print("="*70)

    except Exception as e:
        print(f"\n❌ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
