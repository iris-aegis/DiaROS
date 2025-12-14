#!/usr/bin/env python3
"""
First stage中断 vs 非中断でのSecond stage生成比較

以下の2つのシナリオを比較：
1. シナリオA（中断あり）: First stage生成途中で中断 → Second stage生成
2. シナリオB（中断なし）: First stage完全生成 → Second stage生成

各シナリオでトータルレイテンシを測定
"""

import time
import subprocess
import requests
import json
from typing import Generator, Tuple
import sys
import threading
import statistics

class FirstSecondStageComparator:
    def __init__(self, model_name: str = "gemma3:4b", ollama_host: str = "http://localhost:11434"):
        """初期化"""
        print(f"📦 Ollama クライアント初期化中: {model_name}")

        self.model_name = model_name
        self.ollama_host = ollama_host
        self.current_response = None
        self.response_closed = False
        self.first_stage_result = ""

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

    def scenario_a_with_interrupt(self, num_iterations: int = 5) -> Tuple[list, list]:
        """
        シナリオA: First stage途中で中断 → Second stage生成
        """
        print("\n" + "="*70)
        print(f"🔴 シナリオA: First stage中断あり（{num_iterations}回測定）")
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

        total_times = []
        first_stage_times = []
        second_stage_times = []

        for iteration in range(num_iterations):
            print(f"\n【測定 {iteration+1}/{num_iterations}】")

            total_start = time.time()

            # First stage: 途中で中断（70ms後）
            print("  First stage 生成中（70ms後に中断）...", end="", flush=True)
            first_stage_start = time.time()
            self.first_stage_result = ""

            def interrupt_after_delay():
                time.sleep(0.07)  # 70ms後に中断
                self.interrupt_generation()

            interrupt_thread = threading.Thread(target=interrupt_after_delay, daemon=True)
            interrupt_thread.start()

            gen1, _ = self.generate_streaming(first_stage_prompt, max_tokens=10)
            for token in gen1:
                if token not in ["[中断]", "[APIエラー]"]:
                    self.first_stage_result += token

            first_stage_end = time.time()
            first_stage_elapsed = (first_stage_end - first_stage_start) * 1000
            print(f" {first_stage_elapsed:.1f}ms")

            # Second stage: 本応答生成
            print("  Second stage 生成中...", end="", flush=True)
            second_stage_start = time.time()
            second_stage_result = ""

            second_stage_prompt = second_stage_prompt_template.format(first_stage_result=self.first_stage_result)
            gen2, _ = self.generate_streaming(second_stage_prompt, max_tokens=30)
            for token in gen2:
                if token not in ["[中断]", "[APIエラー]"]:
                    second_stage_result += token

            second_stage_end = time.time()
            second_stage_elapsed = (second_stage_end - second_stage_start) * 1000
            print(f" {second_stage_elapsed:.1f}ms")

            total_elapsed = (second_stage_end - total_start) * 1000

            print(f"  📊 結果: '{self.first_stage_result}' → '{second_stage_result[:30]}...'")
            print(f"  ⏱️  トータル時間: {total_elapsed:.1f}ms (First: {first_stage_elapsed:.1f}ms, Second: {second_stage_elapsed:.1f}ms)")

            total_times.append(total_elapsed)
            first_stage_times.append(first_stage_elapsed)
            second_stage_times.append(second_stage_elapsed)

            time.sleep(0.5)

        return total_times, first_stage_times, second_stage_times

    def scenario_b_without_interrupt(self, num_iterations: int = 5) -> Tuple[list, list]:
        """
        シナリオB: First stage完全生成 → Second stage生成
        """
        print("\n" + "="*70)
        print(f"🟢 シナリオB: First stage完全生成（{num_iterations}回測定）")
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

        total_times = []
        first_stage_times = []
        second_stage_times = []

        for iteration in range(num_iterations):
            print(f"\n【測定 {iteration+1}/{num_iterations}】")

            total_start = time.time()

            # First stage: 完全生成（中断なし）
            print("  First stage 生成中（完全生成）...", end="", flush=True)
            first_stage_start = time.time()
            first_stage_result = ""

            gen1, _ = self.generate_streaming(first_stage_prompt, max_tokens=10)
            for token in gen1:
                if token not in ["[中断]", "[APIエラー]"]:
                    first_stage_result += token

            first_stage_end = time.time()
            first_stage_elapsed = (first_stage_end - first_stage_start) * 1000
            print(f" {first_stage_elapsed:.1f}ms")

            # Second stage: 本応答生成
            print("  Second stage 生成中...", end="", flush=True)
            second_stage_start = time.time()
            second_stage_result = ""

            second_stage_prompt = second_stage_prompt_template.format(first_stage_result=first_stage_result)
            gen2, _ = self.generate_streaming(second_stage_prompt, max_tokens=30)
            for token in gen2:
                if token not in ["[中断]", "[APIエラー]"]:
                    second_stage_result += token

            second_stage_end = time.time()
            second_stage_elapsed = (second_stage_end - second_stage_start) * 1000
            print(f" {second_stage_elapsed:.1f}ms")

            total_elapsed = (second_stage_end - total_start) * 1000

            print(f"  📊 結果: '{first_stage_result}' → '{second_stage_result[:30]}...'")
            print(f"  ⏱️  トータル時間: {total_elapsed:.1f}ms (First: {first_stage_elapsed:.1f}ms, Second: {second_stage_elapsed:.1f}ms)")

            total_times.append(total_elapsed)
            first_stage_times.append(first_stage_elapsed)
            second_stage_times.append(second_stage_elapsed)

            time.sleep(0.5)

        return total_times, first_stage_times, second_stage_times


def main():
    """メイン処理"""
    print("\n" + "="*70)
    print("🔍 First Stage中断 vs 非中断でのSecond Stage生成比較")
    print("="*70)

    try:
        comparator = FirstSecondStageComparator()

        # シナリオA: 中断あり
        a_total, a_first, a_second = comparator.scenario_a_with_interrupt(num_iterations=5)

        # シナリオB: 中断なし
        b_total, b_first, b_second = comparator.scenario_b_without_interrupt(num_iterations=5)

        # 統計分析
        print("\n" + "="*70)
        print("📈 統計分析結果")
        print("="*70)

        print("\n🔴 シナリオA: First stage中断あり")
        print(f"  トータル時間:")
        print(f"    • 個別測定: {[f'{x:.1f}ms' for x in a_total]}")
        print(f"    • 平均値: {statistics.mean(a_total):.1f}ms")
        print(f"    • 中央値: {statistics.median(a_total):.1f}ms")
        if len(a_total) > 1:
            print(f"    • 標準偏差: {statistics.stdev(a_total):.1f}ms")
        print(f"  First stage:")
        print(f"    • 平均値: {statistics.mean(a_first):.1f}ms")
        print(f"  Second stage:")
        print(f"    • 平均値: {statistics.mean(a_second):.1f}ms")

        print("\n🟢 シナリオB: First stage完全生成")
        print(f"  トータル時間:")
        print(f"    • 個別測定: {[f'{x:.1f}ms' for x in b_total]}")
        print(f"    • 平均値: {statistics.mean(b_total):.1f}ms")
        print(f"    • 中央値: {statistics.median(b_total):.1f}ms")
        if len(b_total) > 1:
            print(f"    • 標準偏差: {statistics.stdev(b_total):.1f}ms")
        print(f"  First stage:")
        print(f"    • 平均値: {statistics.mean(b_first):.1f}ms")
        print(f"  Second stage:")
        print(f"    • 平均値: {statistics.mean(b_second):.1f}ms")

        # 比較分析
        a_mean = statistics.mean(a_total)
        b_mean = statistics.mean(b_total)
        difference = b_mean - a_mean
        difference_ratio = (difference / b_mean) * 100

        print("\n" + "="*70)
        print("🔬 比較分析")
        print("="*70)
        print(f"\nシナリオA（中断あり）のトータル: {a_mean:.1f}ms")
        print(f"シナリオB（完全生成）のトータル: {b_mean:.1f}ms")
        print(f"差分: {difference:+.1f}ms ({difference_ratio:+.1f}%)")

        print("\n💡 結論:")
        if difference > 0:
            print(f"  ✅ 中断戦略により約{difference:.1f}ms ({abs(difference_ratio):.1f}%)高速化")
            print("     First stageを途中で中断してSecond stageに切り替える方が効率的")
        elif difference < -20:
            print(f"  ⚠️  中断戦略により約{abs(difference):.1f}ms ({abs(difference_ratio):.1f}%)低速化")
            print("     完全生成の方がコスト効果的（中断のオーバーヘッドが大きい）")
        else:
            print(f"  🤔 差分が小さい（{difference:+.1f}ms）")
            print("     中断のメリットが限定的な可能性")

        print(f"\n詳細分析:")
        print(f"  • シナリオAでのFirst stage: {statistics.mean(a_first):.1f}ms")
        print(f"  • シナリオBでのFirst stage: {statistics.mean(b_first):.1f}ms")
        print(f"  → 中断により約{statistics.mean(b_first) - statistics.mean(a_first):.1f}ms削減")

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
