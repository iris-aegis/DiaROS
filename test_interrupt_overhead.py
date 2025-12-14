#!/usr/bin/env python3
"""
中断のオーバーヘッド検証プログラム

以下の2つのシナリオを比較して、中断固有のオーバーヘッドがあるか検証します：
1. 通常の連続リクエスト：Request 1完了 → Request 2開始
2. 中断後のリクエスト：Request 1中断 → Request 2開始

各シナリオで複数回実行し、オーバーヘッド時間を測定
"""

import time
import subprocess
import requests
import json
from typing import Generator, List, Tuple
import sys
import threading
import statistics

class OverheadValidator:
    def __init__(self, model_name: str = "gemma3:4b", ollama_host: str = "http://localhost:11434"):
        """初期化"""
        print(f"📦 Ollama クライアント初期化中: {model_name}")

        self.model_name = model_name
        self.ollama_host = ollama_host
        self.current_response = None
        self.response_closed = False

        # 中断時のタイムスタンプ計測用
        self.interrupt_send_time = None

        # Ollamaサーバーの状態確認
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

    def test_scenario_1_normal_consecutive_requests(self, num_iterations: int = 5) -> List[float]:
        """
        シナリオ1: 通常の連続リクエスト
        Request 1完了 → Request 2開始までの時間を測定
        """
        print("\n" + "="*70)
        print(f"📊 シナリオ1: 通常の連続リクエスト（{num_iterations}回測定）")
        print("="*70)

        prompt1 = "短い質問です。5文字以内で答えてください。ここはどこですか?"
        prompt2 = "別の短い質問です。5文字以内で答えてください。いま何時ですか?"

        overhead_times = []

        for iteration in range(num_iterations):
            print(f"\n【測定 {iteration+1}/{num_iterations}】")

            # Request 1を完全に完了させる
            print("Request 1 実行中...", end="", flush=True)
            request1_start = time.time()
            result1 = ""
            gen1, _ = self.generate_streaming(prompt1, max_tokens=10)
            for token in gen1:
                if token not in ["[中断]", "[APIエラー]"]:
                    result1 += token
            request1_end = time.time()
            print(f" 完了 ({(request1_end - request1_start)*1000:.1f}ms)")

            # Request 2開始までの時間差を測定
            request2_start = time.time()
            overhead = (request2_start - request1_end) * 1000  # ミリ秒

            # Request 2を実行（ただしオーバーヘッド時間は既に記録済み）
            print("Request 2 実行中...", end="", flush=True)
            result2 = ""
            gen2, _ = self.generate_streaming(prompt2, max_tokens=10)
            for token in gen2:
                if token not in ["[中断]", "[APIエラー]"]:
                    result2 += token
            request2_end = time.time()
            print(f" 完了 ({(request2_end - request2_start)*1000:.1f}ms)")

            print(f"  ⏱️  Request 1完了 → Request 2開始: {overhead:.2f}ms")
            overhead_times.append(overhead)

            time.sleep(0.5)  # テスト間隔

        return overhead_times

    def test_scenario_2_interrupted_requests(self, num_iterations: int = 5) -> List[float]:
        """
        シナリオ2: 中断後のリクエスト
        中断命令送信 → Second stage API実行開始までの時間を測定
        """
        print("\n" + "="*70)
        print(f"📊 シナリオ2: 中断後のリクエスト（{num_iterations}回測定）")
        print("="*70)

        prompt1 = "これは長い質問で、詳しい説明を求めます。できるだけ長く答えてください。"
        prompt2 = "別の短い質問です。5文字以内で答えてください。いま何時ですか?"

        overhead_times = []

        for iteration in range(num_iterations):
            print(f"\n【測定 {iteration+1}/{num_iterations}】")

            # Request 1を途中で中断
            print("Request 1 実行中（50ms後に中断）...", end="", flush=True)
            request1_start = time.time()
            result1 = ""

            # 中断命令送信時刻をリセット
            self.interrupt_send_time = None

            def interrupt_after_delay():
                time.sleep(0.05)  # 50msで中断（Request 1完了前に中断させるため）
                self.interrupt_send_time = time.perf_counter()  # 【重要】中断命令送信時刻を記録
                self.interrupt_generation()

            interrupt_thread = threading.Thread(target=interrupt_after_delay, daemon=True)
            interrupt_thread.start()

            gen1, _ = self.generate_streaming(prompt1, max_tokens=200)
            for token in gen1:
                if token not in ["[中断]", "[APIエラー]"]:
                    result1 += token
            request1_end = time.time()
            print(f" 中断完了 ({(request1_end - request1_start)*1000:.1f}ms)")

            # Request 2を実行
            print("Request 2 実行中...", end="", flush=True)
            result2 = ""
            request2_start = time.time()  # Request 2実行開始時刻
            gen2_start_time = time.perf_counter()  # 【重要】Second stage API実行開始時刻を記録
            gen2, _ = self.generate_streaming(prompt2, max_tokens=10)
            for token in gen2:
                if token not in ["[中断]", "[APIエラー]"]:
                    result2 += token
            request2_end = time.time()
            print(f" 完了 ({(request2_end - request2_start)*1000:.1f}ms)")

            # 実際のオーバーヘッド時間を計算：中断命令送信 → Second stage API実行開始
            if self.interrupt_send_time is not None:
                overhead = (gen2_start_time - self.interrupt_send_time) * 1000  # ミリ秒
            else:
                overhead = 0.0

            print(f"  ⏱️  中断命令送信 → Second stage API実行開始: {overhead:.4f}ms")
            overhead_times.append(overhead)

            time.sleep(0.5)  # テスト間隔

        return overhead_times


def main():
    """メイン処理"""
    print("\n" + "="*70)
    print("🔍 中断のオーバーヘッド検証")
    print("="*70)

    try:
        validator = OverheadValidator()

        # シナリオ1: 通常の連続リクエスト
        normal_overheads = validator.test_scenario_1_normal_consecutive_requests(num_iterations=5)

        # シナリオ2: 中断後のリクエスト
        interrupt_overheads = validator.test_scenario_2_interrupted_requests(num_iterations=5)

        # 統計分析
        print("\n" + "="*70)
        print("📈 統計分析結果")
        print("="*70)

        print("\n🔵 シナリオ1: 通常の連続リクエスト")
        print(f"  • 個別測定値: {[f'{x:.2f}ms' for x in normal_overheads]}")
        print(f"  • 最小値: {min(normal_overheads):.2f}ms")
        print(f"  • 最大値: {max(normal_overheads):.2f}ms")
        print(f"  • 平均値: {statistics.mean(normal_overheads):.2f}ms")
        print(f"  • 中央値: {statistics.median(normal_overheads):.2f}ms")
        if len(normal_overheads) > 1:
            print(f"  • 標準偏差: {statistics.stdev(normal_overheads):.2f}ms")

        print("\n🟠 シナリオ2: 中断後のリクエスト")
        print(f"  • 個別測定値: {[f'{x:.2f}ms' for x in interrupt_overheads]}")
        print(f"  • 最小値: {min(interrupt_overheads):.2f}ms")
        print(f"  • 最大値: {max(interrupt_overheads):.2f}ms")
        print(f"  • 平均値: {statistics.mean(interrupt_overheads):.2f}ms")
        print(f"  • 中央値: {statistics.median(interrupt_overheads):.2f}ms")
        if len(interrupt_overheads) > 1:
            print(f"  • 標準偏差: {statistics.stdev(interrupt_overheads):.2f}ms")

        # 比較
        normal_mean = statistics.mean(normal_overheads)
        interrupt_mean = statistics.mean(interrupt_overheads)
        difference = interrupt_mean - normal_mean
        difference_ratio = (difference / normal_mean) * 100

        print("\n" + "="*70)
        print("🔬 比較分析")
        print("="*70)
        print(f"\n通常リクエストのオーバーヘッド: {normal_mean:.2f}ms")
        print(f"中断後リクエストのオーバーヘッド: {interrupt_mean:.2f}ms")
        print(f"差分: {difference:+.2f}ms ({difference_ratio:+.1f}%)")

        print("\n💡 結論:")
        if abs(difference) < 5:
            print("  ✅ 中断固有のオーバーヘッドはほぼない")
            print("     （差分 < 5ms = 通常のゆらぎ範囲内）")
            print("     両シナリオ間に有意な差がない")
        elif difference > 0:
            print(f"  ⚠️  中断後に約{difference:.1f}msの追加オーバーヘッドが発生")
            print("     中断後の再接続処理に若干の遅延あり")
        else:
            print(f"  ✅ 中断後の方が約{abs(difference):.1f}ms高速")
            print("     中断固有の問題なし")

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
