#!/usr/bin/env python3
"""
正確な中断レイテンシ計測テスト

実測シナリオ：
1. First stage で長い応答を生成開始
2. 100ms で応答生成を中断
3. Second stage のリクエストを送信
4. 中断命令送信時刻 → Second stage 応答が実際に開始される時刻までの時間を計測

【計測対象】
- interrupt_send_time: 中断命令を送信した時刻
- second_stage_first_token_time: Second stage が最初のトークンを受け取った時刻
- 差分: 実際のオーバーヘッド時間
"""

import time
import subprocess
import requests
import json
from typing import Generator, Tuple
import sys
import threading
import statistics

class PreciseInterruptLatencyValidator:
    def __init__(self, model_name: str = "gemma3:4b", ollama_host: str = "http://localhost:11434"):
        """初期化"""
        print(f"📦 Ollama クライアント初期化中: {model_name}")

        self.model_name = model_name
        self.ollama_host = ollama_host
        self.current_response = None
        self.response_closed = False

        # 計測用の変数
        self.interrupt_send_time = None
        self.second_stage_first_token_time = None
        self.first_token_received = False

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

    def generate_streaming_with_timing(
        self,
        prompt: str,
        max_tokens: int = 50,
        temperature: float = 0.7
    ) -> Tuple[Generator[str, None, None], float]:
        """ストリーミング生成（最初のトークン時刻を記録）"""
        url = f"{self.ollama_host}/api/generate"

        payload = {
            "model": self.model_name,
            "prompt": prompt,
            "temperature": temperature,
            "num_predict": max_tokens,
            "stream": True,
            "raw": False
        }

        api_request_time = time.perf_counter()  # API リクエスト送信時刻

        def token_generator():
            try:
                self.response_closed = False
                self.current_response = requests.post(url, json=payload, stream=True, timeout=None)

                if self.current_response.status_code != 200:
                    yield "APIエラー"
                    return

                first_token = True
                for line in self.current_response.iter_lines():
                    if self.response_closed:
                        self.current_response.close()
                        yield "[中断]"
                        return

                    if line:
                        try:
                            data = json.loads(line)
                            token_text = data.get("response", "")

                            # 【重要】最初のトークンの時刻を記録
                            if first_token and token_text:
                                self.second_stage_first_token_time = time.perf_counter()
                                first_token = False

                            if token_text:
                                yield token_text
                            if data.get("done", False):
                                break
                        except json.JSONDecodeError:
                            continue

            except Exception as e:
                yield f"[エラー: {e}]"

        return token_generator(), api_request_time

    def interrupt_generation(self):
        """生成を中断"""
        self.response_closed = True
        if self.current_response:
            try:
                self.current_response.close()
            except:
                pass

    def measure_latency(self, num_iterations: int = 5) -> list:
        """
        中断命令送信 → Second stage 最初のトークン受信までのレイテンシを計測

        シーケンス：
        1. First iteration: 短い応答を生成（完全生成）
        2. Second iteration～: 長い応答を生成開始 → 500msで中断 → Second stage開始
        """
        print("=" * 80)
        print(f"🔬 精密オーバーヘッド計測（{num_iterations}回測定）")
        print("=" * 80)

        latencies = []

        for iteration in range(num_iterations):
            print(f"\n【測定 {iteration+1}/{num_iterations}】")

            if iteration == 0:
                # ============================================================
                # 【初回】Short stage: 短い応答を完全生成
                # ============================================================
                print("  📝 [初回] Short stage: 短い応答を完全生成中...", end="", flush=True)

                short_prompt = """こんにちは。返事をしてください。"""

                result_short = ""
                gen_short, _ = self.generate_streaming_with_timing(short_prompt, max_tokens=10)
                for token in gen_short:
                    if token not in ["[中断]", "[APIエラー]"]:
                        result_short += token

                print(" ✅")
                print(f"     結果: '{result_short}'")
                time.sleep(0.5)
                continue  # 次のイテレーションへ

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
                self.interrupt_send_time = time.perf_counter()  # 【重要】中断命令送信時刻を記録
                print(f"\n  ⏸️  [中断命令] 500ms時点で中断信号を送信", end="", flush=True)
                self.interrupt_generation()

            interrupt_thread = threading.Thread(target=interrupt_after_500ms, daemon=True)
            interrupt_thread.start()

            # First stage 生成を実行
            result1 = ""
            gen1, api_request_time1 = self.generate_streaming_with_timing(first_stage_prompt, max_tokens=200)
            for token in gen1:
                if token not in ["[中断]", "[APIエラー]"]:
                    result1 += token

            interrupt_thread.join()  # 中断スレッド完了待機
            print(" ✅")

            # ============================================================
            # 【Step 2】Second stage: リクエスト送信
            # ============================================================
            print("  💬 Second stage: 応答生成開始", end="", flush=True)

            second_stage_prompt = """感謝のメッセージを短く述べてください。"""

            # Second stage 応答開始
            result2 = ""
            gen2, api_request_time2 = self.generate_streaming_with_timing(second_stage_prompt, max_tokens=30)

            for token in gen2:
                if token not in ["[中断]", "[APIエラー]"]:
                    result2 += token

            print(" ✅")

            # ============================================================
            # 【計測結果】
            # ============================================================
            if self.interrupt_send_time is not None and self.second_stage_first_token_time is not None:
                latency = (self.second_stage_first_token_time - self.interrupt_send_time) * 1000
                print(f"\n  📊 計測結果:")
                print(f"     • 中断命令送信時刻: {self.interrupt_send_time:.6f}s")
                print(f"     • Second stage 最初のトークン受信: {self.second_stage_first_token_time:.6f}s")
                print(f"     • 🎯 オーバーヘッド: {latency:.4f}ms")

                print(f"\n  📝 結果:")
                print(f"     • First stage ({len(result1)}文字): {result1[:50]}...")
                print(f"     • Second stage: {result2[:50]}...")

                latencies.append(latency)
            else:
                print(f"\n  ❌ 計測失敗（タイムスタンプ記録なし）")

            time.sleep(0.5)  # テスト間隔

        return latencies


def main():
    """メイン処理"""
    print("\n" + "=" * 80)
    print("🔬 精密オーバーヘッド計測: 中断命令 → Second stage 応答開始")
    print("=" * 80)

    try:
        validator = PreciseInterruptLatencyValidator()

        # 計測を実行
        latencies = validator.measure_latency(num_iterations=5)

        # 統計分析
        print("\n" + "=" * 80)
        print("📈 統計分析")
        print("=" * 80)

        if latencies:
            print(f"\n📊 個別測定値: {[f'{x:.4f}ms' for x in latencies]}")
            print(f"  • 最小値: {min(latencies):.4f}ms")
            print(f"  • 最大値: {max(latencies):.4f}ms")
            print(f"  • 平均値: {statistics.mean(latencies):.4f}ms")
            print(f"  • 中央値: {statistics.median(latencies):.4f}ms")
            if len(latencies) > 1:
                print(f"  • 標準偏差: {statistics.stdev(latencies):.4f}ms")

            print("\n💡 分析:")
            mean_latency = statistics.mean(latencies)

            if mean_latency < 50:
                print("  ✅ オーバーヘッド少ない: < 50ms")
                print("     → 中断戦略は効率的")
            elif mean_latency < 150:
                print("  ⚠️  中程度のオーバーヘッド: 50-150ms")
                print("     → API初期化のコストを含む正常な値")
            else:
                print("  ⚠️  高いオーバーヘッド: > 150ms")
                print("     → Second stage API初期化に時間がかかっている可能性")
        else:
            print("\n❌ 計測データなし")

        print("\n" + "=" * 80)
        print("✅ 計測完了")
        print("=" * 80)

    except Exception as e:
        print(f"\n❌ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
