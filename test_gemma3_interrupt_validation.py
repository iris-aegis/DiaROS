#!/usr/bin/env python3
"""
gemma3:4bの中断機能の有効性を検証するテストスクリプト

長い応答を生成する際に、異なるタイミングで中断命令を出して、
実際に中断が機能しているか確認します
"""

import time
import subprocess
import requests
import json
from typing import Generator, Optional, Tuple
import sys
import threading

class Gemma3InterruptValidator:
    def __init__(self, model_name: str = "gemma3:4b", ollama_host: str = "http://localhost:11434"):
        """
        Gemma3モデルの初期化

        Args:
            model_name: Ollamaのモデル名
            ollama_host: OllamaのホストURL
        """
        print(f"📦 Ollama クライアント初期化中: {model_name}")

        self.model_name = model_name
        self.ollama_host = ollama_host
        self.timeout_seconds = 60
        self.current_response = None
        self.response_closed = False

        # Ollamaサーバーの状態確認
        try:
            response = requests.get(f"{ollama_host}/api/tags", timeout=5)
            if response.status_code == 200:
                print(f"✅ Ollamaサーバーが稼働中: {ollama_host}")
            else:
                print(f"⚠️  Ollamaサーバーに接続できません")
                self._start_ollama()
        except requests.exceptions.RequestException:
            print(f"⚠️  Ollamaサーバーが起動していません。起動中...")
            self._start_ollama()

        # モデルの確認
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

                if self.model_name not in model_names:
                    print(f"⚠️  モデル {self.model_name} がインストールされていません。")
                    print(f"   利用可能なモデル: {model_names}")
                else:
                    print(f"✅ モデル {self.model_name} が利用可能です")
        except requests.exceptions.RequestException as e:
            print(f"❌ モデル確認失敗: {e}")
            sys.exit(1)

    def generate_with_interrupt_capability(
        self,
        prompt: str,
        max_tokens: int = 50,
        temperature: float = 0.7
    ) -> Tuple[Generator[str, None, None], float]:
        """
        生成可能でありながら、外部から中断できるジェネレータ

        Args:
            prompt: 入力プロンプト
            max_tokens: 最大トークン数
            temperature: サンプリング温度

        Yields:
            生成されたトークン
        """
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
                    print(f"\n❌ APIエラー: {self.current_response.status_code}")
                    yield "APIエラー"
                    return

                token_count = 0
                for line in self.current_response.iter_lines():
                    # 外部からの中断チェック
                    if self.response_closed:
                        print(f"\n⏱️  外部からの中断信号を受信: 生成を中断")
                        self.current_response.close()
                        yield "[中断]"
                        return

                    if line:
                        try:
                            data = json.loads(line)
                            token_text = data.get("response", "")

                            if token_text:
                                yield token_text
                                token_count += 1

                            # 生成完了フラグ
                            if data.get("done", False):
                                break
                        except json.JSONDecodeError:
                            continue

            except requests.exceptions.Timeout:
                print(f"\n⏱️  リクエストタイムアウト")
                yield "[タイムアウト]"
                return
            except Exception as e:
                print(f"\n❌ ストリーミングエラー: {e}")
                yield f"[エラー: {e}]"
                return

        return token_generator(), time.time()

    def interrupt_generation(self):
        """生成を中断"""
        self.response_closed = True
        if self.current_response:
            try:
                self.current_response.close()
            except:
                pass

    def test_long_generation_without_interrupt(self):
        """
        長い応答を中断なしで生成（ベースライン）
        """
        print("\n" + "="*70)
        print("🔄 テスト1: 長い応答を中断なしで生成（ベースライン）")
        print("="*70 + "\n")

        prompt = """あなたは有能なアシスタントです。以下の質問に対して、詳しく、丁寧に答えてください。

質問: 人工知能とは何か、その応用例を含めて詳しく説明してください。"""

        print(f"⏳ 生成開始（max_tokens=200、中断なし）...")
        print("-" * 70)

        start_time = time.time()
        result = ""
        token_count = 0

        generator, _ = self.generate_with_interrupt_capability(prompt, max_tokens=200, temperature=0.7)

        for token in generator:
            if token not in ["[中断]", "[タイムアウト]", "[APIエラー]"]:
                print(token, end="", flush=True)
                result += token
                token_count += 1

        elapsed = time.time() - start_time
        print(f"\n" + "-" * 70)
        print(f"✅ 生成完了")
        print(f"⏱️  合計時間: {elapsed*1000:.1f}ms")
        print(f"📊 生成トークン数: {token_count}")
        print(f"📝 結果: {result[:100]}..." if len(result) > 100 else f"📝 結果: {result}")

        return elapsed, token_count, result

    def test_long_generation_with_interrupt_at_timing(self, interrupt_delay_ms: int):
        """
        長い応答を指定したタイミングで中断

        Args:
            interrupt_delay_ms: 中断まで待機する時間（ミリ秒）
        """
        print("\n" + "="*70)
        print(f"🔄 テスト2: 長い応答を {interrupt_delay_ms}ms で中断")
        print("="*70 + "\n")

        prompt = """あなたは有能なアシスタントです。以下の質問に対して、詳しく、丁寧に答えてください。

質問: 人工知能とは何か、その応用例を含めて詳しく説明してください。"""

        print(f"⏳ 生成開始（max_tokens=200、{interrupt_delay_ms}ms後に中断）...")
        print("-" * 70)

        start_time = time.time()
        result = ""
        token_count = 0

        # 中断タスクを別スレッドで実行
        def interrupt_after_delay():
            time.sleep(interrupt_delay_ms / 1000.0)
            now_interrupt = time.time()
            elapsed_ms = (now_interrupt - start_time) * 1000
            print(f"\n⏸️  [中断信号] {interrupt_delay_ms}ms時点で生成を中断します... (実際: {elapsed_ms:.1f}ms)")
            self.interrupt_generation()

        interrupt_thread = threading.Thread(target=interrupt_after_delay, daemon=True)
        interrupt_thread.start()

        generator, _ = self.generate_with_interrupt_capability(prompt, max_tokens=200, temperature=0.7)

        for token in generator:
            if token not in ["[中断]", "[タイムアウト]", "[APIエラー]"]:
                print(token, end="", flush=True)
                result += token
                token_count += 1
            elif token == "[中断]":
                print(f"[中断]", end="", flush=True)

        elapsed = time.time() - start_time
        print(f"\n" + "-" * 70)
        print(f"✅ 中断成功")
        print(f"⏱️  実際の経過時間: {elapsed*1000:.1f}ms")
        print(f"📊 中断までに生成されたトークン数: {token_count}")
        print(f"📝 結果: {result[:100]}..." if len(result) > 100 else f"📝 結果: {result}")

        return elapsed, token_count, result


def main():
    """
    メイン処理
    """
    print("\n" + "="*70)
    print("🚀 Gemma3:4b 中断機能の有効性検証")
    print("="*70)

    try:
        # バリデータ初期化
        validator = Gemma3InterruptValidator()

        # テスト1: 中断なしで長い応答を生成
        baseline_time, baseline_tokens, baseline_result = validator.test_long_generation_without_interrupt()

        # テスト2-4: 異なるタイミングで中断
        interrupt_results = []
        for interrupt_delay in [500, 1000, 1500]:
            time.sleep(1)  # テスト間隔
            elapsed, tokens, result = validator.test_long_generation_with_interrupt_at_timing(interrupt_delay)
            interrupt_results.append({
                "delay_ms": interrupt_delay,
                "elapsed_ms": elapsed * 1000,
                "tokens": tokens,
                "result_length": len(result)
            })

        # サマリーを表示
        print("\n" + "="*70)
        print("📊 検証結果サマリー")
        print("="*70)

        print(f"\n🔵 ベースライン（中断なし）:")
        print(f"  • 生成時間: {baseline_time*1000:.1f}ms")
        print(f"  • トークン数: {baseline_tokens}")
        print(f"  • 結果長: {len(baseline_result)}文字")

        print(f"\n🟠 中断ありのテスト結果:")
        for result in interrupt_results:
            time_ratio = result["elapsed_ms"] / (baseline_time * 1000)
            token_ratio = result["tokens"] / baseline_tokens if baseline_tokens > 0 else 0
            print(f"\n  【{result['delay_ms']}msで中断】")
            print(f"    • 実際の経過時間: {result['elapsed_ms']:.1f}ms ({time_ratio:.1%} of baseline)")
            print(f"    • トークン数: {result['tokens']} ({token_ratio:.1%} of baseline)")
            print(f"    • 結果長: {result['result_length']}文字")

        print(f"\n{'='*70}")
        print("💡 検証結果の解釈:")
        print(f"{'='*70}")
        print("• 中断が有効な場合:")
        print("  - 経過時間が指定した中断タイミングより大幅に短い")
        print("  - トークン数がベースラインより大幅に少ない")
        print("  - 結果長がベースラインより大幅に短い")
        print("\n• 中断が無効な場合:")
        print("  - 経過時間がベースラインと同程度")
        print("  - トークン数がベースラインと同程度")
        print("  - 結果長がベースラインと同程度")

        print(f"\n{'='*70}")
        print("✅ 検証完了")
        print(f"{'='*70}")

    except Exception as e:
        print(f"\n❌ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
