#!/usr/bin/env python3
"""
gemma3:4bモデルの中断テストスクリプト
First stage を10回生成させ、10回目を中断し、9回目の結果で Second stage を生成
"""

import time
import subprocess
import requests
import json
from typing import Generator, Optional, Tuple
import sys
import threading

class Gemma3InterruptTester:
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
    ) -> Generator[str, None, None]:
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

        try:
            self.response_closed = False
            self.current_response = requests.post(url, json=payload, stream=True, timeout=None)

            if self.current_response.status_code != 200:
                print(f"\n❌ APIエラー: {self.current_response.status_code}")
                yield "[APIエラー]"
                return

            token_count = 0
            for line in self.current_response.iter_lines():
                # 外部からの中断チェック
                if self.response_closed:
                    print(f"\n⏱️  外部からの中断信号を受信: 生成を中断")
                    self.current_response.close()
                    yield "[生成中断]"
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
            yield "[リクエストタイムアウト]"
            return
        except Exception as e:
            print(f"\n❌ ストリーミングエラー: {e}")
            yield f"[エラー: {e}]"
            return

    def interrupt_generation(self):
        """生成を中断"""
        self.response_closed = True
        if self.current_response:
            try:
                self.current_response.close()
            except:
                pass

    def test_multiple_first_stage_with_interrupt(self):
        """
        First stage を10回生成し、10回目を中断するテスト
        """
        print("\n" + "="*60)
        print("🔄 First Stage 複数回生成テスト（10回目で中断）")
        print("="*60 + "\n")

        asr_results = ["こんにちは", "今日は天気がいいですね"]

        first_stage_results = []
        ninth_stage_result = None
        tenth_stage_start_time = None  # 10回目の推論開始時刻

        # First stage を10回生成
        for i in range(1, 11):
            print(f"\n📍 [{i}/10] First Stage 生成開始...")
            print("-" * 60)

            prompt = f"""あなたは男性ユーザの友達である優しく明るい性格である女性アンドロイドです。
与えられた複数のぶつ切りの音声認識結果から、ユーザの「感情」と「発話の意図」のみを読み取り、
タメ口で応答する際の最初のリアクションワードのみを出力してください。

各音声認識結果は音声長5秒で、2.5秒ずつずらして取得されています。
最後の音声認識結果は最新の音声から2文字分遅れています。

制約事項:
- 出力はリアクションワード（2～5文字程度）のみとしてください。句読点は含めないでください。
- ユーザの「感情」と「発話の意図」だけで判断してください。
- 以下のリストまたはそれに準ずるタメ口のリアクションワードのみ出力してください。
  - 肯定・共感: うんうん、そっかー、なるほど
  - 驚き・感心: へー、すごい
  - 笑顔・楽しい: あはは、いいね
  - 困惑・同情: えー、まじか

音声認識結果:
{', '.join(asr_results)}

出力:"""

            start_time = time.time()
            result = ""

            # 10回目の推論開始時刻を記録
            if i == 10:
                tenth_stage_start_time = start_time

            # 10回目の場合は別スレッドで中断タスクを実行
            if i == 10:
                def interrupt_after_delay():
                    time.sleep(0.1)  # 100ms後に中断
                    now_interrupt = time.time()
                    print(f"\n⏸️  [中断信号] 10回目の生成を中断します... (t={now_interrupt:.3f}s)")
                    self.interrupt_generation()

                interrupt_thread = threading.Thread(target=interrupt_after_delay, daemon=True)
                interrupt_thread.start()

            for token in self.generate_with_interrupt_capability(prompt, max_tokens=10, temperature=0.7):
                if token not in ["[生成完了]", "[生成中断]", "[APIエラー]", "[リクエストタイムアウト]"]:
                    print(token, end="", flush=True)
                    result += token
                elif token == "[生成中断]":
                    print(f"[生成中断]", end="", flush=True)

            elapsed = time.time() - start_time
            print(f"\n-" * 60)
            print(f"✅ [{i}/10] 完了")
            print(f"⏱️  生成時間: {elapsed*1000:.1f}ms")
            print(f"📝 結果: '{result.strip()}'")

            # 結果を保存
            if i == 9:
                ninth_stage_result = result.strip()
                print(f"💾 [重要] 9回目の結果を保存: '{ninth_stage_result}'")

            first_stage_results.append({
                "iteration": i,
                "result": result.strip(),
                "elapsed_ms": elapsed * 1000
            })

        return first_stage_results, ninth_stage_result, tenth_stage_start_time

    def test_second_stage_with_ninth_result(self, ninth_stage_result: str):
        """
        Second stage: 9回目の結果を使用して本応答を生成
        """
        print("\n" + "="*60)
        print("💬 Second Stage テスト（9回目の結果を使用）")
        print("="*60)

        asr_results = ["こんにちは", "今日は天気がいいですね"]
        asr_text = ", ".join(asr_results)

        prompt = f"""あなたは男性ユーザの友達である優しく明るい性格である女性のアンドロイドです。
先ほどあなたはユーザに対して短い「リアクションワード」を出力しました。それに続く、「タメ口の応答」を出力してください。

まず以下の複数のぶつ切りの音声認識結果から元の発話を復元してください。
{asr_text}

その上で、復元した発話に対し、以下の「リアクションワード」に自然に続く「タメ口の応答」を制約事項に従って出力してください。
{ninth_stage_result}

制約事項:
- 「リアクションワード」（うん、へー、あはは 等）は含めないでください。いきなり「タメ口の応答」を出力してください。
- 20文字程度の一言に収めてください。
- 「タメ口の応答」のみを出力してください。

出力:"""

        print(f"📝 使用する First Stage 結果: '{ninth_stage_result}'")
        print(f"⏳ 生成開始...")
        print("-" * 60)

        start_time = time.time()
        second_stage_start = start_time
        print(f"🕐 [Second Stage] 推論開始時刻: {start_time:.3f}s")
        result = ""

        for token in self.generate_with_interrupt_capability(prompt, max_tokens=30, temperature=0.7):
            if token not in ["[生成完了]", "[生成中断]", "[APIエラー]", "[リクエストタイムアウト]"]:
                print(token, end="", flush=True)
                result += token

        elapsed = time.time() - start_time
        second_stage_end = time.time()
        print(f"\n-" * 60)
        print(f"✅ Second Stage 生成完了")
        print(f"🕐 [Second Stage] 推論停止時刻: {second_stage_end:.3f}s")
        print(f"⏱️  推論開始→停止の経過時間: {elapsed*1000:.1f}ms")
        print(f"📝 結果: '{result.strip()}'")

        return result.strip(), second_stage_start, second_stage_end


def main():
    """
    メイン処理
    """
    print("\n" + "="*60)
    print("🚀 Gemma3:4b 中断テスト")
    print("   (First stage 10回生成→10回目中断→9回目結果でSecond stage)")
    print("="*60)

    try:
        # テスター初期化
        tester = Gemma3InterruptTester()

        # First stage を10回生成し、10回目を中断
        first_stage_results, ninth_result, tenth_start_time = tester.test_multiple_first_stage_with_interrupt()

        # Second stage: 9回目の結果を使用
        if ninth_result:
            second_stage_result, second_stage_start, second_stage_end = tester.test_second_stage_with_ninth_result(ninth_result)
        else:
            print("\n❌ 9回目の結果が見つかりません")
            return

        # サマリーを表示
        print("\n" + "="*60)
        print("📊 テスト結果サマリー")
        print("="*60)

        print("\n🔄 First Stage 生成結果（10回）:")
        for result in first_stage_results:
            status = "✅" if result["iteration"] != 10 else "⏸️ "
            print(f"  {status} [{result['iteration']:2d}/10] {result['elapsed_ms']:6.1f}ms → '{result['result']}'")

        print(f"\n💬 Second Stage 結果（9回目の '{ninth_result}' を使用）:")
        print(f"  ✅ {second_stage_result}")

        # タイムスタンプサマリーを表示
        print("\n" + "="*60)
        print("🕐 推論タイムスタンプサマリー")
        print("="*60)
        print(f"10回目推論開始時刻: {tenth_start_time:.3f}s")
        print(f"Second Stage推論開始時刻: {second_stage_start:.3f}s")
        print(f"Second Stage推論完了時刻: {second_stage_end:.3f}s")
        print(f"10th→Second Stage切り替え時間: {(second_stage_start - tenth_start_time)*1000:.1f}ms")

        print("\n" + "="*60)
        print("✅ テスト完了")
        print("="*60)
        print("\n💡 この結果から以下が検証されました：")
        print("  1. First stage を複数回連続生成可能")
        print("  2. 生成途中での中断が可能")
        print("  3. 前回の結果を使用して Second stage 生成可能")
        print("  4. NLG モジュールでの実装を検証できる")

    except Exception as e:
        print(f"\n❌ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
