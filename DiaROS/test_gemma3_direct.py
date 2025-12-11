#!/usr/bin/env python3
"""
gemma3:4bモデルの直接実行テストスクリプト
Ollamaをバックエンドとして、Pythonクライアントで直接実行して、
レイテンシと中断可能性を検証します
"""

import time
import subprocess
import requests
import json
from typing import Generator, Optional
import sys

class Gemma3DirectRunner:
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
                    print(f"   インストール中: ollama pull {self.model_name}")
                    try:
                        subprocess.run(
                            ["ollama", "pull", self.model_name],
                            check=True,
                            capture_output=True
                        )
                        print(f"✅ モデル {self.model_name} をインストールしました")
                    except subprocess.CalledProcessError as e:
                        print(f"❌ モデルインストール失敗: {e}")
                        sys.exit(1)
                else:
                    print(f"✅ モデル {self.model_name} が利用可能です")
        except requests.exceptions.RequestException as e:
            print(f"❌ モデル確認失敗: {e}")
            sys.exit(1)

    def generate_streaming(
        self,
        prompt: str,
        max_tokens: int = 50,
        temperature: float = 0.7,
        timeout_seconds: Optional[float] = None
    ) -> Generator[str, None, None]:
        """
        ストリーミング生成をジェネレータで実行
        タイムアウト機能と中断可能性をテスト

        Args:
            prompt: 入力プロンプト
            max_tokens: 最大トークン数
            temperature: サンプリング温度
            timeout_seconds: タイムアウト時間（秒）

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

        start_time = time.time()
        first_token_time = None
        token_count = 0

        try:
            response = requests.post(url, json=payload, stream=True, timeout=timeout_seconds)

            if response.status_code != 200:
                print(f"\n❌ APIエラー: {response.status_code}")
                yield "[APIエラー]"
                return

            for line in response.iter_lines():
                # タイムアウトチェック
                if timeout_seconds is not None:
                    elapsed = time.time() - start_time
                    if elapsed > timeout_seconds:
                        print(f"\n⏱️  タイムアウト ({elapsed:.2f}秒): 生成を中断")
                        response.close()
                        yield "[生成タイムアウト]"
                        return

                if line:
                    try:
                        data = json.loads(line)
                        token_text = data.get("response", "")

                        if token_text:
                            yield token_text
                            token_count += 1

                            # 最初のトークン時刻を記録
                            if first_token_time is None:
                                first_token_time = time.time() - start_time

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

        # 統計情報を出力
        total_time = time.time() - start_time
        if token_count > 0:
            print(f"\n📊 統計:")
            print(f"  ・第1トークン生成時刻: {first_token_time*1000:.1f}ms" if first_token_time else "  ・第1トークン生成時刻: N/A")
            print(f"  ・合計時間: {total_time*1000:.1f}ms")
            print(f"  ・生成トークン数: {token_count}")
            print(f"  ・平均トークン時間: {(total_time/token_count)*1000:.1f}ms")

    def test_first_stage(self):
        """
        First stage (相槌生成) のテスト
        """
        print("\n" + "="*60)
        print("🎤 First Stage テスト (相槌生成)")
        print("="*60)

        # First stageプロンプトテンプレート
        asr_results = ["こんにちは", "今日は天気がいいですね"]
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

        print(f"📝 プロンプト:\n{prompt[:200]}...\n")
        print("⏳ 生成開始...")
        print("-" * 60)

        start_time = time.time()
        result = ""

        for token in self.generate_streaming(prompt, max_tokens=10, timeout_seconds=15.0):
            if token not in ["[生成完了]", "[生成タイムアウト]", "[APIエラー]", "[リクエストタイムアウト]"]:
                print(token, end="", flush=True)
                result += token

        elapsed = time.time() - start_time
        print(f"\n-" * 60)
        print(f"✅ First Stage 生成完了")
        print(f"⏱️  合計時間: {elapsed*1000:.1f}ms")
        print(f"📝 結果: {result.strip()}\n")

        return result.strip()

    def test_second_stage(self, first_stage_result: str):
        """
        Second stage (本応答生成) のテスト
        """
        print("\n" + "="*60)
        print("💬 Second Stage テスト (本応答生成)")
        print("="*60)

        # Second stageプロンプトテンプレート
        asr_results = ["こんにちは", "今日は天気がいいですね"]
        asr_text = ", ".join(asr_results)

        prompt = f"""あなたは男性ユーザの友達である優しく明るい性格である女性のアンドロイドです。
先ほどあなたはユーザに対して短い「リアクションワード」を出力しました。それに続く、「タメ口の応答」を出力してください。

まず以下の複数のぶつ切りの音声認識結果から元の発話を復元してください。
{asr_text}

その上で、復元した発話に対し、以下の「リアクションワード」に自然に続く「タメ口の応答」を制約事項に従って出力してください。
{first_stage_result}

制約事項:
- 「リアクションワード」（うん、へー、あはは 等）は含めないでください。いきなり「タメ口の応答」を出力してください。
- 20文字程度の一言に収めてください。
- 「タメ口の応答」のみを出力してください。

出力:"""

        print(f"📝 プロンプト:\n{prompt[:200]}...\n")
        print("⏳ 生成開始...")
        print("-" * 60)

        start_time = time.time()
        result = ""

        for token in self.generate_streaming(prompt, max_tokens=30, timeout_seconds=15.0):
            if token not in ["[生成完了]", "[生成タイムアウト]", "[APIエラー]", "[リクエストタイムアウト]"]:
                print(token, end="", flush=True)
                result += token

        elapsed = time.time() - start_time
        print(f"\n-" * 60)
        print(f"✅ Second Stage 生成完了")
        print(f"⏱️  合計時間: {elapsed*1000:.1f}ms")
        print(f"📝 結果: {result.strip()}\n")

        return result.strip()

    def test_timeout_interruption(self):
        """
        タイムアウト・中断可能性のテスト
        """
        print("\n" + "="*60)
        print("⏱️  タイムアウト中断テスト")
        print("="*60)

        prompt = "これは長い回答が生成される質問です。できるだけ詳しく説明してください。"

        print(f"📝 プロンプト: {prompt}")
        print("⏳ 3秒でタイムアウトするように設定して生成開始...\n")
        print("-" * 60)

        result = ""
        for token in self.generate_streaming(prompt, max_tokens=200, timeout_seconds=3.0):
            if token not in ["[生成完了]", "[生成タイムアウト]", "[APIエラー]", "[リクエストタイムアウト]"]:
                print(token, end="", flush=True)
                result += token

        print(f"\n-" * 60)
        print(f"📝 中断までに生成されたテキスト: {result.strip()}")
        print(f"✅ タイムアウト中断テスト完了\n")


def main():
    """
    メイン処理
    """
    print("\n" + "="*60)
    print("🚀 Gemma3:4b 直接実行テスト (Ollama Python クライアント)")
    print("="*60)

    try:
        # モデルの初期化
        runner = Gemma3DirectRunner()

        # First stageテスト
        first_stage_result = runner.test_first_stage()

        # Second stageテスト（First stageの結果を使用）
        second_stage_result = runner.test_second_stage(first_stage_result)

        # タイムアウト・中断テスト
        runner.test_timeout_interruption()

        print("\n" + "="*60)
        print("✅ すべてのテスト完了")
        print("="*60)
        print("\n📊 テスト結果サマリー:")
        print(f"  ・First Stage 結果: {first_stage_result}")
        print(f"  ・Second Stage 結果: {second_stage_result}")
        print("\n💡 次のステップ:")
        print("  1. レイテンシがOllama HTTP API比で改善したか確認")
        print("  2. タイムアウト中断機能が正常に動作したか確認")
        print("  3. 改善があれば、NLGモジュールで実装検討")

    except Exception as e:
        print(f"\n❌ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
