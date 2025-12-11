#!/usr/bin/env python3
"""
NaturalLanguageGeneration (naturalLanguageGeneration.py) への
具体的な実装例

このファイルは、First stage生成の中断機能を実装するための
コード例を示しています。

【使用方法】
1. このファイルで実装パターンを確認
2. naturalLanguageGeneration.py に同じロジックを統合
3. テストコマンドで動作確認：
   - python3 test_gemma3_interrupt.py
   - python3 test_first_second_stage_comparison.py
"""

import requests
import json
import time
import threading
from typing import Optional


class InterruptibleNLGExample:
    """
    First stage 中断機能付き NLG の実装例
    """

    def __init__(self):
        """初期化：中断制御用の変数を追加"""
        # ===== 新規追加：中断制御用変数 =====
        self.first_stage_thread = None  # First stageスレッドオブジェクト
        self.cancel_first_stage = False  # キャンセルフラグ
        self.first_stage_response = ""  # First stage結果のキャッシュ
        self.first_stage_start_time = None  # 開始時刻（計測用）
        # ====================================

        # その他の既存変数
        self.current_stage = "first"
        self.ollama_host = "http://localhost:11434"
        self.model_name = "gemma3:4b"

    def update(self, words, stage="first", turn_taking_decision_timestamp_ns=0):
        """
        【修正版】stage パラメータで処理を切り替え

        Args:
            words: 入力テキスト（音声認識結果）
            stage: 'first' or 'second' を指定
            turn_taking_decision_timestamp_ns: TurnTaking判定時刻
        """

        self.current_stage = stage

        # ===== 新規追加：stage別の処理分岐 =====
        if stage == "first":
            # First stage: バックグラウンドで非優先実行
            print(f"[First stage] 非優先処理として開始")

            # キャンセルフラグをリセット
            self.cancel_first_stage = False

            # バックグラウンドスレッドで実行
            self.first_stage_thread = threading.Thread(
                target=self._run_first_stage_background,
                args=(words,),
                daemon=True
            )
            self.first_stage_thread.start()
            # スレッド開始直後に戻る（ブロッキングなし）

        elif stage == "second":
            # Second stage: First stageを中断して優先実行
            print(f"[Second stage] First stageの中断シグナルを送信")

            # キャンセルフラグをセット
            self.cancel_first_stage = True

            # First stageスレッドの終了を待機（タイムアウト付き）
            if self.first_stage_thread and self.first_stage_thread.is_alive():
                self.first_stage_thread.join(timeout=0.1)
                print(f"[Second stage] First stage中断完了")

            # Second stageを実行（同期的に）
            self._run_second_stage_blocking(words)

        # ====================================

    def _run_first_stage_background(self, words):
        """
        【新規メソッド】First stage生成（バックグラウンド実行版）
        """
        print(f"[First stage] 推論開始: '{words}'")
        self.first_stage_start_time = time.perf_counter()

        result = self.generate_first_stage_with_cancellation(words)
        print(f"[First stage] 推論完了: '{result}'")

    def _run_second_stage_blocking(self, words):
        """
        【新規メソッド】Second stage生成（同期実行版）
        """
        print(f"[Second stage] 推論開始")

        # First stageの結果をキャッシュから取得
        first_stage_result = self.first_stage_response or "うん"

        # Second stageプロンプトを組み立て
        second_stage_prompt = f"""あなたは男性ユーザの友達である優しく明るい性格である女性のアンドロイドです。
先ほどあなたはユーザに対して短い「リアクションワード」を出力しました。それに続く、「タメ口の応答」を出力してください。

復元した発話に対し、以下の「リアクションワード」に自然に続く「タメ口の応答」を制約事項に従って出力してください。
{first_stage_result}

制約事項:
- 「リアクションワード」は含めないでください。
- 20文字程度の一言に収めてください。

出力:"""

        result = self.generate_second_stage(second_stage_prompt)
        print(f"[Second stage] 推論完了: '{result}'")

        return result

    def generate_first_stage_with_cancellation(self, query: str) -> str:
        """
        【修正版】First stage生成（キャンセル可能）

        重要なポイント：
        1. ストリーミングループ内でキャンセルフラグを監視
        2. 中断検出時：response.close()で接続を切断
        3. 結果をキャッシュに保存
        4. デフォルト値を返す
        """

        url = f"{self.ollama_host}/api/generate"

        payload = {
            "model": self.model_name,
            "prompt": query,
            "temperature": 0.7,
            "num_predict": 10,  # First stageは短い
            "stream": True,
            "raw": False,
        }

        response = None
        result = ""

        try:
            response = requests.post(url, json=payload, stream=True, timeout=None)

            if response.status_code != 200:
                print(f"❌ APIエラー: {response.status_code}")
                return "うん"

            for line in response.iter_lines():
                # ===== 重要：キャンセルフラグチェック =====
                if self.cancel_first_stage:
                    print(
                        f"⏸️  First stage中断: {len(result)}トークン生成, 結果='{result}'"
                    )

                    # 接続を切断
                    if response:
                        response.close()

                    # 結果をキャッシュに保存（Second stageで使用）
                    self.first_stage_response = result if result else "うん"

                    return result if result else "うん"

                # ==========================================

                if line:
                    try:
                        data = json.loads(line)
                        token_text = data.get("response", "")

                        if token_text:
                            result += token_text

                        # 生成完了フラグ
                        if data.get("done", False):
                            break

                    except json.JSONDecodeError:
                        continue

            # 生成完了時
            print(f"✅ First stage完全生成: '{result}'")
            self.first_stage_response = result
            return result

        except Exception as e:
            print(f"❌ First stage生成エラー: {e}")
            return "うん"

        finally:
            if response:
                response.close()

    def generate_second_stage(self, query: str) -> str:
        """
        【未修正】Second stage生成

        既存の実装をそのまま使用
        """

        url = f"{self.ollama_host}/api/generate"

        payload = {
            "model": self.model_name,
            "prompt": query,
            "temperature": 0.7,
            "num_predict": 30,  # Second stageは長めの応答
            "stream": True,
            "raw": False,
        }

        response = None
        result = ""

        try:
            response = requests.post(url, json=payload, stream=True, timeout=None)

            if response.status_code != 200:
                print(f"❌ APIエラー: {response.status_code}")
                return ""

            for line in response.iter_lines():
                if line:
                    try:
                        data = json.loads(line)
                        token_text = data.get("response", "")

                        if token_text:
                            result += token_text

                        if data.get("done", False):
                            break

                    except json.JSONDecodeError:
                        continue

            return result

        except Exception as e:
            print(f"❌ Second stage生成エラー: {e}")
            return ""

        finally:
            if response:
                response.close()


# ============================================================================
# 使用例
# ============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("🔄 NLG 中断機能の実装例")
    print("=" * 70)

    nlg = InterruptibleNLGExample()

    # ===== シナリオ：First stage生成中にSecond stageをリクエスト =====

    # Step 1: First stage開始
    print("\n【Step 1】First stage開始")
    nlg.update("こんにちは、今日は天気がいいですね", stage="first")

    # Step 2: 100msで中断してSecond stage開始
    print("\n【Step 2】100ms待機後にSecond stage開始（First stage中断）")
    time.sleep(0.1)
    nlg.update("こんにちは、今日は天気がいいですね", stage="second")

    # Step 3: 処理完了待機
    time.sleep(1)

    print("\n" + "=" * 70)
    print("✅ 実装例完了")
    print("=" * 70)

    print("\n💡 実装ポイント:")
    print("  1. update() メソッドで stage パラメータで処理を切り替え")
    print("  2. First stage は自動的にバックグラウンドスレッドで実行")
    print("  3. Second stage 開始時に cancel_first_stage フラグをセット")
    print("  4. First stage 生成ループでキャンセルフラグをチェック")
    print("  5. 結果は self.first_stage_response にキャッシュ")
    print("  6. Second stage で first_stage_response を参照")
