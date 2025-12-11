#!/usr/bin/env python3
"""
Second stage のみの応答生成テスト

このスクリプトはSecond stageの応答生成時間を単独で計測します。
First stageは完了したと仮定し、その結果を使ってSecond stage生成にかかる時間のみを測定します。

使用方法:
  python scripts/test/test_second_stage_only.py [反復回数]

例:
  python scripts/test/test_second_stage_only.py 5  # 5回測定
"""

import sys
import time
import statistics
from datetime import datetime
from pathlib import Path

# DiaROS_py モジュールをインポート
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'DiaROS_py'))

from diaros.naturalLanguageGeneration import NaturalLanguageGeneration


class SecondStageOnlyTester:
    def __init__(self):
        """テスター初期化"""
        print("=" * 70)
        print("🧪 Second Stage のみのテスト")
        print("=" * 70)
        print("\n📦 NLG モジュール初期化中...")

        try:
            self.nlg = NaturalLanguageGeneration()
            print("✅ NLG モジュール初期化完了\n")
        except Exception as e:
            print(f"❌ NLG モジュール初期化失敗: {e}")
            sys.exit(1)

    def test_second_stage_only(self, num_iterations: int = 5):
        """
        Second stage のみのテスト実行

        Args:
            num_iterations: 測定回数
        """
        print("=" * 70)
        print(f"📊 Second Stage のみテスト（{num_iterations}回測定）")
        print("=" * 70)

        # テスト用のASR結果
        test_asr_results = [
            "こんにちは",
            "今日は天気がいいですね",
            "サイクリングもいいかな",
            "でも悩んでて",
            "何か言いあんないかな"
        ]

        # First stage の結果をシミュレート（実際にはfirst stageで生成されるもの）
        simulated_first_stage_result = "そっかー"

        generation_times = []
        responses = []

        for iteration in range(num_iterations):
            print(f"\n【測定 {iteration+1}/{num_iterations}】")
            print(f"  入力ASR結果: {test_asr_results}")
            print(f"  First stage結果（シミュレート）: '{simulated_first_stage_result}'")

            # NLGモジュールの状態を設定
            self.nlg.current_stage = 'second'
            self.nlg.first_stage_response = simulated_first_stage_result
            self.nlg.asr_results = test_asr_results

            # Second stage 生成の計測開始
            print("  ⏱️  Second stage 生成中...", end="", flush=True)
            start_time = time.time()

            try:
                # NLG の generate_second_stage を直接呼び出し
                self.nlg.generate_second_stage(test_asr_results)

                end_time = time.time()
                elapsed_ms = (end_time - start_time) * 1000

                response = self.nlg.last_reply

                print(f" 完了")
                print(f"  📝 生成応答: '{response}'")
                print(f"  ⏱️  生成時間: {elapsed_ms:.1f}ms")

                generation_times.append(elapsed_ms)
                responses.append(response)

            except Exception as e:
                print(f"\n  ❌ エラー発生: {e}")
                import traceback
                traceback.print_exc()
                continue

            # 次の測定の前に少し待機
            time.sleep(0.5)

        # 統計分析
        self._print_statistics(generation_times, responses)

    def _print_statistics(self, times: list, responses: list):
        """統計情報を表示"""
        if not times:
            print("\n❌ テスト失敗: 生成データなし")
            return

        print("\n" + "=" * 70)
        print("📈 統計分析")
        print("=" * 70)

        min_time = min(times)
        max_time = max(times)
        mean_time = statistics.mean(times)

        print(f"\n⏱️  生成時間:")
        print(f"  • 個別測定: {[f'{t:.1f}ms' for t in times]}")
        print(f"  • 最小値: {min_time:.1f}ms")
        print(f"  • 最大値: {max_time:.1f}ms")
        print(f"  • 平均値: {mean_time:.1f}ms")
        print(f"  • 中央値: {statistics.median(times):.1f}ms")

        if len(times) > 1:
            print(f"  • 標準偏差: {statistics.stdev(times):.1f}ms")

        print(f"\n🔍 レイテンシ分析:")

        if mean_time < 500:
            print(f"  ✅ 優秀: {mean_time:.1f}ms < 500ms")
        elif mean_time < 1000:
            print(f"  ⚠️  許容範囲: 500ms ≤ {mean_time:.1f}ms < 1000ms")
        elif mean_time < 2000:
            print(f"  🟡 改善必要: 1000ms ≤ {mean_time:.1f}ms < 2000ms")
        else:
            print(f"  🔴 改善急務: {mean_time:.1f}ms ≥ 2000ms")

        print(f"\n📝 生成応答のサンプル:")
        for i, response in enumerate(responses[:3], 1):
            print(f"  {i}. '{response}'")

        print("\n" + "=" * 70)
        print("✅ テスト完了")
        print("=" * 70)


def main():
    """メイン処理"""
    # コマンドライン引数から反復回数を取得
    num_iterations = 5
    if len(sys.argv) > 1:
        try:
            num_iterations = int(sys.argv[1])
        except ValueError:
            print(f"❌ エラー: 反復回数は整数で指定してください")
            sys.exit(1)

    try:
        tester = SecondStageOnlyTester()
        tester.test_second_stage_only(num_iterations=num_iterations)

    except KeyboardInterrupt:
        print("\n\n⚠️  テストが中断されました")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
