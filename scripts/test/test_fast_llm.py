#\!/usr/bin/env python3
"""
高速日本語LLMモデルのテストスクリプト
使用可能なモデルをテストして応答時間を計測
"""

import os
import sys
import time
import torch
from datetime import datetime

# DiaROSパスを追加
sys.path.append(os.path.join(os.path.dirname(__file__), '../../DiaROS_py'))

def wait_for_model_ready(nlg, max_wait=30):
    """モデルが完全に読み込まれるまで待機"""
    print("モデル読み込み完了を待機中...")
    start_wait = time.time()
    
    while time.time() - start_wait < max_wait:
        try:
            # モデルが利用可能かチェック
            if hasattr(nlg, 'model') and nlg.model is not None:
                # 実際に推論できるかテスト
                test_input = nlg.tokenizer("test", return_tensors="pt")
                if hasattr(nlg, 'device'):
                    test_input = {k: v.to(nlg.device) for k, v in test_input.items()}
                with torch.no_grad():
                    _ = nlg.model(**test_input)
                print("✓ モデル読み込み完了")
                return True
        except Exception as e:
            pass
        time.sleep(0.5)
    
    print("⚠️ モデル読み込みタイムアウト")
    return False

def test_model(model_name):
    """指定されたモデルをテストして応答時間を計測"""
    print(f"\n{'='*60}")
    print(f"テスト開始: {model_name}")
    print('='*60)
    
    # 環境変数を設定
    os.environ["DIAROS_LLM_MODEL"] = model_name
    
    # NLGモジュールをインポート（毎回新規作成）
    from diaros.naturalLanguageGeneration import NaturalLanguageGeneration
    
    try:
        # モジュール初期化
        print("モジュール初期化中...")
        start_init = time.time()
        nlg = NaturalLanguageGeneration()
        nlg.use_local_model = True  # 強制的にローカルモデルを使用
        nlg.init_local_model()
        
        # モデル読み込み完了を待機
        if not wait_for_model_ready(nlg):
            print("モデル読み込みに失敗しました")
            return float('inf')
            
        init_time = time.time() - start_init
        print(f"初期化時間: {init_time:.2f}秒")
        
        # ウォームアップ（初回応答の遅延を回避）
        print("\nウォームアップ中...")
        warmup_start = time.time()
        _ = nlg.generate_local_response("こんにちは")
        warmup_time = (time.time() - warmup_start) * 1000
        print(f"ウォームアップ完了: {warmup_time:.0f}ms")
        
        # テスト発話リスト
        test_queries = [
            "こんにちは",
            "今日はいい天気ですね",
            "疲れました",
            "明日の予定は？",
            "ありがとう"
        ]
        
        print("\n応答テスト:")
        print("-" * 60)
        
        total_time = 0
        response_times = []
        first_response_time = None
        
        for i, query in enumerate(test_queries):
            print(f"\n入力: {query}")
            start_time = time.time()
            response = nlg.generate_local_response(query)
            elapsed_ms = (time.time() - start_time) * 1000
            
            print(f"応答: {response}")
            print(f"時間: {elapsed_ms:.0f}ms")
            
            response_times.append(elapsed_ms)
            if i == 0:
                first_response_time = elapsed_ms
            total_time += elapsed_ms
        
        # 統計情報
        print("\n" + "="*60)
        print("統計情報:")
        avg_time = total_time / len(test_queries)
        avg_time_without_first = sum(response_times[1:]) / len(response_times[1:]) if len(response_times) > 1 else avg_time
        min_time = min(response_times)
        max_time = max(response_times)
        
        print(f"  平均応答時間（全体）: {avg_time:.0f}ms")
        print(f"  平均応答時間（2回目以降）: {avg_time_without_first:.0f}ms")
        print(f"  初回応答時間: {first_response_time:.0f}ms")
        print(f"  最小応答時間: {min_time:.0f}ms")
        print(f"  最大応答時間: {max_time:.0f}ms")
        
        # ウォームアップの効果を表示
        if first_response_time and warmup_time:
            print(f"\n  ウォームアップ効果:")
            print(f"    ウォームアップなし想定: ~{warmup_time:.0f}ms")
            print(f"    ウォームアップあり実測: {first_response_time:.0f}ms")
            print(f"    改善率: {((warmup_time - first_response_time) / warmup_time * 100):.1f}%")
        
        if avg_time <= 500:
            print("\n  ✓ 目標達成: 平均500ms以内")
        else:
            print("\n  ⚠️ 目標未達: 平均500ms超過")
        
        # メモリ使用量（概算）
        if hasattr(nlg, 'model'):
            param_count = sum(p.numel() for p in nlg.model.parameters())
            model_size_mb = param_count * 2 / 1024 / 1024  # float16想定
            print(f"  モデルサイズ: 約{model_size_mb:.0f}MB")
        
        # クリーンアップ
        del nlg
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        
        return avg_time
        
    except Exception as e:
        print(f"エラー発生: {e}")
        import traceback
        traceback.print_exc()
        return float('inf')

def main():
    """メイン処理"""
    print("DiaROS 高速日本語LLMモデル テスト")
    print("="*60)
    
    # デバイス情報を表示
    try:
        from diaros import device_utils
        device = device_utils.get_optimal_device(verbose=True)
    except:
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Device: {device}")
    
    # テストするモデルリスト（全対応モデル）
    models_to_test = [
        ("rinna-small", "最速・最軽量"),
        ("rinna-neox", "高品質・やや重い"),
        ("calm-small", "バランス型"),
        ("stablelm-2", "Japanese StableLM 2・高速"),
        ("phi-3-mini", "Phi-3-mini・超高速"),
        ("elyza-7b", "ELYZA-japanese-Llama-2-7b・高品質日本語"),
        ("gemma-2b", "Google Gemma 2 2B・高速"),
        ("gemma-9b", "Google Gemma 2 9B・高品質"),
        # ("line-small", "LINE LLM・最高品質")  # オプション（大容量）
    ]
    
    # 高速テストモード（最初の3つのみ）
    if len(sys.argv) > 1 and sys.argv[1] == "--fast":
        models_to_test = models_to_test[:3]
        print("高速テストモード: 最初の3モデルのみテスト\n")
    
    results = {}
    
    # 各モデルをテスト
    for model_id, description in models_to_test:
        print(f"\n\n[モデル: {model_id}] {description}")
        avg_time = test_model(model_id)
        if avg_time != float('inf'):
            results[model_id] = avg_time
    
    # 結果サマリー
    if results:
        print("\n\n" + "="*60)
        print("テスト結果サマリー")
        print("="*60)
        
        # 速度順にソート
        sorted_results = sorted(results.items(), key=lambda x: x[1])
        
        for i, (model_id, avg_time) in enumerate(sorted_results, 1):
            status = "✓" if avg_time <= 500 else "✗"
            print(f"{i}. {model_id}: {avg_time:.0f}ms {status}")
        
        # 推奨設定
        print("\n推奨設定:")
        if sorted_results[0][1] <= 500:
            print(f"export DIAROS_LLM_MODEL={sorted_results[0][0]}")
        else:
            print("export DIAROS_LLM_MODEL=rinna-small")
            print("（デフォルト推奨）")
        
        print("\nより高速な応答が必要な場合:")
        print("export OPENAI_API_KEY='your-api-key'  # ChatGPT APIを使用")
        print("export ANTHROPIC_API_KEY='your-api-key'  # Claude APIを使用")

if __name__ == "__main__":
    main()
    print("\nEnterキーを押して続行...")
    input()