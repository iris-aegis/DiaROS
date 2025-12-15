#!/usr/bin/env python3
"""
NLGコンポーネントの直接テスト
"""
import sys
import os
sys.path.insert(0, '/workspace/DiaROS_py')

from diaros.naturalLanguageGeneration import NaturalLanguageGeneration

def test_nlg():
    print("[テスト] NaturalLanguageGenerationクラスの初期化開始")
    
    try:
        nlg = NaturalLanguageGeneration()
        print("[テスト] ✅ NLGクラス初期化成功")
        
        # 単体テスト1: 単一文字列入力
        print("\n[テスト1] 単一文字列入力テスト")
        nlg.update("こんにちは")
        print(f"[テスト1] 結果: '{nlg.last_reply}'")
        
        # 単体テスト2: リスト入力
        print("\n[テスト2] ASR結果リスト入力テスト")
        asr_results = ["今日は", "良い天気", "ですね"]
        nlg.update(asr_results)
        print(f"[テスト2] 結果: '{nlg.last_reply}'")
        
        # 単体テスト3: 空リスト
        print("\n[テスト3] 空リスト入力テスト")
        nlg.update([])
        print(f"[テスト3] 結果: '{nlg.last_reply}'")
        
        print("\n[テスト] ✅ 全てのテストが完了しました")
        
    except Exception as e:
        print(f"[テスト] ❌ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_nlg()