#!/usr/bin/env python3
"""
JSONファイル読み込みテスト（ROS2環境不要版）
"""

import json
import os
import sys
import re

def fix_json_quotes(content):
    """シングルクォーテーションをダブルクォーテーションに変換"""
    # パターン1: 配列内の文字列 'text' -> "text"
    content = re.sub(r"'([^']*)'", r'"\1"', content)
    return content

def load_asr_results(json_file_path):
    """JSONファイルから音声認識結果を読み込む（シングルクォーテーション対応）"""
    try:
        # ファイルが存在するかチェック
        if not os.path.exists(json_file_path):
            print(f"❌ エラー: ファイル '{json_file_path}' が見つかりません")
            return []
        
        # ファイル内容を読み込み
        with open(json_file_path, 'r', encoding='utf-8') as f:
            content = f.read().strip()
        
        print(f"📄 元のファイル内容:")
        print(content[:200] + "..." if len(content) > 200 else content)
        print()
        
        # シングルクォーテーションをダブルクォーテーションに変換
        content_fixed = fix_json_quotes(content)
        
        print(f"🔧 修正後の内容:")
        print(content_fixed[:200] + "..." if len(content_fixed) > 200 else content_fixed)
        print()
        
        try:
            # 修正されたJSONをパース
            data = json.loads(content_fixed)
            print(f"✅ JSONファイルを正常に読み込みました（クォート修正適用）")
        except json.JSONDecodeError as e:
            # 修正が失敗した場合、元のJSONで試行
            print(f"⚠️ クォート修正版でパース失敗: {e}")
            print(f"元のJSONで再試行中...")
            data = json.loads(content)
            print(f"✅ 元のJSONファイルで正常に読み込みました")
        
        # データがリスト形式かチェック
        if isinstance(data, list):
            print(f"📄 JSONファイル形式: リスト（{len(data)}項目）")
            return data
        elif isinstance(data, dict):
            # 辞書形式の場合、適切なキーを探す
            possible_keys = ['words', 'results', 'asr_results', 'texts', 'data']
            for key in possible_keys:
                if key in data and isinstance(data[key], list):
                    print(f"📄 JSONファイル形式: 辞書（キー: '{key}'、{len(data[key])}項目）")
                    return data[key]
            
            print(f"❌ エラー: 辞書形式のJSONですが、適切なキー（{possible_keys}）が見つかりません")
            print(f"利用可能なキー: {list(data.keys())}")
            return []
        else:
            print(f"❌ エラー: サポートされていないJSON形式です（型: {type(data)}）")
            return []
            
    except json.JSONDecodeError as e:
        print(f"❌ JSONパースエラー: {e}")
        return []
    except Exception as e:
        print(f"❌ ファイル読み込みエラー: {e}")
        return []

def main():
    if len(sys.argv) < 2:
        print("使用方法: python3 test_json_loading.py <json_file_path>")
        print("例: python3 test_json_loading.py asr_result_lists/asr_result_list_1s.json")
        print("")
        print("利用可能なJSONファイル:")
        asr_dir = "asr_result_lists"
        if os.path.exists(asr_dir):
            for file in os.listdir(asr_dir):
                if file.endswith('.json'):
                    print(f"  - {asr_dir}/{file}")
        sys.exit(1)
    
    json_file_path = sys.argv[1]
    
    print(f"🧪 JSON読み込みテスト")
    print(f"📂 対象ファイル: {json_file_path}")
    print("=" * 50)
    
    words = load_asr_results(json_file_path)
    
    if words:
        print(f"\n✅ 読み込み成功！{len(words)}件のデータを取得:")
        for i, word in enumerate(words):
            print(f"  [{i+1:2d}] '{word}'")
    else:
        print(f"\n❌ 読み込み失敗")

if __name__ == '__main__':
    main()