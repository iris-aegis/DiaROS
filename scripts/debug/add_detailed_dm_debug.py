#!/usr/bin/env python3
"""
DM条件判定詳細デバッグ追加ツール
response_update条件の各段階を詳細にログ出力
"""

import sys
import os

DM_FILE_PATH = "/workspace/DiaROS_py/diaros/dialogManagement.py"

def add_detailed_debug():
    """詳細デバッグログを追加"""
    print("🔍 DM条件判定詳細デバッグを追加中...")
    
    # バックアップ作成
    backup_path = DM_FILE_PATH + ".debug_backup"
    if not os.path.exists(backup_path):
        with open(DM_FILE_PATH, 'r') as original:
            with open(backup_path, 'w') as backup:
                backup.write(original.read())
        print(f"✅ デバッグ用バックアップ作成: {backup_path}")
    
    # ファイル読み込み
    with open(DM_FILE_PATH, 'r') as f:
        content = f.read()
    
    # 現在の簡易デバッグ版を詳細版に置き換え
    current_simple = """            # ここでNLG用にASR結果をwordにセット（詳細デバッグ追加）
            print(f"[🔍 DM-DEBUG] ASR更新チェック開始 - ASR: '{self.asr.get('you', 'None')}'")
            print(f"[🔍 DM-DEBUG] 前回response_update結果: '{self.last_response_update_asr}'")
            sys.stdout.flush()
            if self.asr["you"]:
                # 前回response_updateがTrueになった時のASR結果と比較
                diff = list(difflib.ndiff(self.last_response_update_asr, self.asr["you"]))
                changed_chars = sum(1 for d in diff if d.startswith('+ ') or d.startswith('- '))
                # 前回response_updateがTrueになった時のASR結果と1文字以上変わった場合のみ判定
                if changed_chars >= 1 and self.asr["you"] != self.last_response_update_asr:
                    self.word = self.asr["you"]
                    self.response_update = True
                    self.last_response_update_asr = self.asr["you"]  # 更新時のASR結果を保存"""
    
    detailed_version = """            # ここでNLG用にASR結果をwordにセット（超詳細デバッグ版）
            current_asr = self.asr["you"]
            print(f"[🔍 DM-DEBUG] ASR結果チェック: '{current_asr}' (type: {type(current_asr)})")
            
            if current_asr:
                print(f"[🔍 DM-DEBUG] ASR結果存在 → 条件判定開始")
                print(f"[🔍 DM-DEBUG] 前回結果: '{self.last_response_update_asr}'")
                print(f"[🔍 DM-DEBUG] 現在結果: '{current_asr}'")
                
                # 前回response_updateがTrueになった時のASR結果と比較
                diff = list(difflib.ndiff(self.last_response_update_asr, current_asr))
                changed_chars = sum(1 for d in diff if d.startswith('+ ') or d.startswith('- '))
                
                print(f"[🔍 DM-DEBUG] 文字差分: {changed_chars}文字")
                print(f"[🔍 DM-DEBUG] diff詳細: {[d for d in diff if d.startswith(('+', '-'))]}")
                
                # 条件1: 1文字以上変化
                condition1 = changed_chars >= 1
                print(f"[🔍 DM-DEBUG] 条件1 (changed_chars >= 1): {condition1}")
                
                # 条件2: 前回と完全に異なる
                condition2 = current_asr != self.last_response_update_asr
                print(f"[🔍 DM-DEBUG] 条件2 (asr != last_asr): {condition2}")
                
                # 最終判定
                final_condition = condition1 and condition2
                print(f"[🔍 DM-DEBUG] 最終条件 (condition1 AND condition2): {final_condition}")
                
                # 前回response_updateがTrueになった時のASR結果と1文字以上変わった場合のみ判定
                if final_condition:
                    self.word = current_asr
                    self.response_update = True
                    self.last_response_update_asr = current_asr  # 更新時のASR結果を保存
                    
                    print(f"[🚀 DM-DEBUG] ✅ NLGメッセージ送信条件満たした！")
                    print(f"[💡 DM内部] response_update=True, ASR: '{current_asr}'")
                    sys.stdout.flush()
                else:
                    print(f"[❌ DM-DEBUG] NLGメッセージ送信条件満たさず")
                    print(f"[❌ DM-DEBUG] response_update=False のまま")
                    sys.stdout.flush()"""
    
    # 置き換え実行
    if current_simple in content:
        content = content.replace(current_simple, detailed_version)
        print("✅ 詳細デバッグログに更新しました")
    else:
        print("⚠️  簡易デバッグコードが見つかりません")
        # 元の形式でも試行
        simple_pattern = '# ここでNLG用にASR結果をwordにセット'
        if simple_pattern in content:
            replacement = """# ここでNLG用にASR結果をwordにセット（超詳細デバッグ版）
            current_asr = self.asr["you"]
            print(f"[🔍 DM-DEBUG] ASR結果チェック: '{current_asr}' (type: {type(current_asr)})")
            sys.stdout.flush()"""
            content = content.replace(simple_pattern, replacement)
            print("✅ 基本デバッグログを追加しました")
    
    # ファイル書き込み
    with open(DM_FILE_PATH, 'w') as f:
        f.write(content)
    
    print("✅ 詳細デバッグ追加完了")

def remove_detailed_debug():
    """詳細デバッグログを削除"""
    backup_path = DM_FILE_PATH + ".debug_backup"
    if os.path.exists(backup_path):
        with open(backup_path, 'r') as backup:
            with open(DM_FILE_PATH, 'w') as original:
                original.write(backup.read())
        print(f"✅ 詳細デバッグを削除し、元のファイルを復元しました")
    else:
        print(f"❌ デバッグ用バックアップが見つかりません: {backup_path}")

def main():
    print("🔍 DM詳細デバッグツール")
    print("=" * 40)
    
    if len(sys.argv) < 2:
        print("使用方法:")
        print("  python3 add_detailed_dm_debug.py add     # 詳細デバッグ追加")
        print("  python3 add_detailed_dm_debug.py remove  # 詳細デバッグ削除")
        return
    
    command = sys.argv[1]
    
    if command == "add":
        add_detailed_debug()
        print("\n📋 次の手順:")
        print("1. DiaROSパッケージを再インストール:")
        print("   cd /workspace/DiaROS_py && python -m pip install . --user")
        print("2. DiaROSシステムを再起動")
        print("3. テストメッセージを送信して詳細ログを確認")
        print("4. 確認後、remove で詳細ログを削除")
        
    elif command == "remove":
        remove_detailed_debug()
        print("\n📋 次の手順:")
        print("1. DiaROSパッケージを再インストール:")
        print("   cd /workspace/DiaROS_py && python -m pip install . --user")
        print("2. DiaROSシステムを再起動")
        
    else:
        print(f"❌ 不明なコマンド: {command}")
        print("add または remove を指定してください")

if __name__ == "__main__":
    main()