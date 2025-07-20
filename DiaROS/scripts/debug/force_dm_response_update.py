#!/usr/bin/env python3
"""
DM強制応答更新パッチ
response_update条件を緩和してNLGメッセージ送信を促進
"""

import sys
import os

# DiaROS dialogManagement.py のパス
DM_FILE_PATH = "/workspace/DiaROS/DiaROS_py/diaros/dialogManagement.py"

def backup_original():
    """元ファイルのバックアップ"""
    backup_path = DM_FILE_PATH + ".backup"
    if not os.path.exists(backup_path):
        with open(DM_FILE_PATH, 'r') as original:
            with open(backup_path, 'w') as backup:
                backup.write(original.read())
        print(f"✅ 元ファイルをバックアップしました: {backup_path}")
    else:
        print(f"✅ バックアップファイルが既に存在します: {backup_path}")

def apply_patch():
    """response_update条件の緩和パッチを適用"""
    print("🔧 DM response_update条件緩和パッチを適用中...")
    
    # バックアップ作成
    backup_original()
    
    # ファイル読み込み
    with open(DM_FILE_PATH, 'r') as f:
        content = f.read()
    
    # 既存の厳密な条件判定を緩和
    old_condition = """                # 前回response_updateがTrueになった時のASR結果と比較
                diff = list(difflib.ndiff(self.last_response_update_asr, self.asr["you"]))
                changed_chars = sum(1 for d in diff if d.startswith('+ ') or d.startswith('- '))
                # 前回response_updateがTrueになった時のASR結果と1文字以上変わった場合のみ判定
                if changed_chars >= 1 and self.asr["you"] != self.last_response_update_asr:"""
    
    new_condition = """                # 緩和された条件: ASR結果が存在し、前回と異なる場合に応答更新
                # (デバッグ用) より積極的にNLGメッセージを送信
                if self.asr["you"] and (
                    self.asr["you"] != self.last_response_update_asr or 
                    len(self.asr["you"]) >= 2  # 2文字以上で積極的に応答
                ):"""
    
    # パッチ適用
    if old_condition in content:
        content = content.replace(old_condition, new_condition)
        print("✅ 条件判定パッチを適用しました")
    else:
        print("⚠️  既存の条件判定が見つかりません。手動パッチを適用します...")
        
        # 手動パッチ：より積極的なresponse_update条件を追加
        asr_check_pattern = 'if self.asr["you"]:'
        if asr_check_pattern in content:
            # ASRチェックの直後に強制的なresponse_update条件を追加
            manual_patch = """if self.asr["you"]:
                # 🔧 強制パッチ: より積極的なresponse_update
                current_asr = self.asr["you"]
                should_update = (
                    current_asr != self.last_response_update_asr or  # 前回と異なる
                    len(current_asr) >= 2 or  # 2文字以上
                    current_asr.strip()  # 空白以外の文字が存在
                )
                
                if should_update:
                    self.word = current_asr
                    self.response_update = True
                    self.last_response_update_asr = current_asr
                    print(f"[🔧 強制パッチ] response_update=True, ASR: '{current_asr}'")
                    sys.stdout.flush()
                
                # 元の条件判定（バックアップ用）"""
            
            content = content.replace(asr_check_pattern, manual_patch)
            print("✅ 手動パッチを適用しました")
    
    # ファイル書き込み
    with open(DM_FILE_PATH, 'w') as f:
        f.write(content)
    
    print("✅ パッチ適用完了")

def restore_original():
    """元ファイルの復元"""
    backup_path = DM_FILE_PATH + ".backup"
    if os.path.exists(backup_path):
        with open(backup_path, 'r') as backup:
            with open(DM_FILE_PATH, 'w') as original:
                original.write(backup.read())
        print(f"✅ 元ファイルを復元しました")
    else:
        print(f"❌ バックアップファイルが見つかりません: {backup_path}")

def main():
    print("🔧 DM応答更新パッチツール")
    print("=" * 40)
    
    if len(sys.argv) < 2:
        print("使用方法:")
        print("  python3 force_dm_response_update.py apply   # パッチ適用")
        print("  python3 force_dm_response_update.py restore # 元に戻す")
        return
    
    command = sys.argv[1]
    
    if command == "apply":
        apply_patch()
        print("\n📋 次の手順:")
        print("1. DiaROSパッケージを再インストール:")
        print("   cd /workspace/DiaROS/DiaROS_py && python -m pip install . --user")
        print("2. DiaROSシステムを再起動")
        print("3. テストメッセージを送信してDM→NLG通信を確認")
        
    elif command == "restore":
        restore_original()
        print("\n📋 次の手順:")
        print("1. DiaROSパッケージを再インストール:")
        print("   cd /workspace/DiaROS/DiaROS_py && python -m pip install . --user")
        print("2. DiaROSシステムを再起動")
        
    else:
        print(f"❌ 不明なコマンド: {command}")
        print("apply または restore を指定してください")

if __name__ == "__main__":
    main()