#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.msg import Idm
import sys
import json
import os

class TestDMtoNLGSender(Node):
    def __init__(self, json_file_path):
        super().__init__('test_dm_to_nlg_sender')
        self.publisher = self.create_publisher(Idm, 'DMtoNLG', 10)
        
        # JSONファイルから音声認識結果を読み込み
        self.words_to_send = self.load_asr_results(json_file_path)
        
        if not self.words_to_send:
            print(f"❌ エラー: JSONファイル '{json_file_path}' からデータを読み込めませんでした")
            sys.exit(1)
        
        print(f"✅ JSONファイル '{json_file_path}' から{len(self.words_to_send)}件のASR結果を読み込みました")
        
        # 1秒後に送信
        self.timer = self.create_timer(1.0, self.send_message)
    
    def load_asr_results(self, json_file_path):
        """JSONファイルから音声認識結果を読み込む（シングルクォーテーション対応）"""
        try:
            # ファイルが存在するかチェック
            if not os.path.exists(json_file_path):
                print(f"❌ エラー: ファイル '{json_file_path}' が見つかりません")
                return []
            
            # ファイル内容を読み込み
            with open(json_file_path, 'r', encoding='utf-8') as f:
                content = f.read().strip()
            
            # シングルクォーテーションをダブルクォーテーションに変換
            # ただし、文字列内のシングルクォーテーションは保護する
            content_fixed = self.fix_json_quotes(content)
            
            try:
                # 修正されたJSONをパース
                data = json.loads(content_fixed)
                print(f"✅ JSONファイルを正常に読み込みました（クォート修正適用）")
            except json.JSONDecodeError:
                # 修正が失敗した場合、元のJSONで試行
                print(f"⚠️ クォート修正版でパース失敗、元のJSONで再試行中...")
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
            print(f"ファイル内容（最初の200文字）:")
            try:
                with open(json_file_path, 'r', encoding='utf-8') as f:
                    print(repr(f.read()[:200]))
            except:
                pass
            return []
        except Exception as e:
            print(f"❌ ファイル読み込みエラー: {e}")
            return []
    
    def fix_json_quotes(self, content):
        """シングルクォーテーションをダブルクォーテーションに変換"""
        import re
        
        # 基本的なシングルクォート→ダブルクォート変換
        # 文字列の開始と終了のみを対象にする簡単な変換
        
        # パターン1: 配列内の文字列 'text' -> "text"
        content = re.sub(r"'([^']*)'", r'"\1"', content)
        
        return content
        
    def send_message(self):
        msg = Idm()
        msg.words = self.words_to_send
        
        print(f"DMtoNLGトピックに送信中...")
        print(f"送信内容（全{len(self.words_to_send)}件）:")
        for i, word in enumerate(self.words_to_send):
            print(f"  [{i+1}] {word}")
        
        self.publisher.publish(msg)
        print("送信完了！")
        
        # 送信後にノードを終了
        self.timer.cancel()
        rclpy.shutdown()

def main(args=None):
    # コマンドライン引数の処理
    if len(sys.argv) < 2:
        print("使用方法: python3 test_dm_to_nlg_send.py <json_file_path>")
        print("例: python3 test_dm_to_nlg_send.py asr_result_lists/asr_result_list_2_5s_test_gpt5.json")
        print("")
        print("利用可能なJSONファイル:")
        asr_dir = "asr_result_lists"
        if os.path.exists(asr_dir):
            for file in os.listdir(asr_dir):
                if file.endswith('.json'):
                    print(f"  - {asr_dir}/{file}")
        sys.exit(1)
    
    json_file_path = sys.argv[1]
    
    # 相対パスの場合、現在のディレクトリからの相対パスとして解釈
    if not os.path.isabs(json_file_path):
        json_file_path = os.path.join(os.getcwd(), json_file_path)
    
    print(f"🚀 DM to NLG送信テスト開始")
    print(f"📂 対象JSONファイル: {json_file_path}")
    print("=" * 50)
    
    rclpy.init(args=args)
    
    try:
        sender = TestDMtoNLGSender(json_file_path)
        rclpy.spin(sender)
    except KeyboardInterrupt:
        print("\n⚠️ ユーザーによって中断されました")
    except Exception as e:
        print(f"❌ エラーが発生しました: {e}")
    finally:
        try:
            sender.destroy_node()
        except:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()