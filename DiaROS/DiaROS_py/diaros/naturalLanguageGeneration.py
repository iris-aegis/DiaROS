# 一旦履歴諦め

import requests
import json
import sys
import os
import time
from datetime import datetime
import openai

class NaturalLanguageGeneration:
    def __init__(self):
        self.rc = { "word": "" }
        
        self.query = ""
        self.update_flag = False
        self.dialogue_history = []
        self.user_speak_is_final = False
        self.last_reply = ""  # 生成した対話文をここに格納
        self.words = ["", "", ""]  # 追加: 履歴リスト
        self.source_words = []  # 追加: 対話生成に使用した音声認識結果リスト

        sys.stdout.write('NaturalLanguageGeneration  start up.\n')
        sys.stdout.write('=====================================================\n')
        # OpenAI APIキーを環境変数から設定
        openai.api_key = os.environ.get("OPENAI_API_KEY")


    def update(self, words):
        # wordsはリスト型で受信
        self.words = words
        self.query = words[0] if words else ""
        self.update_flag = True
        
        # ★性能監視: 大量履歴の受信を記録
        now = datetime.now()
        word_count = len(words)
        timestamp = now.strftime('%H:%M:%S.%f')[:-3]
        
        if word_count > 20:  # 20個以上で大容量判定
            sys.stdout.write(f"[{timestamp}][NLG] 大容量履歴受信: {word_count}個\n")
            sys.stdout.flush()
        
        # 最初の3個と最後の3個のみを表示（中間は省略）
        if word_count > 6:
            preview_words = words[:3] + ["..."] + words[-3:]
            sys.stdout.write(f"[{timestamp}][NLG] 履歴受信（{word_count}個）: {preview_words}\n")
        else:
            sys.stdout.write(f"[{timestamp}][NLG] 履歴受信（{word_count}個）: {words}\n")
        sys.stdout.flush()
    # def generate_dialogue(self, query):
    #     sys.stdout.write('対話履歴作成\n')
    #     sys.stdout.flush()
    #     response_res = self.response(query)
    #     dialogue_res = response_res
    #     if ":" in dialogue_res:
    #         dialogue_res = dialogue_res.split(":")[1]
    #     self.dialogue_history.append("usr:" + query)
    #     self.dialogue_history.append("sys:" + dialogue_res)
    #     # self.dialogue_historyの最後から４つの要素を保存
    #     if len(self.dialogue_history) > 5:
    #         self.dialogue_history = self.dialogue_history[-4:]
    #     sys.stdout.write('対話履歴作成\n')
    #     sys.stdout.flush()
    #     return response_res
    
    def run(self):
        DEBUG = True
        response_cnt = 0
        while True:
            if self.update_flag and self.words:
                # ★性能監視: 処理開始時刻とキューサイズ記録
                process_start = datetime.now()
                timestamp = process_start.strftime('%H:%M:%S.%f')[:-3]
                word_count = len(self.words)
                
                # 最新・3つ前・6つ前の履歴を使う
                query_list = self.words
                query = query_list[0] if len(query_list) > 0 else ""
                
                sys.stdout.write(f"[{timestamp}][NLG] 対話生成開始（履歴{word_count}個）\n")
                sys.stdout.flush()
                
                # OpenAI API呼び出し
                api_start = datetime.now()
                role = "優しい性格のアンドロイドとして、相手を労るような返答を２０文字以内でしてください。"
                # openai>=1.0.0対応
                chat_response = openai.chat.completions.create(
                    model="gpt-3.5-turbo",
                    messages=[
                        {"role": "system","content": role},
                        {"role": "user","content": query},
                    ],
                )
                api_end = datetime.now()
                
                res = chat_response.choices[0].message.content
                sys.stdout.write("res: " + res + "\n")
                sys.stdout.flush()
                
                # ★性能監視: 詳細なタイミング計測
                total_elapsed = api_end - process_start
                api_elapsed = api_end - api_start
                
                sys.stdout.write(f"[{timestamp}][NLG] 処理時間詳細:\n")
                sys.stdout.write(f"  - API呼び出し: {api_elapsed.total_seconds():.3f}秒\n")
                sys.stdout.write(f"  - 総処理時間: {total_elapsed.total_seconds():.3f}秒\n")
                sys.stdout.write(f"  - 使用履歴数: {word_count}個\n")
                
                # 遅延警告
                if total_elapsed.total_seconds() > 2.0:
                    sys.stdout.write(f"⚠️  [NLG] 処理遅延警告: {total_elapsed.total_seconds():.3f}秒\n")
                
                sys.stdout.flush()
                if ":" in res:
                    res = res.split(":", 1)[1]
                if self.user_speak_is_final:
                    self.dialogue_history.append("usr:" + query)
                    self.dialogue_history.append("sys:" + res)
                    self.user_speak_is_final = False
                    if len(self.dialogue_history) > 5:
                        self.dialogue_history = self.dialogue_history[-4:]
                    sys.stdout.write('対話履歴完了\n')
                    sys.stdout.flush()
                self.last_reply = res  # ここに生成文を格納
                self.source_words = self.words.copy()  # ★対話生成に使用した音声認識結果リストを保存
                
                # ★時刻情報を設定
                response_cnt += 1
                self.request_id = response_cnt
                self.worker_name = "nlg-worker-1"
                self.start_timestamp_ns = int(process_start.timestamp() * 1_000_000_000)
                self.completion_timestamp_ns = int(api_end.timestamp() * 1_000_000_000)
                self.inference_duration_ms = api_elapsed.total_seconds() * 1000
                
                print(f"[DEBUG] NLG時刻情報設定 - ID:{self.request_id}, start:{self.start_timestamp_ns}, completion:{self.completion_timestamp_ns}")
                
                # 生成文と使用した音声認識結果を標準出力
                print(f"[NLG生成文] {res}")
                print(f"[NLG] 対話生成に使用した音声認識履歴（全{len(self.source_words)}個）:")
                for i, word in enumerate(self.source_words):
                    print(f"    [{i+1:3d}] {word}")
                sys.stdout.flush()
                
                self.update_flag = False
            # last_replyが空でない場合のみros2_natural_language_generation.pyで送信される
            time.sleep(0.01)