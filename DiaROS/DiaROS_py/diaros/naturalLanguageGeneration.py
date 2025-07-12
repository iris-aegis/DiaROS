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

        sys.stdout.write('NaturalLanguageGeneration  start up.\n')
        sys.stdout.write('=====================================================\n')
        # OpenAI APIキーを環境変数から設定
        openai.api_key = os.environ.get("OPENAI_API_KEY")


    def update(self, words):
        # wordsはリスト型
        self.words = words
        self.query = words[0] if words else ""
        self.update_flag = True
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
                # 最新・3つ前・6つ前の履歴を使う
                query_list = self.words
                query = query_list[0] if len(query_list) > 0 else ""
                # 必要に応じて3つ前・6つ前も利用可能
                # sys.stdout.write(f"input:{text_input}\n")
                # sys.stdout.flush()
                start_time = datetime.now()
                role = "優しい性格のアンドロイドとして、相手を労るような返答を２０文字以内でしてください。"
                # openai>=1.0.0対応
                chat_response = openai.chat.completions.create(
                    model="gpt-3.5-turbo",
                    messages=[
                        {"role": "system","content": role},
                        {"role": "user","content": query},
                    ],
                )
                res = chat_response.choices[0].message.content
                sys.stdout.write("res: " + res + "\n")
                sys.stdout.flush()
                elapsed_time = datetime.now() - start_time
                sys.stdout.write(f"time: {elapsed_time.total_seconds()}秒\n")
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
                # 生成文を標準出力
                print(f"[NLG生成文] {res}")
                sys.stdout.flush()
                
                self.update_flag = False
            # last_replyが空でない場合のみros2_natural_language_generation.pyで送信される
            time.sleep(0.01)